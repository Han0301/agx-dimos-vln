#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

constexpr char kColorInput[] = "/camera/camera/color/image_raw";
constexpr char kColorOutput[] = "/camera/camera/color/image_raw/compressed";
constexpr char kDepthInput[] = "/camera/camera/aligned_depth_to_color/image_raw";
constexpr char kDepthOutput[] =
  "/camera/camera/aligned_depth_to_color/image_raw/compressed";

template<typename T>
class LatestSlot
{
public:
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (value_) {
      ++overwritten_;
    }
    value_ = std::move(value);
    ready_.notify_one();
  }

  std::optional<T> take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(
      lock, [this]() {
        return stopped_ || value_.has_value();
      });
    if (!value_) {
      return std::nullopt;
    }
    auto value = std::move(value_);
    value_.reset();
    return value;
  }

  uint64_t take_overwritten()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto count = overwritten_;
    overwritten_ = 0;
    return count;
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    ready_.notify_one();
  }

private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<T> value_;
  uint64_t overwritten_{0};
  bool stopped_{false};
};

struct EncodedFrame
{
  std::shared_ptr<sensor_msgs::msg::CompressedImage> message;
};

struct RawFrame
{
  sensor_msgs::msg::Image::ConstSharedPtr message;
  rclcpp::Time ros_received_at;
};

struct StreamStats
{
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> encoded{0};
  std::atomic<uint64_t> published{0};
  std::atomic<uint64_t> input_overwritten{0};
  std::atomic<uint64_t> output_overwritten{0};
  std::atomic<int64_t> last_received_ns{0};
  std::atomic<int64_t> last_published_ns{0};
};

class StreamCompressor
{
public:
  StreamCompressor(
    rclcpp::Node & node,
    rclcpp::Node & output_node,
    std::string name,
    std::string input_topic,
    std::string output_topic,
    bool color,
    int color_quality,
    int depth_compression_level,
    double trace_slow_ms)
  : node_(node), name_(std::move(name)), input_topic_(std::move(input_topic)),
    output_topic_(std::move(output_topic)), color_(color),
    color_quality_(color_quality), depth_compression_level_(depth_compression_level),
    trace_slow_ms_(trace_slow_ms)
  {
    // Raw frames stay on the local SHM/UDP participant. The camera offers
    // RELIABLE, so request the same here to avoid dropping frames before the
    // callback. Compressed network output remains BEST_EFFORT so recovery
    // traffic cannot create head-of-line congestion for fresh frames.
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    // A one-frame publisher mailbox gives precedence to freshness when DDS is
    // temporarily unable to drain the writer.
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto group = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = group;
    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      input_topic_, input_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        // Establish the common time base at the local ingress boundary. RGB,
        // depth and odom all use the local ROS clock, so downstream matching
        // never mixes camera timestamps with receipt or decode timestamps.
        const auto received_at = node_.get_clock()->now();
        stats_.received.fetch_add(1, std::memory_order_relaxed);
        stats_.last_received_ns.store(
          std::chrono::steady_clock::now().time_since_epoch().count(),
          std::memory_order_relaxed);
        input_.push(RawFrame{std::move(message), received_at});
      },
      options);
    // A separate ROS context gives the network publishers their own DDS
    // participant; frames still cross only the existing in-process mailbox.
    publisher_ = output_node.create_publisher<sensor_msgs::msg::CompressedImage>(
      output_topic_, output_qos);

    encoder_thread_ = std::thread(&StreamCompressor::encode_loop, this);
    publisher_thread_ = std::thread(&StreamCompressor::publish_loop, this);
  }

  ~StreamCompressor()
  {
    stop();
  }

  void stop()
  {
    if (stopped_.exchange(true)) {
      return;
    }
    input_.stop();
    encoded_.stop();
    if (encoder_thread_.joinable()) {
      encoder_thread_.join();
    }
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }
  }

  const StreamStats & stats() const
  {
    return stats_;
  }

private:
  void encode_loop()
  {
    while (auto frame = input_.take()) {
      const auto & message = frame->message;
      const auto overwritten = input_.take_overwritten();
      stats_.input_overwritten.fetch_add(overwritten, std::memory_order_relaxed);
      const auto started = std::chrono::steady_clock::now();
      std::vector<uint8_t> encoded;
      const bool ok = color_ ? encode_color(*message, encoded) :
        encode_depth(*message, encoded);
      const auto finished = std::chrono::steady_clock::now();
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(finished - started).count();
      if (!ok) {
        continue;
      }

      auto output = std::make_shared<sensor_msgs::msg::CompressedImage>();
      output->header = message->header;
      output->header.stamp = frame->ros_received_at;
      output->format = color_ ? "jpeg" : "16UC1; compressedDepth png";
      output->data = std::move(encoded);
      stats_.encoded.fetch_add(1, std::memory_order_relaxed);
      if (trace_slow_ms_ > 0.0 && elapsed_ms > trace_slow_ms_) {
        RCLCPP_WARN(
          node_.get_logger(), "COMPRESSOR_SLOW %s stage=encode elapsed_ms=%.3f overwritten=%lu",
          name_.c_str(), elapsed_ms, static_cast<unsigned long>(overwritten));
      }
      encoded_.push(std::move(output));
    }
  }

  void publish_loop()
  {
    while (auto message = encoded_.take()) {
      const auto overwritten = encoded_.take_overwritten();
      stats_.output_overwritten.fetch_add(overwritten, std::memory_order_relaxed);
      const auto started = std::chrono::steady_clock::now();
      publisher_->publish(**message);
      const auto finished = std::chrono::steady_clock::now();
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(finished - started).count();
      stats_.published.fetch_add(1, std::memory_order_relaxed);
      stats_.last_published_ns.store(
        finished.time_since_epoch().count(), std::memory_order_relaxed);
      if (trace_slow_ms_ > 0.0 && (elapsed_ms > trace_slow_ms_ || overwritten > 0)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "COMPRESSOR_SLOW %s stage=publish elapsed_ms=%.3f overwritten=%lu",
          name_.c_str(), elapsed_ms, static_cast<unsigned long>(overwritten));
      }
    }
  }

  bool encode_color(const sensor_msgs::msg::Image & message, std::vector<uint8_t> & encoded)
  {
    if (message.encoding != "rgb8" && message.encoding != "bgr8") {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "%s: unsupported color encoding %s", name_.c_str(), message.encoding.c_str());
      return false;
    }
    cv::Mat image(
      static_cast<int>(message.height), static_cast<int>(message.width), CV_8UC3,
      const_cast<uint8_t *>(message.data.data()), static_cast<size_t>(message.step));
    cv::Mat bgr;
    if (message.encoding == "bgr8") {
      bgr = image;
    } else {
      cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
    }
    return cv::imencode(
      ".jpg", bgr, encoded,
      {cv::IMWRITE_JPEG_QUALITY, color_quality_, cv::IMWRITE_JPEG_OPTIMIZE, 0});
  }

  bool encode_depth(const sensor_msgs::msg::Image & message, std::vector<uint8_t> & encoded)
  {
    if (message.encoding != "16UC1") {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "%s: unsupported depth encoding %s", name_.c_str(), message.encoding.c_str());
      return false;
    }
    cv::Mat depth(
      static_cast<int>(message.height), static_cast<int>(message.width), CV_16UC1,
      const_cast<uint8_t *>(message.data.data()), static_cast<size_t>(message.step));
    return cv::imencode(
      ".png", depth, encoded,
      {cv::IMWRITE_PNG_COMPRESSION, depth_compression_level_,
        cv::IMWRITE_PNG_STRATEGY, cv::IMWRITE_PNG_STRATEGY_RLE});
  }

  rclcpp::Node & node_;
  std::string name_;
  std::string input_topic_;
  std::string output_topic_;
  bool color_;
  int color_quality_;
  int depth_compression_level_;
  double trace_slow_ms_;
  LatestSlot<RawFrame> input_;
  LatestSlot<std::shared_ptr<sensor_msgs::msg::CompressedImage>> encoded_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  std::thread encoder_thread_;
  std::thread publisher_thread_;
  std::atomic<bool> stopped_{false};
  StreamStats stats_;
};

}  // namespace

class CompressedRgbdNode final : public rclcpp::Node
{
public:
  CompressedRgbdNode(rclcpp::Node & output_node)
  : Node("compressed_rgbd_node")
  {
    const int color_quality = declare_parameter<int>("color_quality", 80);
    const int depth_level = declare_parameter<int>("depth_compression_level", 1);
    const double stats_period = declare_parameter<double>("stats_period_s", 10.0);
    const double trace_slow_ms = declare_parameter<double>("trace_slow_ms", 0.0);
    metrics_path_ = declare_parameter<std::string>("metrics_path", "");

    color_ = std::make_unique<StreamCompressor>(
      *this, output_node, "color", kColorInput, kColorOutput, true,
      color_quality, depth_level, trace_slow_ms);
    depth_ = std::make_unique<StreamCompressor>(
      *this, output_node, "depth", kDepthInput, kDepthOutput, false,
      color_quality, depth_level, trace_slow_ms);

    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period),
      [this]() {
        const auto & c = color_->stats();
        const auto & d = depth_->stats();
        RCLCPP_INFO(
          get_logger(),
          "compressed RGB-D: color rx=%lu enc=%lu pub=%lu in_drop=%lu out_drop=%lu; "
          "depth rx=%lu enc=%lu pub=%lu in_drop=%lu out_drop=%lu",
          static_cast<unsigned long>(c.received.load()),
          static_cast<unsigned long>(c.encoded.load()),
          static_cast<unsigned long>(c.published.load()),
          static_cast<unsigned long>(c.input_overwritten.load()),
          static_cast<unsigned long>(c.output_overwritten.load()),
          static_cast<unsigned long>(d.received.load()),
          static_cast<unsigned long>(d.encoded.load()),
          static_cast<unsigned long>(d.published.load()),
          static_cast<unsigned long>(d.input_overwritten.load()),
          static_cast<unsigned long>(d.output_overwritten.load()));
      });
    metrics_timer_ = create_wall_timer(1s, [this]() { write_metrics(); });
    write_metrics();
  }

  ~CompressedRgbdNode() override
  {
    stop_streams();
  }

  void stop_streams()
  {
    color_->stop();
    depth_->stop();
  }

private:
  static int64_t steady_now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }

  static double stream_hz(uint64_t current, uint64_t previous, double seconds)
  {
    return seconds > 0.0 ?
      static_cast<double>(current - previous) / seconds : 0.0;
  }

  static double stream_age_sec(int64_t timestamp_ns, int64_t now_ns)
  {
    if (timestamp_ns <= 0) {
      return -1.0;
    }
    return std::max(0.0, static_cast<double>(now_ns - timestamp_ns) / 1.0e9);
  }

  void write_metrics()
  {
    if (metrics_path_.empty()) {
      return;
    }
    const auto now_ns = steady_now_ns();
    const double elapsed = metrics_previous_ns_ > 0 ?
      static_cast<double>(now_ns - metrics_previous_ns_) / 1.0e9 : 0.0;
    const auto & c = color_->stats();
    const auto & d = depth_->stats();
    const uint64_t color_received = c.received.load();
    const uint64_t depth_received = d.received.load();
    const uint64_t color_published = c.published.load();
    const uint64_t depth_published = d.published.load();

    try {
      const std::filesystem::path path(metrics_path_);
      if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
      }
      const auto temporary = path.string() + ".tmp." + std::to_string(getpid());
      std::ofstream output(temporary, std::ios::trunc);
      if (!output) {
        metrics_previous_ns_ = now_ns;
        metrics_previous_c_.received = color_received;
        metrics_previous_c_.published = color_published;
        metrics_previous_d_.received = depth_received;
        metrics_previous_d_.published = depth_published;
        return;
      }
      output << std::fixed << std::setprecision(3)
             << "{\"schema\":1,\"group\":\"D455\","
             << "\"updated_at\":" << std::setprecision(6)
             << std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count()
             << ",\"owner_pid\":" << getpid()
             << ",\"raw_rx_hz\":{\"color\":" << stream_hz(
        color_received, metrics_previous_c_.received, elapsed)
             << ",\"depth\":" << stream_hz(
        depth_received, metrics_previous_d_.received, elapsed) << "}"
             << ",\"compressed_tx_hz\":{\"color\":" << stream_hz(
        color_published, metrics_previous_c_.published, elapsed)
             << ",\"depth\":" << stream_hz(
        depth_published, metrics_previous_d_.published, elapsed) << "}"
             << ",\"raw_age_sec\":{\"color\":" << stream_age_sec(
        c.last_received_ns.load(), now_ns)
             << ",\"depth\":" << stream_age_sec(d.last_received_ns.load(), now_ns) << "}"
             << ",\"compressed_age_sec\":{\"color\":" << stream_age_sec(
        c.last_published_ns.load(), now_ns)
             << ",\"depth\":" << stream_age_sec(d.last_published_ns.load(), now_ns) << "}"
             << ",\"counters\":{\"color\":{\"received\":" << color_received
             << ",\"encoded\":" << c.encoded.load()
             << ",\"published\":" << color_published
             << ",\"input_overwritten\":" << c.input_overwritten.load()
             << ",\"output_overwritten\":" << c.output_overwritten.load() << "},"
             << "\"depth\":{\"received\":" << depth_received
             << ",\"encoded\":" << d.encoded.load()
             << ",\"published\":" << depth_published
             << ",\"input_overwritten\":" << d.input_overwritten.load()
             << ",\"output_overwritten\":" << d.output_overwritten.load() << "}}}";
      output.close();
      std::rename(temporary.c_str(), path.c_str());
    } catch (const std::filesystem::filesystem_error &) {
      // Telemetry must never interfere with image processing.
    }

    metrics_previous_c_.received = color_received;
    metrics_previous_c_.published = color_published;
    metrics_previous_d_.received = depth_received;
    metrics_previous_d_.published = depth_published;
    metrics_previous_ns_ = now_ns;
  }

  struct PreviousCounters
  {
    uint64_t received{0};
    uint64_t published{0};
  };

  std::string metrics_path_;
  std::unique_ptr<StreamCompressor> color_;
  std::unique_ptr<StreamCompressor> depth_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  int64_t metrics_previous_ns_{0};
  PreviousCounters metrics_previous_c_;
  PreviousCounters metrics_previous_d_;
};

int main(int argc, char ** argv)
{
  // Keep both contexts alive until publisher/encoder threads have joined.
  // The signal handler only sets a flag; shutdown is ordered on this thread.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  auto input_context = rclcpp::contexts::get_global_default_context();
  auto output_context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions output_init;
  output_init.set_domain_id(input_context->get_domain_id());
  output_context->init(argc, argv, output_init);
  rclcpp::NodeOptions output_options;
  output_options.context(output_context)
    // Preserve topic/namespace remaps, but do not inherit the RX node name.
    .arguments({"--ros-args", "-r", "__node:=compressed_rgbd_tx"})
    .enable_rosout(false)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false);
  auto output_node = std::make_shared<rclcpp::Node>("compressed_rgbd_tx", output_options);
  auto node = std::make_shared<CompressedRgbdNode>(*output_node);
  RCLCPP_INFO(node->get_logger(), "Separate RX/TX DDS contexts enabled; domain=%zu",
    input_context->get_domain_id());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::exception_ptr executor_error;
  std::atomic<bool> executor_finished{false};
  std::thread executor_thread([&]() {
    try {
      executor.spin();
    } catch (...) {
      executor_error = std::current_exception();
    }
    executor_finished.store(true);
  });
  while (!stop_requested && !executor_finished.load() && input_context->is_valid()) {
    std::this_thread::sleep_for(50ms);
  }
  executor.cancel();
  executor_thread.join();
  executor.remove_node(node);
  node->stop_streams();
  node.reset();
  output_node.reset();
  output_context->shutdown("compression threads stopped");
  rclcpp::shutdown();
  if (executor_error) {
    std::rethrow_exception(executor_error);
  }
  return 0;
}
