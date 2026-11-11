#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class SurroundViewsNode : public rclcpp::Node
{
public:
  SurroundViewsNode()
  : Node("surround_views")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/equirectangular/image");
    output_width_ = declare_parameter<int>("output_width", 640);
    output_height_ = declare_parameter<int>("output_height", 480);
    horizontal_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 90.0);
    vertical_fov_deg_ = declare_parameter<double>("vertical_fov_deg", 60.0);

    if (output_width_ <= 0 || output_height_ <= 0 ||
        horizontal_fov_deg_ <= 0.0 || horizontal_fov_deg_ >= 180.0 ||
        vertical_fov_deg_ <= 0.0 || vertical_fov_deg_ >= 180.0) {
      throw std::invalid_argument("Invalid surround view size or field of view");
    }

    const auto qos = rclcpp::QoS(1).reliable();
    const std::array<std::string, 4> topics = {
      "/surround/front/image", "/surround/right/image",
      "/surround/back/image", "/surround/left/image"};
    for (const auto & topic : topics) {
      publishers_.push_back(create_publisher<sensor_msgs::msg::Image>(topic, qos));
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, qos,
      std::bind(&SurroundViewsNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing four rectilinear views from %s at %dx%d (HFOV %.1f°, VFOV %.1f°)",
      input_topic_.c_str(), output_width_, output_height_, horizontal_fov_deg_, vertical_fov_deg_);
  }

private:
  void buildMaps(int source_width, int source_height)
  {
    maps_.clear();
    maps_.reserve(4);

    const double hfov = horizontal_fov_deg_ * M_PI / 180.0;
    const double vfov = vertical_fov_deg_ * M_PI / 180.0;
    const double fx = (output_width_ - 1) / (2.0 * std::tan(hfov / 2.0));
    const double fy = (output_height_ - 1) / (2.0 * std::tan(vfov / 2.0));
    const double cx = (output_width_ - 1) / 2.0;
    const double cy = (output_height_ - 1) / 2.0;

    // Front, right, back, left. The source equirectangular image uses
    // longitude -pi..pi from left to right and latitude pi/2..-pi/2 top to bottom.
    const std::array<double, 4> yaws = {0.0, M_PI / 2.0, M_PI, -M_PI / 2.0};
    for (double yaw : yaws) {
      cv::Mat map_x(output_height_, output_width_, CV_32FC1);
      cv::Mat map_y(output_height_, output_width_, CV_32FC1);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);

      for (int v = 0; v < output_height_; ++v) {
        for (int u = 0; u < output_width_; ++u) {
          const double local_x = (u - cx) / fx;
          const double local_y = -(v - cy) / fy;
          const double local_z = 1.0;
          const double world_x = c * local_x + s * local_z;
          const double world_y = local_y;
          const double world_z = -s * local_x + c * local_z;
          const double longitude = std::atan2(world_x, world_z);
          const double norm = std::sqrt(
            world_x * world_x + world_y * world_y + world_z * world_z);
          const double latitude = std::asin(world_y / norm);

          double source_x = (longitude / (2.0 * M_PI) + 0.5) * source_width;
          source_x = std::fmod(source_x, static_cast<double>(source_width));
          if (source_x < 0.0) {
            source_x += source_width;
          }
          const double source_y = (0.5 - latitude / M_PI) * source_height;
          map_x.at<float>(v, u) = static_cast<float>(source_x);
          map_y.at<float>(v, u) = static_cast<float>(
            std::clamp(source_y, 0.0, static_cast<double>(source_height - 1)));
        }
      }
      maps_.push_back(std::move(map_x));
      maps_.push_back(std::move(map_y));
    }

    source_width_ = source_width;
    source_height_ = source_height;
    RCLCPP_INFO(get_logger(), "Built surround projection maps for %dx%d input", source_width, source_height);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      const auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      const cv::Mat & source = cv_ptr->image;
      if (source.empty()) {
        return;
      }
      if (source.cols != source_width_ || source.rows != source_height_) {
        buildMaps(source.cols, source.rows);
      }

      for (std::size_t i = 0; i < 4; ++i) {
        cv::Mat view;
        // Wrap horizontally so a view crossing the -pi/pi seam remains continuous.
        cv::remap(source, view, maps_[2 * i], maps_[2 * i + 1], cv::INTER_LINEAR,
                  cv::BORDER_WRAP);
        auto out = cv_bridge::CvImage(msg->header, "bgr8", view).toImageMsg();
        publishers_[i]->publish(*out);
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "surround view: %s", e.what());
    }
  }

  std::string input_topic_;
  int output_width_ = 640;
  int output_height_ = 480;
  double horizontal_fov_deg_ = 90.0;
  double vertical_fov_deg_ = 60.0;
  int source_width_ = 0;
  int source_height_ = 0;
  std::vector<cv::Mat> maps_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurroundViewsNode>());
  rclcpp::shutdown();
  return 0;
}
