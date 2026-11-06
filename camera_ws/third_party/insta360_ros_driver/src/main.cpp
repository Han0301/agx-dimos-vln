#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    // Stream 0 is the original combined dual-fisheye stream consumed by the
    // upstream decoder/equirectangular nodes.
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    // Keep the SDK's second stream available without changing the upstream
    // panorama topic or its processing path.
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_aux_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/image/compressed",
            rclcpp::QoS(10)
        );
        compressed_aux_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/back/image/compressed",
            rclcpp::QoS(10)
        );

        // Publisher for IMU data (remains the same)
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
        if (stream_index == 0) {
            publisher = compressed_pub_;
        } else if (stream_index == 1) {
            publisher = compressed_aux_pub_;
        } else {
            return;
        }

        if (size > 0 && publisher) {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

            // Set the header
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = stream_index == 0 ? "camera_frame" : "camera_aux";

            // Set the format to H.264
            // The subscriber will need to know this to select the correct decoder.
            msg->format = "h264";

            // Copy the compressed video data directly into the message
            msg->data.assign(data, data + size);

            publisher->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            
            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    // The SDK invokes the stream delegate asynchronously after
    // StartLiveStreaming returns.  Keep it alive for the full camera
    // lifetime; a local shared_ptr here leaves the SDK with a dangling
    // callback target as soon as run_camera() returns.
    std::shared_ptr<ins_camera::StreamDelegate> delegate_;
    std::shared_ptr<rclcpp::Node> node_;

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    ~CameraWrapper() {
        if (cam) {
            cam->Close();
        }
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");

        if (!cam->SetActiveSensor(ins_camera::SENSOR_DEVICE_ALL)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to enable both camera sensors.");
            cam->Close();
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Both camera sensors enabled.");

        // Keep the two lenses on the same fixed white-balance preset.  Auto
        // white balance can change while streaming and makes the panorama's
        // colour discontinuity move over time.
        const int white_balance_kelvin =
            node_->declare_parameter<int>("white_balance_kelvin", 5000);
        ins_camera::PhotographyOptions_WhiteBalance white_balance;
        switch (white_balance_kelvin) {
            case 2700:
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_2700K;
                break;
            case 4000:
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_4000K;
                break;
            case 5000:
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_5000K;
                break;
            case 6500:
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_6500K;
                break;
            case 7500:
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_7500K;
                break;
            default:
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Unsupported white_balance_kelvin=%d; using fixed 5000K. "
                    "Supported values: 2700, 4000, 5000, 6500, 7500.",
                    white_balance_kelvin);
                white_balance = ins_camera::PhotographyOptions_WhiteBalance::WB_5000K;
                break;
        }

        const auto set_fixed_white_balance =
            [this, white_balance](ins_camera::CameraFunctionMode mode,
                                  const char* mode_name) {
                auto settings = cam->GetCaptureSettings(mode);
                if (!settings) {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Could not read capture settings for %s mode.", mode_name);
                    return false;
                }

                settings->SetWhiteBalance(white_balance);
                if (!cam->SetCaptureSettings(mode, settings)) {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Could not set fixed white balance for %s mode.", mode_name);
                    return false;
                }

                auto applied_settings = cam->GetCaptureSettings(mode);
                if (!applied_settings ||
                    applied_settings->WhiteBalance() != white_balance) {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "White-balance readback did not match for %s mode.", mode_name);
                    return false;
                }
                return true;
            };

        const bool live_wb_set = set_fixed_white_balance(
            ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM,
            "live-stream");
        const bool video_wb_set = set_fixed_white_balance(
            ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_VIDEO,
            "normal-video");
        if (live_wb_set || video_wb_set) {
            const int applied_kelvin =
                white_balance == ins_camera::PhotographyOptions_WhiteBalance::WB_2700K ? 2700 :
                white_balance == ins_camera::PhotographyOptions_WhiteBalance::WB_4000K ? 4000 :
                white_balance == ins_camera::PhotographyOptions_WhiteBalance::WB_6500K ? 6500 :
                white_balance == ins_camera::PhotographyOptions_WhiteBalance::WB_7500K ? 7500 : 5000;
            RCLCPP_INFO(
                node_->get_logger(),
                "Fixed camera white balance applied: %dK (live=%s, video=%s).",
                applied_kelvin,
                live_wb_set ? "ok" : "unsupported",
                video_wb_set ? "ok" : "unsupported");
        } else {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Camera rejected fixed white balance; automatic white balance may remain active.");
        }

        discovery.FreeDeviceDescriptors(list);

        delegate_ = std::make_shared<TestStreamDelegate>(node_);
        cam->SetStreamDelegate(delegate_);

        auto start = time(NULL);

        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0; //no offset from UTC

        cam->SyncLocalTimeToCamera(utc_time,offset_time);       
        ins_camera::LiveStreamParam param;
        // X3 supports 3840x1920 or 1440x720 for panoramic preview.
        // 1920x960 is not supported by the X3 in the bundled CameraSDK and
        // makes the camera fall back to two separate 2880x2880 streams.
        param.video_resolution = ins_camera::VideoResolution::RES_1440_720P30;
        //Possible resolutions (results may vary per model) are:
        //RES_3840_1920P30
        //RES_2560_1280P30
        //RES_1152_1152P30 (this will give 2304 x 1152 at 30 FPS)
        //RES_1920_960P30  
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_publisher");
    
    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
