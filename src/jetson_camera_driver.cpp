/*
Copyright (c) 2022 Kyuhyong You

Code based on Andreas Klintberg's usb_camera_driver implementation for ROS2
*/


#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "jetson_camera_driver.hpp"

using namespace std::chrono_literals;

namespace jetson_camera_driver
{
std::string mat_type2encoding(int mat_type)
{
    switch (mat_type) {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("unsupported encoding type");
    }
}
std::string get_tegra_pipeline(int width, int height, int fps) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
        std::to_string(height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(fps) +
        "/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

JetsonCameraDriver::JetsonCameraDriver(const rclcpp::NodeOptions &node_options) : 
    Node("jetson_camera_driver", node_options),
    canceled_(false)
{
    RCLCPP_INFO(get_logger(), "Node start");
    frame_id_ = this->declare_parameter("frame_id", "camera");
    image_width_ = this->declare_parameter("image_width", 1280);
    image_height_ = this->declare_parameter("image_height", 720);
    width_ = this->declare_parameter("width", 480);
    height_ = this->declare_parameter("height", 60);
    fps_ = this->declare_parameter("fps", 60.0);
    camera_id = this->declare_parameter("camera_id", 0);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    pub_camera_transport_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    //pipeline_ = gstreamer_pipeline(image_width_, image_height_, width_, height_, fps_, flip_method_);
    pipeline_ = get_tegra_pipeline(image_width_, image_height_, fps_);
    
#ifdef USE_LOOP_THREAD
    RCLCPP_INFO(get_logger(), "Pipeline start");
    //capture_ = std::make_shared<cv::VideoCapture>(camera_id);
    //capture_->open(pipeline_, cv::CAP_GSTREAMER);
    next_stamp_ = now();
    RCLCPP_INFO(get_logger(), "Loop bind");
    thread_ = std::thread(std::bind(&JetsonCameraDriver::LoopThread, this));
    RCLCPP_INFO(get_logger(), "start publishing");
#else
    cap_.open(pipeline_, cv::CAP_GSTREAMER);
    last_frame_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(1ms, std::bind(&JetsonCameraDriver::ImageCallback, this));
#endif
}

JetsonCameraDriver::~JetsonCameraDriver()
{
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
}

#ifdef USE_LOOP_THREAD
void JetsonCameraDriver::LoopThread(void)
{
    std::string pipeline = get_tegra_pipeline(image_width_, image_height_, fps_);
    RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline.c_str());
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    //cv::Mat frame;
    cv_bridge::CvImage img;
    img.header.frame_id = frame_id_;
    img.encoding = sensor_msgs::image_encodings::BGR8;
    RCLCPP_INFO(get_logger(), "LoopThread Start!!");
    
    while (rclcpp::ok() && !canceled_.load()) {
        if (cap.read(img.image)) {
            
            auto stamp = now();
            
            // Convert OpenCV Mat to ROS Image
            img.header.stamp = stamp;
            //img.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            //img.data.assign(frame.datastart, frame.dataend);
            pub_image_->publish(*img.toImageMsg());
        }
    }

}
#else
void JetsonCameraDriver::ImageCallback()
{
    //std::string pipeline = gstreamer_pipeline(image_width_, image_height_, width_, height_, fps_, flip_method_);
    //cv::VideoCapture cap(pipeline_, cv::CAP_GSTREAMER);
    cap_ >> frame;
    //cap_.read(frame);
    cv_bridge::CvImagePtr cv_prt;
    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/fps_*1000)
    {
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                .toImageMsg();
        
        pub_image_->publish(*msg.get());
        last_frame_ = now;

        // Convert to a ROS2 image
        if (!is_flipped)
        {
            msg_image_ = ConvertFrameToMessage(frame);
        }
        else
        {
            // Flip the frame if needed
            //cv::flip(frame, flipped_frame, 1);
            msg_image_ = ConvertFrameToMessage(frame);
        }

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        msg_image_->header.stamp = timestamp;
        msg_image_->header.frame_id = frame_id_;

        msg_camera_info_->header.stamp = timestamp;
        msg_camera_info_->header.frame_id = frame_id_;

        pub_camera_info_.publish(msg_image_, msg_camera_info_);
    }
}
#endif


} // namespace jetson_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_camera_driver::JetsonCameraDriver)