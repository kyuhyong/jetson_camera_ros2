
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

//#include <vector>
//#include <string>
//#include <unordered_map>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace jetson_camera_driver
{
    class JetsonCameraDriver : public rclcpp::Node
    {
    public:
        explicit JetsonCameraDriver(const rclcpp::NodeOptions &);
        virtual ~JetsonCameraDriver() {};
    private:
        static std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
		    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
		       std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
		       "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
		       std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	    }
        // Publisher used for intra process comm
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pub_;

        // Publisher used for inter process comm
        image_transport::CameraPublisher publisher_;

        sensor_msgs::msg::CameraInfo::SharedPtr camerainfo_;
        sensor_msgs::msg::Image::SharedPtr image_;

        int         cap_width_, cap_height_;
	    int         width_, height_, framerate_;
	    int         flip_method_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::Mat     frame;
        bool        is_flipped;
	    std::string frame_id_;
        std::string pipeline_;
        cv::VideoCapture cap_;
        int         image_height_;
        int         image_width_;
        double      fps_;
        int         camera_id;
	    
        double      delay_;
        
        
        std::chrono::steady_clock::time_point last_frame_;

        std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
        image_transport::CameraPublisher camera_info_pub_;
    
        std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    
        std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame);

        void ImageCallback();
    };
}   // namespace jetson_camera_driver