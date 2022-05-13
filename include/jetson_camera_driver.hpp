
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "camera_calibration_parsers/parse.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"

#define USE_LOOP_THREAD

namespace jetson_camera_driver
{
    class JetsonCameraDriver : public rclcpp::Node
    {
#ifdef USE_LOOP_THREAD
        std::thread thread_;
        std::atomic<bool> canceled_;
#endif
    public:
        explicit JetsonCameraDriver(const rclcpp::NodeOptions &);
        virtual ~JetsonCameraDriver();
    private:  
        // Publisher used for intra process comm
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       pub_image_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  pub_camera_info_;

        // Publisher used for inter process comm
        image_transport::CameraPublisher                            pub_camera_transport_;

        sensor_msgs::msg::CameraInfo::SharedPtr             camerainfo_ptr_;
        sensor_msgs::msg::Image::SharedPtr                  image_;
        
        int                 cap_width_, cap_height_;
	    int                 width_, height_, framerate_;
	    int                 flip_method_;
        rclcpp::TimerBase::SharedPtr                        timer_;
        
        bool                is_flipped;
	    std::string         frame_id_;
        std::string         pipeline_;
        
        int                 image_height_;
        int                 image_width_;
        double              fps_;
        int                 camera_id;
        double              delay_;
        
        std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
        image_transport::CameraPublisher                        camera_pub_;
        std::shared_ptr<sensor_msgs::msg::Image>                msg_image_;

        
#ifdef USE_LOOP_THREAD
        void LoopThread();
        rclcpp::Time                                            next_stamp_;
        std::shared_ptr<cv::VideoCapture>                       capture_;
        sensor_msgs::msg::CameraInfo                            msg_camera_info_;
#else
        void ImageCallback();
        //std::chrono::steady_clock::time_point last_frame_;        
        cv::Mat             frame;
        cv::VideoCapture    cap_;
#endif
    };
}   // namespace jetson_camera_driver