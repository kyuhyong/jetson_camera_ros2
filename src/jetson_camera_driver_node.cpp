#include "jetson_camera_driver.hpp"
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  printf("hello world jetson_camera_ros package\n");

  rclcpp::init(argc, argv);
  //rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::executors::SingleThreadedExecutor exec;

  //const rclcpp::NodeOptions options;
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);
  auto jetson_camera_node = std::make_shared<jetson_camera_driver::JetsonCameraDriver>(options);
  
  exec.add_node(jetson_camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
