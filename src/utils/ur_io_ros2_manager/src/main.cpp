#include "ur_io_ros2_manager/ur_io_ros2_manager.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto ur_interface = std::make_shared<UrIoInterface>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ur_interface);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

