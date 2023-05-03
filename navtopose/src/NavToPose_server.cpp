#include <NavToPose/NavToPose.hpp>


int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<nav2pose::nav2pose_server> server = std::make_shared<nav2pose::nav2pose_server>("nav2pose_server");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
