#include <NavToPose/NavToPose.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<nav2pose_client> client = std::make_shared<nav2pose_client>("nav2pose_client");
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.frame_id = "map";
  pose1.pose.position.x = 1.62;
  pose1.pose.position.y = -1.84;
  client->call_server( pose1 );
  rclcpp::spin(client);
  rclcpp::shutdown();


  return 0;
}
