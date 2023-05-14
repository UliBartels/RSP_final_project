#include <turtlebot_actionlib/maze_action.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<turtlebot_action::action_client> client = std::make_shared<turtlebot_action::action_client>("client");

  geometry_msgs::msg::PoseStamped W1,W2;
  geometry_msgs::msg::PoseStamped start,end;
  W1.header.frame_id = "map";
  W1.pose.position.x = -0.19;
  W1.pose.position.y = -1.92;
  W1.pose.orientation.z = 0.7071;
  W1.pose.orientation.w = 0.7071;
  W2.header.frame_id = "map";
  W2.pose.position.x = 1;
  W2.pose.position.y = -1.2;
  client->call(start,end,W1,W2);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}

