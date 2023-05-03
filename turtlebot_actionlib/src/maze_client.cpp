#include <turtlebot_actionlib/maze_action.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<turtlebot_action::action_client> client = std::make_shared<turtlebot_action::action_client>("client");

  geometry_msgs::msg::PoseStamped P1,P2,P3,P4;
  geometry_msgs::msg::PoseStamped W1,W2;
  geometry_msgs::msg::PoseStamped start,end;

  client->call(start,end,P1,P2,P3,P4,W1,W2);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}

