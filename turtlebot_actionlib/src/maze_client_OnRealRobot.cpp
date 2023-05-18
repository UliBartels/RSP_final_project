#include <turtlebot_actionlib/maze_action.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<turtlebot_action::action_client> client = std::make_shared<turtlebot_action::action_client>("robot_client");

/* // -------------------real robot ----------------// */
  geometry_msgs::msg::PoseStamped W1,W2;
  W1.header.frame_id = "map";
  W1.pose.position.x = 0.74;
  W1.pose.position.y = 0.57;
  W1.pose.orientation.z = -.431381;
  W1.pose.orientation.w = 0.90217;
  W2.header.frame_id = "map";
  W2.pose.position.x = 0.224;
  W2.pose.position.y = -0.025;
  W2.pose.orientation.z = 0.315;
  W2.pose.orientation.w = 0.949;

  float S1 = 0.56;
  float S2 = 0.32;
  // start running the client
  client->call(S1,S2,W1,W2);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}

