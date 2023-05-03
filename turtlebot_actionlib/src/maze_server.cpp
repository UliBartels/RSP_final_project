#include <turtlebot_actionlib/maze_action.hpp>
#include <turtlebot_actionlib/burger_action.hpp>
#include <turtlebot_actionlib/waffle_action.hpp>
int main( int argc, char** argv ){

  rclcpp::init( argc, argv );

  /* rclcpp::spin( std::make_shared<turtlebot_action::action_server>("pnp") ); */

  std::shared_ptr<turtlebot_action::action_server> s1 = std::make_shared<turtlebot_action::action_server>("maze") ;
  std::shared_ptr<turtlebot_action::burger_server> s2 = std::make_shared<turtlebot_action::burger_server>("burger") ;
  std::shared_ptr<turtlebot_action::waffle_server> s3 = std::make_shared<turtlebot_action::waffle_server>("waffle") ;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(s1);
  executor.add_node(s2);
  executor.add_node(s3);

  executor.spin();

  rclcpp::shutdown();

  return 0;

}

