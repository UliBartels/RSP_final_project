#include <turtlebot_actionlib/maze_action.hpp>
#include <turtlebot_actionlib/burger_action.hpp>
#include <turtlebot_actionlib/waffle_action.hpp>
int main( int argc, char** argv ){

  rclcpp::init( argc, argv );


  std::shared_ptr<turtlebot_action::action_server> s1 = std::make_shared<turtlebot_action::action_server>("maze") ;
  std::shared_ptr<turtlebot_action::burger_server> s2 = std::make_shared<turtlebot_action::burger_server>("burger") ;
  std::shared_ptr<turtlebot_action::waffle_server> s3 = std::make_shared<turtlebot_action::waffle_server>("waffle") ;
  std::shared_ptr<turtlebot_action::subscriber> s4 = std::make_shared<turtlebot_action::subscriber>("aruco_camera") ;
  std::shared_ptr<nav2pose::nav2pose_client> c1 = std::make_shared<nav2pose::nav2pose_client>("navigate_to_pose");

  s3->NavToPose_client = c1;
  s1->Aruco_subscriber = s4;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(s1);
  executor.add_node(s2);
  executor.add_node(s3);
  executor.add_node(s4);
  executor.add_node(c1);

  executor.spin();

  rclcpp::shutdown();

  return 0;

}

