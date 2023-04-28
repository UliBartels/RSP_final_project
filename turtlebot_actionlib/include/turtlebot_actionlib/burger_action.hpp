#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/action/burger.hpp>

namespace turtlebot_action{

  class robot_server : public rclcpp::Node {

  private:

    rclcpp_action::Server<action_msgs::action::burger>::SharedPtr server;

  public:

    robot_server( const std::string& name );

    rclcpp_action::GoalResponse
    goal_callback(const rclcpp_action::GoalUUID& id,
    action_msgs::action::Robot::Goal::ConstSharedPtr goal);

    rclcpp_action::CancelResponse
    cancel_callback
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_msgs::action::burger>>
     goal_handle );

    void
    accept_goal
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_msgs::action::burger>>
     goal_handle );
    
  };



}

