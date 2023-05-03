#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace nav2pose{

class nav2pose_server : public rclcpp::Node{

private:
  rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr server;

public:

  nav2pose_server( const std::string& name );

  // goal response declaration
  rclcpp_action::GoalResponse goal_callback( const rclcpp_action::GoalUUID&, nav2_msgs::action::NavigateToPose::Goal::ConstSharedPtr goal );
  // cancel response declaration
  rclcpp_action::CancelResponse
  cancel_callback( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle );
  // accept goal declaration
  void accept_goal( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle );


};


class nav2pose_client : public rclcpp::Node{

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;

public:
  nav2pose_client( const std::string& name );

  void call_server( const geometry_msgs::msg::PoseStamped& target );
  void client_response_callback( rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle );
  void client_feedback_callback(
                                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle,
                                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
                                );
  void client_result_callback( const
                              rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result
 );
};
};
