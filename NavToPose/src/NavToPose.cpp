#include <NavToPose/NavToPose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

nav2pose_server::nav2pose( const std::string& name ):
  Node(name){

}

rclcpp_action::GoalResponse nav2pose_server::goal_callback( const rclcpp_action::GoalUUID&, nav2_msgs::action::NavigateToPose::Goal::ConstSharedPtr goal )
{
  std::cout << "Target Pose is set!\n";
  std::cout << "Position (" << goal->pose.position.x
    << goal->pose.position.y
    << goal->pose.position.z
    << std::endl;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse nav2pose_server::cancel_callback( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle )
{
  std::cout << "Robot action cancel" << std::endl;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void accept_goal( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle ){
  std::cout << "Robot Server Processing\n";

}

// ---------------------------------------------------- Client ------------------------------------------------------------
nav2pose_client::nav2pose_client( const std::string& name ):
  Node(name){
  nav2pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>( this, "nav2pose_client" );
  nav2pose_client -> wait_for_action_server();
}

void nav2pose_client::call_server( const geometry_msgs::msg::PoseStamped& goal )
{
  std::cout << "Calling nav2pose_server!\n";

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = std::bind(this->client_response_callback, this, _1);
  options.feedback_callback = std::bind(this->client_feedback_callback, this, _1, _2);
  options.result_callback = std::bind(this->client_result_callback, this, _1);

  nav2pose_client->async_send_goal( goal, options );
  std::cout << "nav2pose_client: Goal Sent!\n";

}

void nav2pose_client::client_response_callback( rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle )
{
  std:cout << "nav2pose client response callback!\n";
}

void nav2pose_client::client_feedback_callback(
                                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle,
                                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
                                               ){
  std:cout << "nav2pose client feedback callback!\n";

}

void nav2pose_client::client_result_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
  std:cout << "nav2pose client result callback!\n";
}
