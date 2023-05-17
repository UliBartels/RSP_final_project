#include <NavToPose/NavToPose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2pose{

nav2pose_server::nav2pose_server( const std::string& name ):
  Node(name){
  server = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>( this,
                                                                            name,
                                                                            std::bind( &nav2pose_server::goal_callback, this, _1, _2 ),
                                                                            std::bind( &nav2pose_server::cancel_callback, this, _1 ),
                                                                            std::bind( &nav2pose_server::accept_goal, this, _1 ));
  /* std::cout << "nav2pose_server created!\n"; */


}

rclcpp_action::GoalResponse nav2pose_server::goal_callback( const rclcpp_action::GoalUUID&, nav2_msgs::action::NavigateToPose::Goal::ConstSharedPtr goal )
{
  std::cout << "Target Pose is set!\n";
  std::cout << "Position (" << goal->pose.pose.position.x
    << ", "
    << goal->pose.pose.position.y
    << ", "
    << goal->pose.pose.position.z
    << ")"
    << std::endl;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse nav2pose_server::cancel_callback( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle )
{
  std::cout << "Robot action cancel" << std::endl;
  return rclcpp_action::CancelResponse::ACCEPT;
}


  void nav2pose_server::accept_goal( const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle ){
  std::cout << "Robot Server Processing\n";

}

// ---------------------------------------------------- Client ------------------------------------------------------------
nav2pose_client::nav2pose_client( const std::string& name ):
  Node(name){
  nav2pose_client_group = this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
  client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>( this, name, nav2pose_client_group );
  /* std::cout << "nav2pose_client created!\n"; */
  client -> wait_for_action_server();
  result = 0;
  declare_parameter("result", result);
}

  void nav2pose_client::client_response_callback( rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr )
{
  /* std::cout << "nav2pose client response callback!\n"; */
  result = 2;
}

  void nav2pose_client::client_feedback_callback(
                                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle,
                                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
                                               ){
  this-> get_pose( feedback->current_pose  );

}

  void nav2pose_client::client_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& res)
{
  /* std::cout << "nav2pose client result callback!\n"; */
  result = 1;
  rclcpp::Parameter param("result", result);
  set_parameter(param);
  std::cout << "result is " << this->get_result() << std::endl;
}

  void nav2pose_client::call_server( const geometry_msgs::msg::PoseStamped& target )
  {
    /* std::cout << "Calling nav2pose_server!\n"; */
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose = target;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&nav2pose_client::client_response_callback, this, _1);
    options.feedback_callback = std::bind(&nav2pose_client::client_feedback_callback, this, _1, _2);
    options.result_callback = std::bind(&nav2pose_client::client_result_callback, this, _1);

    client->async_send_goal( goal, options );
    /* std::cout << "nav2pose_client: Goal Sent!\n"; */
    //result
    result = 0;
  }

  void nav2pose_client::get_pose( const geometry_msgs::msg::PoseStamped& Pose ) const
  {
    std::cout << "Current Pose:\n"
      << "position:\n"
      // << "seq " << Pose.header.seq << std::endl
      << "x " << Pose.pose.position.x << std::endl
      << "y " << Pose.pose.position.y << std::endl
      << "z " << Pose.pose.position.z << std::endl
      << "orientation:\n"
      << "x " << Pose.pose.orientation.x << std::endl
      << "y " << Pose.pose.orientation.y << std::endl
      << "z " << Pose.pose.orientation.z << std::endl
      << "w " << Pose.pose.orientation.w << std::endl;

  }
 int nav2pose_client::get_result() const{
   return result;
 }

}
