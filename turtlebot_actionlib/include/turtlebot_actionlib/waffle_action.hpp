#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <maze_msgs/action/waffle.hpp>
#include <NavToPose/NavToPose.hpp>

namespace turtlebot_action{

  class waffle_server : public rclcpp::Node {

  private:
    
    using WaffleAction = maze_msgs::action::Waffle;

    rclcpp_action::Server<WaffleAction>::SharedPtr server;
    std::shared_ptr<nav2pose::nav2pose_client> NavToPose_client;
  public:

    waffle_server( const std::string& name );

    rclcpp_action::GoalResponse
    goal_callback(const rclcpp_action::GoalUUID& id,
    WaffleAction::Goal::ConstSharedPtr goal);

    rclcpp_action::CancelResponse
    cancel_callback
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaffleAction>>
     goal_handle );

    void
    accept_goal
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaffleAction>>
     goal_handle );


    // Call NavtoPose
    void NavToPose_call(const geometry_msgs::msg::PoseStamped& target);
    void NavToPose_response_callback( rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle );
    void NavToPose_feedback_callback(
                                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle,
                                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
                                );
    void NavToPose_result_callback( const
                              rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result
 );

    void get_pose( const geometry_msgs::msg::PoseStamped& Pose ) const;
  };
}
