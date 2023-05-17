#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <maze_msgs/action/waffle.hpp>
#include <NavToPose/NavToPose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace turtlebot_action{

  class waffle_server : public rclcpp::Node {

  private:
    
    using WaffleAction = maze_msgs::action::Waffle;

    rclcpp_action::Server<WaffleAction>::SharedPtr server;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cmd_publisher;
  public:

    waffle_server( const std::string& name );

    std::shared_ptr<nav2pose::nav2pose_client> NavToPose_client;

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
    void teleop_publish(const std::string& command);
  };
}
