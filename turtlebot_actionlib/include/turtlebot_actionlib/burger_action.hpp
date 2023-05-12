#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <maze_msgs/action/burger.hpp>

namespace turtlebot_action{

  class burger_server : public rclcpp::Node {

  private:

    using BurgerAction = maze_msgs::action::Burger;
    rclcpp_action::Server<BurgerAction>::SharedPtr server;

  public:

    burger_server( const std::string& name );

    rclcpp_action::GoalResponse
    goal_callback(const rclcpp_action::GoalUUID& id,
    BurgerAction::Goal::ConstSharedPtr goal);

    rclcpp_action::CancelResponse
    cancel_callback
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<BurgerAction>>
     goal_handle );

    void
    accept_goal
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<BurgerAction>>
     goal_handle );
    
  };



}

