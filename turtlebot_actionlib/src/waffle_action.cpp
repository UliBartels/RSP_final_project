#include <turtlebot_actionlib/waffle_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  waffle_server::waffle_server( const std::string& name ):
    Node( name ){

  server  = rclcpp_action::create_server<WaffleAction>
      ( this,
	"waffle_action_server",
	std::bind( &waffle_server::goal_callback, this, _1, _2 ),
	std::bind( &waffle_server::cancel_callback, this, _1 ),
	std::bind( &waffle_server::accept_goal, this, _1 ) );


  NavToPose_client = std::make_shared<nav2pose::nav2pose_client>("navigate_to_pose");
  }
  
  rclcpp_action::GoalResponse
  waffle_server::goal_callback(const rclcpp_action::GoalUUID&,
			       WaffleAction::Goal::ConstSharedPtr goal){

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse
  waffle_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaffleAction>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void
  waffle_server::accept_goal
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaffleAction>>
   goal_handle ){

    std::cout << "Waffle server processing" << std::endl;

    auto feedback = std::make_shared<WaffleAction::Feedback>();
    auto goal = goal_handle -> get_goal();
    //call NavToPose
    NavToPose_client -> call_server(goal->target);
    feedback->progress = "feedback:call nav2 server";
    // get result
    auto result = std::make_shared<WaffleAction::Result>();
    std::cout << "nav2pose_client result = " << NavToPose_client -> get_result() << std::endl;
    result->result = NavToPose_client -> get_result();
    if (result->result == 1){
      goal_handle -> succeed(result);
      feedback->progress = "feedback:finish waffle nav2 path";}
    else {
      goal_handle -> abort(result);
      feedback->progress = "feedback:abort waffle nav2 path";
    }

  }
  /* double calculate_progress(const geometry_msgs::msg) */
}

