#include <turtlebot_actionlib/burger_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  robot_server::robot_server( const std::string& name ):
    Node( name ){

    server  = rclcpp_action::create_server<action_msgs::action::burger>
      ( this,
	"burger_action_server",
	std::bind( &robot_server::goal_callback, this, _1, _2 ),
	std::bind( &robot_server::cancel_callback, this, _1 ),
	std::bind( &robot_server::accept_goal, this, _1 ) );
    
  }
  
  rclcpp_action::GoalResponse
  robot_server::goal_callback(const rclcpp_action::GoalUUID&,
			       action_msgs::action::burger::Goal::ConstSharedPtr goal){

    if( goal->action.size() == 1 ){
      std::cout << "accept" << std::endl;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else{
      std::cout << "reject" << std::endl;
      return rclcpp_action::GoalResponse::REJECT;
    }    
    
  }


  rclcpp_action::CancelResponse
  robot_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_msgs::action::burger>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void
  robot_server::accept_goal
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_msgs::action::burger>>
   goal_handle ){

    std::cout << "server processing" << std::endl;
    for( int i=0; i<5; i++ ){
      
      std::this_thread::sleep_for( 1000ms );
      auto feedback = std::make_shared<action_msgs::action::burger::Feedback>();
      feedback->progress  = 0.2 * (i+1) ;
      goal_handle->publish_feedback( feedback );
      
    }

    auto result = std::make_shared<action_msgs::action::burger::Result>();
    result->result = 1;
    goal_handle->succeed(result);
    
  }
  
  
}

