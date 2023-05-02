#include <turtlebot_actionlib/waffle_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  waffle_server::waffle_server( const std::string& name ):
    Node( name ){

    server  = rclcpp_action::create_server<WaffleAction>
      ( this,
	"burger_action_server",
	std::bind( &waffle_server::goal_callback, this, _1, _2 ),
	std::bind( &waffle_server::cancel_callback, this, _1 ),
	std::bind( &waffle_server::accept_goal, this, _1 ) );
    
  }
  
  rclcpp_action::GoalResponse
  waffle_server::goal_callback(const rclcpp_action::GoalUUID&,
			       WaffleAction::Goal::ConstSharedPtr goal){

    if( typeid(goal->target)== typeid(uint)){
      std::cout << "accept" << std::endl;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else{
      std::cout << "reject" << std::endl;
      return rclcpp_action::GoalResponse::REJECT;
    }    
    
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

    std::cout << "server processing" << std::endl;
    for( int i=0; i<5; i++ ){
      
      std::this_thread::sleep_for( 1000ms );
      auto feedback = std::make_shared<WaffleAction::Feedback>();
      feedback->progress  = 0.2 * (i+1) ;
      goal_handle->publish_feedback( feedback );
      
    }

    auto result = std::make_shared<WaffleAction::Result>();
    result->result = 1;
    goal_handle->succeed(result);
    
  }
  
  
}

