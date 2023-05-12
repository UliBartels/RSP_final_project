#include <turtlebot_actionlib/burger_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  burger_server::burger_server( const std::string& name ):
    Node( name ){

    server  = rclcpp_action::create_server<BurgerAction>
      ( this,
	"burger_action_server",
	std::bind( &burger_server::goal_callback, this, _1, _2 ),
	std::bind( &burger_server::cancel_callback, this, _1 ),
	std::bind( &burger_server::accept_goal, this, _1 ) );
    
  }
  
  rclcpp_action::GoalResponse
  burger_server::goal_callback(const rclcpp_action::GoalUUID&,
			       BurgerAction::Goal::ConstSharedPtr goal){

    /* if( typeid(goal)== typeid(uint)){ */
    /*   std::cout << "accept" << std::endl; */
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    /* } */
    /* else{ */
    /*   std::cout << "reject" << std::endl; */
      /* return rclcpp_action::GoalResponse::REJECT; */
    /* } */    
    
  }


  rclcpp_action::CancelResponse
  burger_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<BurgerAction>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void
  burger_server::accept_goal
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<BurgerAction>>
   goal_handle ){

    std::cout << "burger server processing" << std::endl;
    for( int i=0; i<5; i++ ){
      
      std::this_thread::sleep_for( 1000ms );
      auto feedback = std::make_shared<BurgerAction::Feedback>();
      feedback->progress  = 0.2 * (i+1) ;
      goal_handle->publish_feedback( feedback );
      
    }

    auto result = std::make_shared<BurgerAction::Result>();
    result->result = 1;
    goal_handle->succeed(result);
    
  }
  
  
}

