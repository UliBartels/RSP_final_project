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
    
    cmd_publisher = create_publisher<geometry_msgs::msg::Twist>("/burger/cmd_vel" , 10 );
  }
  
  rclcpp_action::GoalResponse
  burger_server::goal_callback(const rclcpp_action::GoalUUID&,
			       BurgerAction::Goal::ConstSharedPtr goal){

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
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

      
    std::this_thread::sleep_for( 1000ms );
    auto feedback = std::make_shared<BurgerAction::Feedback>();
    auto result = std::make_shared<BurgerAction::Result>();
    auto goal = goal_handle -> get_goal();
    
    cmd_publish(goal->command);
    feedback -> progress = "Burger is moving " + goal->command;
    result->result = 1;
    goal_handle -> succeed(result);
  }
     
  void burger_server::cmd_publish(const std::string& command){
    geometry_msgs::msg::Twist v;
    if (command == "Forward"){
      v.linear.x = 0.02;
      cmd_publisher->publish(v);
    }
    else if (command == "Stop"){
      v.linear.x = 0;
      cmd_publisher->publish(v);
    }
    else if (command == "Backward"){
      v.linear.x = -.02;
      cmd_publisher->publish(v);
    }
  } 
}

