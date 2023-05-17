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


    cmd_publisher = create_publisher<geometry_msgs::msg::Twist>("/waffle/cmd_vel" , 10 );
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
    // get result
    auto result = std::make_shared<WaffleAction::Result>();
    std::cout << "Waffle is moving to target. Please Wait...\n";
    while ( NavToPose_client -> get_result() != 1 ){}
    result->result = NavToPose_client -> get_result();
    // add the teleop operation after it get w1


    /* rclcpp::sleep_for(5000ms); */
    if (result->result == 1){
      teleop_publish("Forward");
      rclcpp::sleep_for(3000ms);
      teleop_publish("Stop");
      teleop_publish("Backward");
      rclcpp::sleep_for(1000ms);
      teleop_publish("Stop");
      goal_handle -> succeed(result);
      feedback->progress = "finish waffle nav2 path";
      goal_handle->publish_feedback(feedback);}
    else {
      goal_handle -> abort(result);
      feedback->progress = "abort waffle nav2 path";
      goal_handle->publish_feedback(feedback);
    }
  }
  void waffle_server::teleop_publish(const std::string& command){
    geometry_msgs::msg::Twist v;
    if (command == "Forward"){
      v.linear.x = 0.2;
      cmd_publisher->publish(v);
    }
    else if (command == "Stop"){
      v.linear.x = 0;
      cmd_publisher->publish(v);
    }
    else if (command == "Backward"){
      v.linear.x = -.2;
      cmd_publisher->publish(v);
    }
  }
}
