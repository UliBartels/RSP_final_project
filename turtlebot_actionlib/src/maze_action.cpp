#include <turtlebot_action/maze_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  action_server::action_server( const std::string& name ):
    Node( name ){
    // Create a callback group
     burger_client_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
     waffle_client_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create an action client for the burger
  burger_client = rclcpp_action::create_client<BurgerAction>(this, "burger_action_server", burger_client_group);
  waffle_client = rclcpp_action::create_client<WaffleAction>(this, "waffle_action_server", waffle_client_group);


    server  = rclcpp_action::create_server<MazeAction>
      ( this,
	"maze",
	std::bind( &action_server::goal_callback, this, _1, _2 ),
	std::bind( &action_server::cancel_callback, this, _1 ),
	std::bind( &action_server::execute, this, _1 ) );
    
    
  }  

  rclcpp_action::GoalResponse
  action_server::goal_callback(const rclcpp_action::GoalUUID&,
			       MazeAction::Goal::ConstSharedPtr goal){

    /* if(goal -> start_location.size() == 2 || goal -> end_location.size() == 2){ */
    /*   std::cout << "accept" << std::endl; */
    /*   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; */
    /* } */
    /* else{ */
    /*   std::cout << "reject" << std::endl; */
    /*   return rclcpp_action::GoalResponse::REJECT; */
    /* } */    
    
    
    RCLCPP_INFO(node_->get_logger(), "Received goal with start position (%f, %f) and end position (%f, %f )",
    goal->start_pose.position.x(), goal->start_pose.position.y(),
    goal->end_pose.position.x(), goal->end_pose.position.y());
  }


  rclcpp_action::CancelResponse
  action_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // ----------------------call for both burger and waffle----------------------------------------------//
// define the call for burger
 int  action_server::burger_call(const int& command==1){

   BurgerAction::Goal goal;

    goal.order = command;
     std::cout << "burger order : "<< goal.order<< std::endl;
    rclcpp_action::Client<BurgerAction>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&action_server::burger_response_callback, this, _1);
    options.feedback_callback =
      std::bind(&action_server::burger_feedback_callback, this, _1, _2);
    options.result_callback = 
      std::bind(&action_server::burger_result_callback, this, _1);



    auto goal_handle = burger_client->async_send_goal(goal, options);
    auto burger_result = burger_client-> async_get_result( goal_handle.get());
    if (burger_result.get().code == rclcpp_action::ResultCode::SUCCEEDED){

    return 1;
    
    };
    return 0;
    /* return burger_result.result->result; */
  }
  
  void action_server::burger_response_callback
  ( rclcpp_action::ClientGoalHandle<BurgerAction>::SharedPtr )
  {
    std::cout << "burger client response" << std::endl;
  }
  void action_server::burger_feedback_callback
  ( rclcpp_action::ClientGoalHandle<BurgerAction>::SharedPtr ,
    const std::shared_ptr<const BurgerAction::Feedback> feedback )
  {
    std::cout << "burger feedback" << std::endl
	      << (double)feedback->progress*100<< "%" << std::endl;
  }
  void action_server::burger_result_callback
  ( const rclcpp_action::ClientGoalHandle<BurgerAction>::WrappedResult&
    result )
  {
    std::cout << "burger result" << std::endl
	      << (int)result.result->result << std::endl;
  }

// ---------------------------------------burger call-----------------------------//

  // define the call for waffle
 int action_server::waffle_call(const geometry_msgs::msg::Pose& AirTagPose){

     WaffleAction::Goal goal;
     goal.target  = AirTagPose;


    rclcpp_action::Client<WaffleAction>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&action_server::waffle_response_callback, this, _1);
    options.feedback_callback =
      std::bind(&action_server::waffle_feedback_callback, this, _1, _2);
    options.result_callback = 
      std::bind(&action_server::waffle_result_callback, this, _1);
    
    auto goal_handle = waffle_client->async_send_goal(goal, options);
    auto waffle_result  = waffle_client-> async_get_result( goal_handle.get());
    if (waffle_result.get().code == rclcpp_action::ResultCode::SUCCEEDED){

    return 1;
    
    };
    return 0;
    
  }
  
  void action_server::waffle_response_callback
  ( rclcpp_action::ClientGoalHandle<WaffleAction>::SharedPtr )
  {
    std::cout << "waffle client response" << std::endl;
  }
  void action_server::waffle_feedback_callback
  ( rclcpp_action::ClientGoalHandle<WaffleAction>::SharedPtr ,
    const std::shared_ptr<const WaffleAction::Feedback> feedback )
  {
    std::cout << "waffle feedback" << std::endl
	      <<(double) feedback->progress*100<<"%"  << std::endl;
  }
  void action_server::waffle_result_callback
  ( const rclcpp_action::ClientGoalHandle<WaffleAction>::WrappedResult&
    result )
  {
    std::cout << "waffle result" << std::endl
	      << (int)result.result->result << std::endl;
  }

// ----------------------------waffle call end----------------------------------------//



   /* **TODO** */ 
    //main action work flow 
void  action_server::execute
  (const std::hared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>>
   goal_handle, const int& command){

    std::cout << "maze server processing" << std::endl;
    // send a goal to the burger to move to start_location position
    auto feedback = std::make_shared<MazeAction::Feedback>();
    feedback->message = "place burger at start_location postion";
    goal_handle->publish_feedback( feedback );

    auto goal = goal_handle -> get_goal();
    /* TODO */
    // publish the start postion of the burger 
    this -> pub

  /* auto burger_result = callback(goal -> start_location); */
    std::cout << "burger result output" << burger_result << std::endl;
    std::cout << "burger is ready to go" << std::endl;


    /* TODO */
    // switch case of the waffle 
    auto burger_result = this -> burger_execute(command); 

  }

int action_server::burger_execute(const int& command){
  
  switch(command){
    case 1:
      std::cout << "go straight" << std::endl;
      break;
    case 2:
      std::cout << "Turn Right " << std::endl;
      break;
    case 3:
      std::cout << "Turn Left" << std::endl;
      break;
    case 4:
      std::cout << "Stop" << std::endl;
      break;
    case 5:
      std::cout << "Call Waffle" << std::endl;
      break;
    default:
      std::cout << "Invalid command" << std::endl;
    }
  return 1;
  }

void action_server::publish_pose(const std::string& name, const geometry_msgs::msg::Pose& pose ){
     maze_publisher = this->create_publisher<geometry_msgs::msg::Pose>(name,10);
     maze_publisher->publish(pose);
  }
}
/*   action_client::action_client( const std::string& name ) : */
/*     Node(name){ */
/*     client = */
/*       rclcpp_action::create_client<action_msgs::action::MazeAction>( this, "start_location_and_end_location" ); */
/*     client->wait_for_action_server(); */
/*   } */
/*   void action_client::call(const std::array<float, 2>& start_location, const std::array<float,2>& end_location){ */
/*     action_msgs::action::MazeAction::Goal goal; */
/*     goal.start_location = pick; */
/*     goal.end_location = end_location; */

/*     rclcpp_action::Client<action_msgs::action::MazeAction>::SendGoalOptions options; */
/*     options.goal_response_callback = */
/*       std::bind(&action_client::response_callback, this, _1); */
/*     options.feedback_callback = */
/*       std::bind(&action_client::feedback_callback, this, _1, _2); */
/*     options.result_callback = */ 
/*       std::bind(&action_client::result_callback, this, _1); */

/*     client->async_send_goal( goal, options ); */
    
/*   } */
  
/*   void action_client::response_callback */
/*   ( rclcpp_action::ClientGoalHandle<action_msgs::action::MazeAction>::SharedPtr ) */
/*   { */
/*     std::cout << "pnp_client response" << std::endl; */
/*   } */
/*   void action_client::feedback_callback */
/*   ( rclcpp_action::ClientGoalHandle<action_msgs::action::MazeAction>::SharedPtr , */
/*     const std::shared_ptr<const action_msgs::action::MazeAction::Feedback> feedback ) */
/*   { */
/*     std::cout << "pnp_client feedback : " << std::endl */
/* 	      << feedback->message << std::endl; */
/*   } */
/*   void action_client::result_callback */
/*   ( const rclcpp_action::ClientGoalHandle<action_msgs::action::MazeAction>::WrappedResult& */
/*     result ) */
/*   { */
/*     std::cout << "result" << std::endl */
/* 	      << (int)result.result->result << std::endl; */
/*   } */
/* } */
