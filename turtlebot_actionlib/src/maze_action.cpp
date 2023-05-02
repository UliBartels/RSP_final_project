#include <turtlebot_actionlib/maze_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{

  action_server::action_server( const std::string& name ):
    Node(name){
    // Create a callback group
     burger_client_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
     waffle_client_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create an action client for the burger and waffle
  burger_client = rclcpp_action::create_client<BurgerAction>(this, "burger_action_server", burger_client_group);
  waffle_client = rclcpp_action::create_client<WaffleAction>(this, "waffle_action_server", waffle_client_group);


  // create an server for Maze
    server  = rclcpp_action::create_server<MazeAction>
      ( this,
	"maze",
	std::bind( &action_server::goal_callback, this, _1, _2 ),
	std::bind( &action_server::cancel_callback, this, _1 ),
	std::bind( &action_server::execute, this, _1) );
    
    geometry_msgs::msg::PoseStamped start_pose, end_pose;
    geometry_msgs::msg::Pose pose;
    start_pose.header.frame_id = "map";
    end_pose.header.frame_id = "map";
    start_pose.header.stamp = now();
    end_pose.header.stamp = now();
    start_pose.pose = pose ;
    end_pose.pose = pose;
  }  

  rclcpp_action::GoalResponse
  action_server::goal_callback(const rclcpp_action::GoalUUID&,
			       MazeAction::Goal::ConstSharedPtr goal){

    /* if(goal -> start_location.size() == 2 || goal -> end_location.size() == 2){ */
    /*   std::cout << "accept" << std::endl; */
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    /* } */
    /* else{ */
    /*   std::cout << "reject" << std::endl; */
    /*   return rclcpp_action::GoalResponse::REJECT; */
    /* } */    
    
    
    RCLCPP_INFO(node_->get_logger(), "Received goal with start position (%f, %f) and end position (%f, %f )",
    goal->start_pose.pose.position.x, goal->start_pose.pose.position.y,
    goal->end_pose.pose.position.x, goal->end_pose.pose.position.y);
  }


  rclcpp_action::CancelResponse
  action_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // ----------------------call for both burger and waffle----------------------------------------------//
// define the call for burger
 int  action_server::burger_call(const geometry_msgs::msg::PoseStamped target_location){


    BurgerAction::Goal goal;
    goal.target = target_location;

     std::cout << "burger target : "
       << goal.target.pose.position.x<<std::setw(5) 
       << goal.target.pose.position.y<<std::setw(5) 
       << goal.target.pose.position.z<<std::setw(5) 
       << std::endl;
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
 int action_server::waffle_call(const geometry_msgs::msg::PoseStamped& AirTagPose){

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

// ----------------------------waffle call end---------------------------------------//


    //main action work flow 
void action_server::execute
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>>
   goal_handle){

    std::cout << "maze server processing" << std::endl;
    /* // send a goal to the burger to move to start_location position */
    /* auto feedback = std::make_shared<MazeAction::Feedback>(); */
    /* feedback->message = "place burger at start_location postion"; */
    /* goal_handle->publish_feedback( feedback ); */

    /* auto goal = goal_handle -> get_goal(); */
    /* // set the parameter for start and end location */
    /* auto burger_result = burger_call(goal->start_pose); */
    /* std::cout << "burger result output" << burger_result << std::endl; */
    /* std::cout << "burger is ready to go" << std::endl; */



    /* // move to waypoint 1 */
    /* feedback->message = "Moving to the P2"; */
    /* goal_handle->publish_feedback( feedback ); */

    /* geometry_msgs::msg::PoseStamped P2,P3,P4,P5; */
    /* burger_result = burger_call(P2); */
    /* std::cout << "burger result output" << burger_result << std::endl; */
    /* std::cout << "burger move to P2 is done" << std::endl; */
   

    /* // call waffle to come */  
    /* feedback->message = "Waffle come to W1"; */
    /* geometry_msgs::msg::PoseStamped W1,W2; */
    /* auto waffle_result = waffle_call(W1); */ 
    /* std::cout << "waffle result output" << burger_result << std::endl; */
    /* std::cout << "waffle move to W1 is done" << std::endl; */


    /* auto maze_result = std::make_shared<MazeAction::Result>(); */
    /* maze_result -> result = 1; */
    /* goal_handle -> succeed(maze_result); */

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
