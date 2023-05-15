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
  }

  rclcpp_action::GoalResponse
  action_server::goal_callback(const rclcpp_action::GoalUUID&,
			       MazeAction::Goal::ConstSharedPtr goal){

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    
    RCLCPP_INFO(node_->get_logger(), "Received goal with start position (%f, %f) and end position (%f, %f )",
    goal->w1.pose.position.x, goal->w1.pose.position.y,
    goal->w2.pose.position.x, goal->w2.pose.position.y);
  }

  rclcpp_action::CancelResponse
  action_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // ----------------------call for both burger and waffle----------------------------------------------//
// define the call for burger
 int  action_server::burger_call(const std::string& Command){


    BurgerAction::Goal goal;
    goal.command = Command;

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
	      << feedback->progress << std::endl;
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
    std::cout << "waffle feedback: " 
	      << feedback->progress << std::endl;
  }
  void action_server::waffle_result_callback
  ( const rclcpp_action::ClientGoalHandle<WaffleAction>::WrappedResult&
    result )
  {
    std::cout << "waffle result: " 
	      << (int)result.result->result << std::endl;
  }

// ----------------------------waffle call end---------------------------------------//


    //main action work flow 
void action_server::execute
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>>
   goal_handle){

    std::cout << "maze server processing" << std::endl;
    // send a goal to the burger to move to start_location position
    auto feedback = std::make_shared<MazeAction::Feedback>();
    feedback->message = "place burger at start_location postion";
    goal_handle->publish_feedback( feedback );

    auto goal = goal_handle -> get_goal();
    auto waffle_result = 0;
    auto burger_result = 0;
    


    // -----------------Test Burger ---------------------------//
// Burger start Moving 
    if (Aruco_subscriber->z_distance != NULL){ 
      burger_result = burger_call("Forward");
      feedback->message = "Burger Moving Forward";
      goal_handle->publish_feedback( feedback );
    }
// two stop location: 0.58 0.23
// two stop location in simulation: 0.0595 0.0516
    while(Aruco_subscriber->z_distance > goal->s1){ 
      std::cout << Aruco_subscriber->z_distance<< std::endl;
      rclcpp::sleep_for(100ms);};
// Butger stop when close to the gap
    burger_result = burger_call("Stop");
    feedback->message = "Stop";
    goal_handle->publish_feedback( feedback );

    // ------------------Test Whaffle -------------------------//
     // call waffle to come  
    feedback->message = "Waffle come to W1";
    goal_handle->publish_feedback(feedback);
    waffle_result = waffle_call(goal -> w1); 
    feedback->message = "Waffle finish bridge";
    /* auto waffle_result =  1; */
    if (waffle_result == 1){
      rclcpp::sleep_for(500ms);
      burger_result = burger_call("Forward");
      feedback->message = "Forward";
      goal_handle->publish_feedback( feedback );
    }

    while(Aruco_subscriber->z_distance > goal->s2){ 
      std::cout << Aruco_subscriber->z_distance<< std::endl;
      rclcpp::sleep_for(100ms);};

    burger_result = burger_call("Stop");
    feedback->message = "Stop";
    goal_handle->publish_feedback( feedback );

    feedback->message = "Waffle come to W2";
    goal_handle->publish_feedback(feedback);
    waffle_result = waffle_call(goal -> w2); 
    feedback->message = "back to origin place";

// Butger stop when close to the gap

    auto maze_result = std::make_shared<MazeAction::Result>();
    maze_result -> result = waffle_result; 
    goal_handle -> succeed(maze_result); 
}

  action_client::action_client( const std::string& name ) :
    Node(name){
    client =
      rclcpp_action::create_client<MazeAction>( this, "maze" );
    client->wait_for_action_server();
  }
  void action_client::call(const float& S1,
      const float& S2,
      const geometry_msgs::msg::PoseStamped& W1,
      const geometry_msgs::msg::PoseStamped& W2){


    //define the value for request info
    MazeAction::Goal goal;
    goal.s1 =  S1;
    goal.s2 =  S2;
    goal.w1 =  W1;
    goal.w2 =  W2;

    rclcpp_action::Client<MazeAction>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&action_client::response_callback, this, _1);
    options.feedback_callback =
      std::bind(&action_client::feedback_callback, this, _1, _2);
    options.result_callback = 
      std::bind(&action_client::result_callback, this, _1);

    client->async_send_goal( goal, options );
    
  }
  
  void action_client::response_callback
  ( rclcpp_action::ClientGoalHandle<MazeAction>::SharedPtr )
  {
    std::cout << "maze_client response" << std::endl;
  }
  void action_client::feedback_callback
  ( rclcpp_action::ClientGoalHandle<MazeAction>::SharedPtr ,
    const std::shared_ptr<const MazeAction::Feedback> feedback )
  {
    std::cout << "maze_client feedback : " << std::endl
	      << feedback->message << std::endl;
  }
  void action_client::result_callback
  ( const rclcpp_action::ClientGoalHandle<MazeAction>::WrappedResult&
    result )
  {
    std::cout << "result" << std::endl
	      << (int)result.result->result << std::endl;
  }
}
