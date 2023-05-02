#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <maze_msgs/action/bridge_connect.hpp>
#include <maze_msgs/action/burger.hpp>
#include <maze_msgs/action/waffle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace turtlebot_action{

  class action_server : public rclcpp::Node {

  private:
    using MazeAction = maze_msgs::action::BridgeConnect;
    using BurgerAction = maze_msgs::action::Burger;
    using WaffleAction = maze_msgs::action::Waffle;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<BurgerAction>::SharedPtr burger_client;
    rclcpp_action::Client<WaffleAction>::SharedPtr waffle_client;
    rclcpp_action::Server<MazeAction>::SharedPtr server;
    rclcpp::CallbackGroup::SharedPtr burger_client_group;
    rclcpp::CallbackGroup::SharedPtr waffle_client_group;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr maze_publisher;

  public:

    action_server( const std::string& name );

    rclcpp_action::GoalResponse
    goal_callback(const rclcpp_action::GoalUUID& id,
    MazeAction::Goal::ConstSharedPtr goal);

    rclcpp_action::CancelResponse
    cancel_callback
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>>
     goal_handle );

    // ---------------------------------------------------------------//
    int  burger_call(const geometry_msgs::msg::PoseStamped target_location);
    
    void burger_response_callback
    ( rclcpp_action::ClientGoalHandle<BurgerAction>::SharedPtr handle );
    void burger_feedback_callback
    ( rclcpp_action::ClientGoalHandle<BurgerAction>::SharedPtr handle,
      const std::shared_ptr<const BurgerAction::Feedback> feedback );
    void burger_result_callback
    ( const rclcpp_action::ClientGoalHandle<BurgerAction>::WrappedResult&
      result );
    //-------------------------------------------------------------//
    int  waffle_call(const geometry_msgs::msg::PoseStamped& AirTagPose);
    
    void waffle_response_callback
    ( rclcpp_action::ClientGoalHandle<WaffleAction>::SharedPtr handle );
    void waffle_feedback_callback
    ( rclcpp_action::ClientGoalHandle<WaffleAction>::SharedPtr handle,
      const std::shared_ptr<const WaffleAction::Feedback> feedback );
    void waffle_result_callback
    ( const rclcpp_action::ClientGoalHandle<WaffleAction>::WrappedResult&
      result );

    //----------------------------------------------------------------//
    void
    execute
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<MazeAction>>
     goal_handle);

    /* void burger_execute(const int command); */
    /* void publish_pose(const std::string& name, const geometry_msgs::msg::Pose& pose); */
  };
}

 /* class action_client : public rclcpp::Node { */
 /*  private: */
 /*    rclcpp_action::Client<maze_msgs::action::BridgeConnect>::SharedPtr client; */
 /*  public: */
 /*    action_client( const std::string& name ); */

    
 /*    void call(const std::array<float, 2>& pick, const std::array<float,2>& place); */
 /*    void response_callback */
 /*    ( rclcpp_action::ClientGoalHandle<maze_msgs::action::BridgeConnect>::SharedPtr handle ); */
 /*    void feedback_callback */
 /*    ( rclcpp_action::ClientGoalHandle<maze_msgs::action::BridgeConnect>::SharedPtr handle, */
 /*      const std::shared_ptr<const maze_msgs::action::BridgeConnect::Feedback> feedback ); */
 /*    void result_callback */
 /*    ( const rclcpp_action::ClientGoalHandle<maze_msgs::action::BridgeConnect>::WrappedResult& */
 /*      result ); */
    
 /*  }; */
/* } */
