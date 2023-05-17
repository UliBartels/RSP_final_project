#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace turtlebot_action{

  class publisher : public rclcpp::Node{
    private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Nav2InitPose_publisher;
    public:
      publisher( const std::string& name );
      void initial_pose_publish();
  };
}
