#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace final_project{
  class teleop : public rclcpp::Node{
  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_publisher;

  public:
    teleop( const std::string& name );
    publish( const geometry_msgs::msg::Twist& velocity );


  }
}
