#include <rclcpp/rclcpp.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>


namespace turtlebot_action{

  class subscriber : public rclcpp::Node{
    private:
      rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;

    public:
      subscriber( const std::string& name );
      void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& markers);
      float z_distance;
  };
}
