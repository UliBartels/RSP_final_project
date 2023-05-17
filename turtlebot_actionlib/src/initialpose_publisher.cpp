#include<turtlebot_actionlib/initialpose_publisher.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{
    publisher::publisher(const std::string& name):
      Node(name){
            Nav2InitPose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose" , 10 );
        }
void publisher::initial_pose_publish(){
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.frame_id = "map";
    p.header.stamp = rclcpp::Time();
    p.pose.pose.position.x = 0.152;
    p.pose.pose.position.y = -0.01;
    p.pose.pose.orientation.z = 0.315;
    p.pose.pose.orientation.w = 0.949;
    p.pose.covariance[0] = 0.05;
    p.pose.covariance[7] = 0.05;
    p.pose.covariance[35] = 0.05;
    Nav2InitPose_publisher -> publish(p);
  }
}

