#include<turtlebot_actionlib/camera_subscriber.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace turtlebot_action{
  subscriber::subscriber(const std::string& name):
      Node(name){
         aruco_subscriber = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",10,std::bind(&subscriber::aruco_callback,this,std::placeholders::_1));
      }

  void subscriber::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& markers){

    /* std::cout << "marker reading: " << markers.poses[0].position.z << std::endl; */
    this->z_distance =  markers.poses[0].position.z;
  };
}

