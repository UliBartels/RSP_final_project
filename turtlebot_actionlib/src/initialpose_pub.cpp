#include<turtlebot_actionlib/initialpose_publisher.hpp>
int main( int argc , char** argv){

    rclcpp::init(argc,argv);
    turtlebot_action::publisher publisher("pub");

    rclcpp::Rate rate(1);

    while (rclcpp::ok()){

        publisher.initial_pose_publish();
        rate.sleep();

    }

    rclcpp::shutdown();

    return 0;

}
