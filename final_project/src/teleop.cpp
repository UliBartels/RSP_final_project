#include <final_project/teleop.hpp>


namespace final_project{

  teleop::teleop( const std::string& name ) :
    Node(name){

      teleop_publisher = create_publisher<geometry_msgs::msg::Twist>( "waffle/cmd_vel", 10 );

  }


}
