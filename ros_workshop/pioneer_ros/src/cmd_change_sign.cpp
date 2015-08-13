#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

ros::Publisher pub ;

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
    geometry_msgs::Twist current_cmd_vel = msg ;
    
    
    //std::cout << linear_gains[0] << "  " << angular_gains[0] << std::endl ;
    
    
    current_cmd_vel.angular.z = -current_cmd_vel.angular.z;
    pub.publish(current_cmd_vel) ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_change_sign");

       
    ros::NodeHandle nHandpub ;
    pub = nHandpub.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 10) ;
    
    ros::NodeHandle nHandsub ;  
    ros::Subscriber sub = nHandsub.subscribe("/cmd_vel", 10, &cmd_velCallback) ;
    
    ros::spin() ;
    return 0;
}
