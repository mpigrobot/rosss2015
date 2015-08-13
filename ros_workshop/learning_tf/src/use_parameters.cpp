// This program changes turtlesim's background color at start up
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "use_parameters");
  ros::NodeHandle nh;

  std::string back_red, back_green, back_blue;

  // Wait until the clear service is available, which indicates that turtlesim's background color parameters have been set by default.
  ros::service::waitForService("clear");

  // Read default parameters
  ros::param::get("background_r", back_red);
  ros::param::get("background_g", back_green);
  ros::param::get("background_b", back_blue);

  ROS_INFO("the background parameter of RED is %s", back_red.c_str());
  ROS_INFO("the background parameter of GREEN is %s", back_green.c_str());
  ROS_INFO("the background parameter of BLUE is %s", back_blue.c_str());

  // Set the background color for turtlesim with new values
  ros::param::set("background_r", 255);
  ros::param::set("background_g", 255);
  ros::param::set("background_b", 0);

  // Get turtlesim to pick up the new parameter values.
  ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
  std_srvs::Empty srv;
  clearClient.call(srv);

  // Read current new parameters
  ros::param::get("background_r", back_red);
  ros::param::get("background_g", back_green);
  ros::param::get("background_b", back_blue);

  ROS_INFO("the background parameter of RED now is %s", back_red.c_str());
  ROS_INFO("the background parameter of GREEN now is %s", back_green.c_str());
  ROS_INFO("the background parameter of BLUE now is %s", back_blue.c_str());

}
