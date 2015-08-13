/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/common/common.h>
#include <boost/thread/mutex.hpp>
#include<boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
//#include <out_corner.hpp>
#include<math.h>
#include <vector>
#include <pcl/common/centroid.h>
#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower1/FollowerConfig.h"

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>


namespace turtlebot_follower1
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud1;

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
int counter =0;
int turn_counter = 0;
bool status;
int right_in;
int digree_alpha;
int digree_alpha_second;
int start_time;
int end_time;
float curren_disrection;
float angular_right;
float angular_spped_z;
int left;
int right;
std::deque<unsigned> point_counts_ahead;
geometry_msgs::TwistPtr command(new geometry_msgs::Twist());
boost::mutex locomotion_message_mutex;
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollower() : min_y_(0.1), max_y_(0.2),
                        min_x_(-0.1), max_x_(0.1),
                        max_z_(0.5), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
  {

  }

  ~TurtlebotFollower()
  {
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower1::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */

  void clearLocomotionMessage(){
    command->linear.x=0;
    command->angular.z=0;
  }
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);
     counter =0;
    turn_counter = 0;
    status=true;
    right_in=0;
    digree_alpha=0;
    digree_alpha_second=0;
    start_time=0;
    end_time=0;
    curren_disrection=0.0;
    angular_right=0.0;
    angular_spped_z=0.0;
    left=0;
    right=0;
    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    sub_= nh.subscribe<PointCloud1>("/camera/depth/points", 1, &TurtlebotFollower::cloudcb, this);

    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower1::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower1::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
    clearLocomotionMessage();
    ros::Rate rate(10);
       while (ros::ok())
       {
           locomotion_message_mutex.lock();
           {
            // command->linear.x=0;
            // command->angular.z=0.2;
              //   cmdpub_.publish(command);
           //  locomotion_message->linear.x=1.0;
            // locomotion_message->angular.z=0.4;
             //cmdpub_.publish(command);
                           // ROS_INFO("linear: %f\n",command->linear.x);
                         //   ROS_INFO("angular: %f\n",command->angular.z);
             float angular_raw=command->angular.z;
             if((abs(angular_raw)*10)>3)
             {  int k=(abs(angular_raw)*10)/3;
                float yu = (abs(angular_raw))-k*0.3;
               for(int j=0;j<k;j++){
               for(int i=0;i<15;i++){
                 command->angular.z=copysign(0.30,angular_raw);
               cmdpub_.publish(command);
           //    ROS_INFO("end_linear: %f\n",command->linear.x);
            //   ROS_INFO("end_angular: %f\n",command->angular.z);
               rate.sleep();
               }
             }
               command->angular.z=copysign(0.1,angular_raw);
               cmdpub_.publish(command);
            //   ROS_INFO("end_linear: %f\n",command->linear.x);
             //  ROS_INFO("end_angular: %f\n",command->angular.z);
               angular_raw=0;
             }
             else{
               //command->angular.z=angular_spped_z;
               cmdpub_.publish(command);
              // ROS_INFO("end_linear: %f\n",command->linear.x);
              //  ROS_INFO("end_angular: %f\n",angular_raw);
                rate.sleep();
             }

           }
           locomotion_message_mutex.unlock();

           ros::spinOnce();
           rate.sleep();
       }

  }

  void reconfigure(turtlebot_follower1::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for point clouds. Uses PCL to find the centroid
   * of the points in a box in the center of the point cloud.
   * Publishes cmd_vel messages with the goal from the cloud.
   * @param cloud The point cloud message.
   */
  float get_angle(pcl::PointXYZ& pt){
       float x=pt.x;
       float z = pt.z;
       return -atan2(x,z);
  }
 float get_distance(float z){
   // return sqrt(x*x+y*y+z*z);
    return z;

  }

 void reducePointCloudDensity(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_to_filter, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, double voxel_size)
 {
     // Create the appropriate voxel grid filter
     pcl::VoxelGrid<pcl::PointXYZ> voxel_size_filter;
     voxel_size_filter.setInputCloud(cloud_to_filter);
     voxel_size_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

     // Subsample the input point cloud
     voxel_size_filter.filter(*filtered_cloud);
 }
 void cropPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cropped_cloud,
                     double x_limit_left,
                     double x_limit_right,
                     double y_limit_above,
                     double y_limit_below,
                     double z_limit_ahead,
                     double z_limit_behind)
{
// Create the appropriate pass-through filter
pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
pass_through_filter.setInputCloud(in_cloud);

// Crop horizontally
pass_through_filter.setFilterFieldName("x");
pass_through_filter.setFilterLimits(x_limit_left, x_limit_right);
pass_through_filter.filter(*cropped_cloud);

// Crop vertically
pass_through_filter.setFilterFieldName("y");
pass_through_filter.setFilterLimits(y_limit_above, y_limit_below);
pass_through_filter.filter(*cropped_cloud);

// Crop depth-wise
pass_through_filter.setFilterFieldName("z");
pass_through_filter.setFilterLimits(z_limit_behind, z_limit_ahead);
pass_through_filter.filter(*cropped_cloud);
}
 void levelPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& levelled_cloud)
 {
     // Get the last known sensor's tilt angle
     float angle_rad=0;
    /* sensor_angle_mutex.lock();
     {
         angle_rad = M_PI * sensor_angle / 180.0f;
     }
     sensor_angle_mutex.unlock();*/
     angle_rad = M_PI *  0.0f / 180.0f;
     float sensor_distance_from_ground;
     sensor_distance_from_ground=0.17f;
     // Create the appropriate rotation matrix
     Eigen::Matrix4f rotation_matrix;
     rotation_matrix <<
         1.0f,     0.0f,           0.0f,            0.0f,
         0.0f,     cos(angle_rad), -sin(angle_rad), -sensor_distance_from_ground,
         0.0f,     sin(angle_rad), cos(angle_rad),  0.0f,
         0.0f,     0.0f,           0.0f,            1.0f;

     // Rotate back the point cloud according to input from Kinect's accelerometer
     pcl::transformPointCloud(*in_cloud, *levelled_cloud, rotation_matrix);
 }
  void cloudcb(const PointCloud1::ConstPtr&  cloud)
  {
    float x = 0.0;
        float y = 0.0;
        float z = 1e6;
        float max_z=-1;
        float focus_field_width=0.45;
        float focus_field_heigh=1.4;
        float focus_field_depth=0.6;  //ROI depth ,障碍物距离检测最大0.6。
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;
        unsigned int n = 0;
        Eigen::Vector4f obj_centroid,max_pt_sph;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // Reduce the point cloud density
                reducePointCloudDensity(cloud, cloud_filtered, 0.05);

                // Level it to be parallel to the ground
                levelPointCloud(cloud_filtered, cloud_filtered);

                // Crop it to the area in front of the robot
                cropPointCloud(cloud_filtered,
                               cloud_filtered,
                              -focus_field_width / 2,
                              focus_field_width / 2,
                              -focus_field_heigh,
                               0.0,
                               focus_field_depth,
                               0.0);
        //std::vector<pcl::PointXYZ> uesful_points;
       pcl::PointCloud<pcl::PointXYZ>::Ptr uesful_points (new pcl::PointCloud<pcl::PointXYZ>);
    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
        {
          //First, ensure that the point's position is valid. This must be done in a seperate
          //if because we do not want to perform comparison on a nan value.
          if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
          {
            //Test to ensure the point is within the aceptable box.
            if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_&&pt.z<max_z_)
            {
              //Add the point to the totals
              uesful_points->push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
             // x += pt.x;
            //  y += pt.y;
              z = std::min(z, pt.z);
             // max_z=std::max(max_z,pt.z);

              n++;
            }
            max_z=std::max(max_z,pt.z);
          }

        }
           float centroid_x = 0.0f;
            for (unsigned i = 0; i < cloud_filtered->size(); i++)
            {
                centroid_x += cloud_filtered->points[i].x;
            }
            centroid_x /= cloud_filtered->size();

       // ROS_INFO("max_z: %f\n",max_z);
             pcl::getMinMax3D(*uesful_points,min_pt,max_pt);
           // pcl::compute3DCentroid(*uesful_points,obj_centroid);
           // pcl::getMaxDistance(*uesful_points,obj_centroid,max_pt_sph);
           // ROS_INFO("min_pt of x %f",max_pt_sph[0]);
             ROS_INFO("num: %d",n);
            if( centroid_x < 0.0)
            {
              ROS_INFO("TURN right!!");
            }
            else
              ROS_INFO("TURN left!!");

            if (curren_disrection != 3)
                {
                    point_counts_ahead.clear();
                }

                // Add the sample of the number of points ahead, capping the sample count
                point_counts_ahead.push_front(cloud_filtered->size());
                while (point_counts_ahead.size() > 1)
                {
                    point_counts_ahead.pop_back();
                }

                // Get the smoothed number of points ahead
                int smoothed_point_count_ahead = 0;
                for (unsigned i = 0; i < point_counts_ahead.size(); i++)
                {
                    smoothed_point_count_ahead += point_counts_ahead[i];
                }
                smoothed_point_count_ahead /= point_counts_ahead.size();

                ROS_INFO("smoothed_point_count_ahead: %d",smoothed_point_count_ahead);
       // if(n>3000)
       if(smoothed_point_count_ahead!=0){
        //  x /= n;
         // y /=n;
          x=centroid_x;
          //z /=n;
          float average_distance = (max_z+z)/2;
           
           {
             int linear_speed=0;
             float  min_distance = get_distance(z);
             float  turn_angle = get_angle(min_pt);
             ROS_INFO("x %f",x);
             if(x>0){
               curren_disrection=1;
                float angular_speed = asin((min_distance/average_distance));
                if(angular_speed>0.01){
                 // curren_disrection=angular_speed;
                  if(right==0){ //说明第一次从右边进入，下次要从左边出来

                        command->linear.x=linear_speed;
                        if(angular_speed<0.25){
                          command->angular.z = angular_speed;
                        }else
                        {
                          command->angular.z=0.25;
                        }
                    //  ROS_INFO("turn left\n");
                      left=1;
                  }
                  else{
                    if(angular_speed<(M_PI/2)){

                command->linear.x=linear_speed;
                command->angular.z = -(M_PI/2-angular_speed);
                ROS_INFO("Out corner !! form right  \n");
                  left=0;
                  right=0;
                    }
                    else{

                      command->linear.x=0;
                      command->angular.z = -0.6;
                     ROS_INFO("Out corner !! form right right  \n");
                     left=0;
                     right=0;
                    }
                  }


                }
                //ROS_INFO("turn left angular_speed : %f",angular_speed);

              //  turn_counter+=1;

             }
             else
             {
               curren_disrection=2;
               float angular_speed = asin((min_distance/average_distance));
               if(angular_speed>0.01){
                 if(left==0){ //说明第一次从左边进入，下次要从右边出去。
                   command->linear.x=linear_speed;
                   if(angular_speed<0.25){
                     command->angular.z = -angular_speed;}
                   else{
                     command->angular.z = -0.25;
                   }
                  right=1;
                //  ROS_INFO("turn right\n");
                 }
                 else{
                   if(angular_speed<(M_PI/2)){

               command->linear.x=linear_speed;
               command->angular.z = (M_PI/2-angular_speed);

               left=0;
               right=0;
               ROS_INFO("Out corner !! form left  \n");
                   }
                   else{

                     command->linear.x=linear_speed;
                     command->angular.z = 0.6;
                  left=0;
                  right=0;
                   ROS_INFO("Out corner !! form left  \n");
                   }
                 }

               }
           
             }
           }

        }
        else
        {
          command->linear.x=0.2;
          command->angular.z =0;
          counter+=1;
          curren_disrection=3;
          if(counter>2)
          {
            turn_counter=0;
            digree_alpha=0;
            digree_alpha_second=0;
            right_in=0;
            left=0;
            right=0;
            counter=0;
          }
        }
        status=false;

       
  }

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
};

PLUGINLIB_DECLARE_CLASS(turtlebot_follower1, TurtlebotFollower, turtlebot_follower1::TurtlebotFollower, nodelet::Nodelet);

}
