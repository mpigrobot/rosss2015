/*
 * turtle_tf_listener.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: exbot
 */
#include<ros/ros.h>
//tf包提供了一个实现机制TransformListener来实现接收转换
#include<tf/transform_listener.h>
#include<geometry_msgs/Twist.h>
//#include<turtlesim/Velocity.h>

#include<turtlesim/Spawn.h>
//用于生成乌龟
int main(int argc, char** argv)
{
  ros::init(argc,argv,"my_tf_listener");
  ros::NodeHandle node;
  ros::service::waitForService("spawn");
  ros::ServiceClient  add_turtle=node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  //生成新的乌龟

  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);
  //通过主题turtle2/cmd_vel来发布geometry_msgs::Twist类型的消息
   tf::TransformListener listener;
   ros::Rate rate(10.0);
//在这里创建了一个TransformListener名称为listener对象,一旦listener创建后，它开始就接收tf的转换消息并让它们缓冲10s.
//TransformListener应该要一直存在，
//否则它的缓存将不能填满并且几乎所有的查询将不能进行
   while(node.ok()){

      tf::StampedTransform transform;
  /* wait for time
try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/turtle2", "/turtle1",
                              now, ros::Duration(3.0));
    listener.lookupTransform("/turtle2", "/turtle1",
                             now, transform); 
   }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }*/


     // follow  1
  try{ 
      listener.lookupTransform("/turtle2", "/turtle1",  
          		  ros::Time(0), transform);
				  //time
				 /* ros::Time now = ros::Time::now();
    listener.waitForTransform("/turtle2", "/turtle1",
                              now, ros::Duration(3.0));
    listener.lookupTransform("/turtle2", "/turtle1",
                             now, transform);*/
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
   //查询一个指定的转换，有四个主要参数：
//我们想要转换从turtle2到turtle1,也即最初运行结果所看到的当turtle1移动时，turtle2跟随着turtle1的轨迹移动
//转换的时间
//用transform来存储转换结果

  /*    // add carrot
  try{ 
      listener.lookupTransform("/turtle2", "/carrot1",  
                               ros::Time(0), transform);
							    //查询一个指定的转换，有四个主要参数：
//我们想要转换从turtle2到/carrot1,也即最初运行结果所看到的当/carrot1移动时，
//turtle2跟随着/carrot1的轨迹移动并保持2米距离
//转换的时间
//用transform来存储转换结果

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }*/

     //time travel  
     /* try{
          ros::Time now = ros::Time::now();
          ros::Time past = now-ros::Duration(5.0);
        listener.waitForTransform("/turtle2",now,"/turtle1",past,"/world" ,ros::Duration(1.0));
        listener.lookupTransform("/turtle2",now,"/turtle1",past,"/world",transform);

      }catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
       // ros::Duration(1.0).sleep();
        
      }*/
	  /* lookupTransform()中的六个参数，寻找turtle2的现在的轨迹，寻找turtle1五秒以前的轨迹，
		  world框架不随时间变换而转变，
		  而框架turtle2跟随的是turtle1五秒前的轨迹，即可以实现功能。
		  */


       // tf::StampedTransform transform;
  


      geometry_msgs::Twist vel_msg;
      vel_msg.angular.z=4.0*atan2(transform.getOrigin().y(),transform.getOrigin().x());
      vel_msg.linear.x=0.5*sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));
      turtle_vel.publish(vel_msg);
      rate.sleep();
	  //计算新的线性和角度的向量值为turtle2,这是基于turtle2到turtle1的距离和角度，新的向量值将会发布在主题turtle2/cmd_vel上
//更改turtle2的移动
   }
   return 0;
}


