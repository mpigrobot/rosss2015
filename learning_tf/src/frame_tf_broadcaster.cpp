/*
 * frame_tf_broadcaster.cpp
 *
 *  Created on: Aug 8, 2014
 *      Author: exbot
 */

#include<ros/ros.h>
//包提供了一个实现机制TransformBroadcaster
#include<tf/transform_broadcaster.h>

int main(int argc,char** argv){

  ros::init(argc,argv,"my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok())
  {   //这个文件与上一节中的turtle_tf_broadcaster.cpp很相似，
// 定义了一个新的frame名称为carrot1,它的父母是turtle1,它距离父母的距离是在y侧即在左侧2米左右。
    transform.setOrigin(tf::Vector3(0.0,2.0,0.0));
    transform.setRotation( tf::Quaternion(0,0,0,1) );
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"turtle1","carrot1"));
	   //通过TransformBroadcaster来发布信息需要四个参数:
//1.首先需要transform
//2.需要给transform一个时间戳，这个时间戳是现在的时间
//3.将我们创建的父框架的名字传输过去，在这个例子中是turtle1
//4.将我们创建的子框架的名字传输过去，在这里就是carrot1本身

    rate.sleep();
    
  }
  return 0;
};


