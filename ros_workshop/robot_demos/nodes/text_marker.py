#!/usr/bin/env python
import roslib
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
#from tf.transformations import quaternion_from_euler
#from math import pi

def markerPub():
    markerArray = MarkerArray()
    pub = rospy.Publisher('text_marker', MarkerArray)
    #markerArray.ns = 'text_makers'
    p1 = Point(0.502, 11.492, 0.000) #room1 Position(0.718, -0.571, 0.000),Orientation(0.000, 0.000, -0.008, 1.000) = Angle: -0.015
    p2 = Point(-3.66308784485,13.2560949326, 0.000) #room2  Orientation(0.000, 0.000, -0.741, 0.672) = Angle: -1.668 
    p3 = Point(-0.00961661338806, 9.34885692596, 0.000) #room3 Position(0.782, -2.700, 0.000), Orientation(0.000, 0.000, -0.052, 0.999) = Angle: -0.105
    p4 = Point(-4.05501270294,10.2383213043, 0.000) #room4 Position(-0.343, -6.387, 0.000), Orientation(0.000, 0.000, -0.667, 0.745) = Angle: -1.459
    p5 = Point(-3.27898073196, 6.35970401764, 0.000)  #door1 , Position(-1.134, -1.524, 0.000), Orientation(0.000, 0.000, -0.704, 0.710) = Angle: -1.562
    p6 = Point( -1.94060504436, 0.0072660446167, 0.000) #door2 Position(-0.203, -3.766, 0.000), Orientation(0.000, 0.000, -0.717, 0.697) = Angle: -1.599
    p7 = Point(1.34751307964, 0.948370933533, 0.000) #desk Position(-1.579, -3.107, 0.000), Orientation(0.000, 0.000, 0.012, 1.000) = Angle: 0.023
    p8 = Point(-1.64714360237,4.02495145798, 0.000) #cooridor Position(-0.175, -4.631, 0.000), Orientation(0.000, 0.000, -0.717, 0.697) = Angle: -1.599
    p9 = Point(-3.74548912048,3.87098288536,0.000)
    txt1 = "Office"
    txt2 = "Meeting room"
    txt3 = "Warehouse"
    txt4 = "Dock"
    txt5 = "Door1"
    txt6 = "Door2"
    txt7 = "Corridor"
    txt8 = "Computer Desk"
    txt9 = "hall"
    txtPoint1 = text_marker(1, p1, txt1)
    txtPoint2 = text_marker(2, p2, txt2)
    txtPoint3 = text_marker(3, p3, txt3)
    txtPoint4 = text_marker(4, p4, txt4)
    txtPoint5 = text_marker(5, p5, txt5)
    txtPoint6 = text_marker(6, p6, txt6)
    txtPoint7 = text_marker(7, p7, txt7)
    txtPoint8 = text_marker(8, p8, txt8)
    txtPoint9 = text_marker(9, p9, txt9)
    markerArray.markers.append(txtPoint1)
    markerArray.markers.append(txtPoint2)
    markerArray.markers.append(txtPoint3)
    markerArray.markers.append(txtPoint4)
    markerArray.markers.append(txtPoint5)
    markerArray.markers.append(txtPoint6)
    markerArray.markers.append(txtPoint7)
    markerArray.markers.append(txtPoint8)
    markerArray.markers.append(txtPoint9)
    while not rospy.is_shutdown():
        pub.publish(markerArray)

def text_marker(marker_id, _point, txt):
    #waypoints = list()
    #waypoints.append(Pose(Point(2.680, 1.600, 0.000), 0.1))
    #waypoints.append(Pose(Point(2.680, -1.600, 0.000), 0.1))
    #waypoints.append(Pose(Point(-2.680, 1.600, 0.000), 0.1))
    #waypoints.append(Pose(Point(-2.680, -1.600, 0.000), 0.1))
    #set up markeri
    marker = Marker()
    marker_scale = 0.6
    marker_lifetime = 0
    marker_ns = 'point name'
    marker_color= {'r':0.0, 'g':1.0, 'b':0.0, 'a':1.0}
    marker.ns = marker_ns
    marker.id = marker_id
  #  marker.type = Marker.SPHERE
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.header.frame_id = '/map'
    marker.lifetime = rospy.Duration(marker_lifetime)
    marker.scale.x = marker_scale
    marker.scale.y = marker_scale
    marker.scale.z = marker_scale
    marker.color.r = marker_color['r']
    marker.color.g = marker_color['g']
    marker.color.b = marker_color['b']
    marker.color.a = marker_color['a']
    marker.text = txt
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = _point.x
    marker.pose.position.y = _point.y
    marker.pose.position.z = _point.z
    print txt
    return marker
    # for waypoint in waypoints:
   #     p = Point()
   #     p = waypoint.position
   #     marker.points.append(p)
  #  i = 0
  #  while i < 4 and not rospy.is_shutdown():
  #     marker_pub.publish(marker)
  #     i += 1


if __name__ == '__main__':
    try:
        rospy.init_node("text_marker")
        markerPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Down")
