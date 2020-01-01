#!/usr/bin/env python
#coding:utf-8
from __future__ import print_function
import rospy
from geometry_msgs.msg import PointStamped,PoseStamped

def test_publish_segments():
    """set a path consisting of segments"""
    rospy.init_node('test_publish_segments', anonymous=True)
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    point_list=[[0.721069872379,-0.348542779684],
    [0.343987584114,-0.353359788656],
    [0.325208365917,-0.147307574749],
    [0.728177905083,0.198717370629],
    [0.293201953173,0.238933160901],
    [-0.14968355,0.234476700425],
    [-0.252078384161,-0.333438158035],
    [0.131837338209,-0.365527868271],
    [0.279740422964,0.220124557614],
    [-0.245669052005,-0.333652347326],
    [-0.692820727825,-0.273771047592],
    [-0.67185819149,-0.0305300317705],
    [-0.194762974977,0.229563802481],
    [-0.61670601368,0.275763481855],
    [-1.03288269043,0.302511930466],
    [-1.07011294365,-0.235483184457],
    [-0.789817690849,-0.296207368374],
    [-0.738482773304,0.279833465815]]#2020
    point_msg=PointStamped()
    rospy.sleep(3.0)#wait for the publisher registering
    rate = rospy.Rate(5.0)
    point_msg.header.frame_id='map'
    for each_point in point_list:
        point_msg.point.x=each_point[0]
        point_msg.point.y=each_point[1]
        print('[',each_point[0],',',each_point[1],']')
        pub.publish(point_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        test_publish_segments()
    except rospy.ROSInterruptException:
        pass
