#!/usr/bin/env python2 
import rospy
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
import math

import argparse



class Lidar():
    """
    Class that lets us read pointclouds off of the given topic

    """
    def __init__(self, scan_topic="/sensor/ouster/pointcloud2", pub_topic="/chatter/",threshold=65536):
        self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)
        self.scan_up  = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)
        self.scan_errors  = rospy.Publisher("/Errors", Int32, queue_size=10)
        
    def on_scan(self, scan):
        rospy.loginfo("Got scan, projecting")
        time.sleep(1)
        errors = 0
        self.scan_up.publish( scan )
        #import ipdb; ipdb.set_trace() # BREAKPOINT
        for p in pc2.read_points(scan, field_names = ("x", "y", "z"), skip_nans=True):
            if math.sqrt(p[0]**2+p[1]**2+p[2]**2) <= 0.5:
                errors += 1
        rospy.loginfo("Printed cloud")
        self.scan_errors.publish( errors )
    def spin(self):
        rospy.spin()
        
def listener(node_name,topic):
    lidar = Lidar()
    rospy.spin()

node_name = ""
    
def parse_args():
    parser = argparse.ArgumentParser(description="A parser for this script")
    parser.add_argument("--node_name", "-N",default="testnode",
                        help="Node name to run this processing node as") 
    parser.add_argument("--pub_topic", "-P", #publish_topic,
                        default="/chatter",
                        help="Topic name to publish PointClouds on")
    parser.add_argument("--scan_topic", "-S", #publish_topic,
                        default="/sensor/ouster/pointcloud2",
                        help="Topic name to publish PointClouds on")
    parser.add_argument("--point_threshold", help="Required Number of points to exceed")
    
    
    args = parser.parse_args()
    # import ipdb; ipdb.set_trace() # BREAKPOINT
    return args
    
if __name__ == "__main__":
    args = parse_args()

    rospy.init_node(args.node_name,anonymous=False)
    lidar = Lidar(args.scan_topic, args.pub_topic )
    lidar.spin()
    
    # listener(args.scan_topic, args.pub_topic )



