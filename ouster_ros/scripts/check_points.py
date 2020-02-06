#!/usr/bin/env python2 
from __future__ import print_function

import rospy
import sys
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
import math

import argparse

def report_error(string,testing=False):
    if testing:
        assert(string)
    else:
        print(string,file=sys.stderr )


class Lidar():
    """
    Class that lets us read pointclouds off of the given topic
    Used for verifying the absolute number of points in a cloud
    """
    def __init__(self, scan_topic="/sensor/ouster/pointcloud2", pub_topic="/results",quadrants=[20,20,20,20],checks={},testing=False,error=30):
        self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)
        self.scan_up  = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)
        self.scan_good  = rospy.Publisher("/good", PointCloud2, queue_size=10)
        self.scan_errors  = rospy.Publisher("/Errors", Int32, queue_size=10)
        self.results = []
        self.errors = []
        self.quadrants= quadrants
        self.checks = checks
        self.error = error

        
    def on_scan(self, scan):
        quadrants = [0,0,0,0]
        errors = 0
        for p in pc2.read_points(scan, field_names = ("x", "y", "z"), skip_nans=False):

            if self.checks.has_key("expected_quadrants"):
                if p[0] > 0 and p[1] > 0 :
                    quadrants[0] += 1
                elif p[0] < 0 and p[1] > 0:
                    quadrants[1] += 1
                elif p[0] < 0 and p[1] < 0:
                    quadrants[2] += 1
                elif p[0] > 0 and p[1] < 0:
                    quadrants[3] += 1
            if self.checks.has_key("point_checks"):
                pass
                
        has_error = False
        if self.checks.has_key("expected_quadrants"):
            for quad in range(0,len(quadrants)):
                if quadrants[quad] > self.quadrants[quad] + self.error:
                    has_error = True
                    break

                elif quadrants[quad] <  self.quadrants[quad] - self.error:
                    has_error = True
                    break

        if has_error:
            # import ipdb; ipdb.set_trace()
            print("Out of range (%s)  vs. (%s)" % (quadrants, self.quadrants ))
            self.scan_up.publish(scan)
        else:
            self.scan_good.publish(scan)
            
        if self.checks.has_key("point_checks"):
            pass
                    
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

    parser.add_argument('--hz',type=int,default=10,help="Frequency ")
    parser.add_argument('--resolution',type=int,default=512,help="Resolution")
    parser.add_argument("--scan_topic", "-S", #publish_topic,
                        default="/sensor/lidar/pointcloud2",
                        help="Topic name to publish PointClouds on")
    parser.add_argument("-w","--width", type=float,default=1.0, help="Width of the box")
    
    parser.add_argument("--point_threshold", type=int,help="Number of points to exceed per quadrant")
    parser.add_argument("--number_channels",type=int,default=16,
                         help="Number of channels in the Lidar")
    parser.add_argument("--expected_quadrants",type=int,default=None,
                         help="Number of points per quadrants")
    parser.add_argument("--error_per_quadrant",type=list,default=None,
                        help="Error on points per quadrant")
    parser.add_argument("--stop_time",type=int, default=60)

    parser.add_argument("-E","--error",type=int, default=30 )
    parser.add_argument("--point_checks",default=False)
    
    args = parser.parse_args()
    if args.resolution not in [512,1024,2048]:
        print("Error: resolution=%d is not an allowed range %s" % ( args.resolution, [512,1024,2048]), file=sys.stderr )
        sys.exit(1)

    if args.hz not in [10,20]:
        print("Error: hz=%d  is not an allowed range %s" % ( args.hz, [10,20]), file=sys.stderr )
        sys.exit(1)
        
    if args.resolution == 2048 and args.hz == 20:
        print("Error: Resolution=%d can only run at %d hz" % (args.resolution, 10 ),file=sys.stderr)
        sys.exit(1)

    if args.expected_quadrants:
        args.expected_quadrants = [args.expected_quadrants for x in range(0,4)]
        
        
    if not args.expected_quadrants:
        val = args.resolution * args.number_channels / 4
        args.expected_quadrants = [val for x in range(0,4)]

    args.checks={"expected_quadrants":1,"point_checks":1}

    return args
    
if __name__ == "__main__":
    args = parse_args()

    rospy.init_node(args.node_name,anonymous=False)
    lidar = Lidar(args.scan_topic, "/results",args.expected_quadrants,args.checks,False,args.error)
    lidar.spin()
   




