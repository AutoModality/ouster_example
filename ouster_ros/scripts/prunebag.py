#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
import math

import argparse


def prune_bag(infile,outfile,num_msgs=100,start=0):
    #import ipdb; ipdb.set_trace()
    outbag = rosbag.Bag(outfile, 'w')
    tmp = rosbag.Bag(infile).read_messages()

    while num_msgs:
        x = tmp.next()
        if start <= 0:
            topic = x.topic
            msg = x.message
            t = x.timestamp
            outbag.write(topic, msg, t)
            num_msgs -= 1
        else:
            start -= 1

# print("Reached")
# with rosbag.Bag(outbag, 'w') as outbagfile: }
#     for topic, msg, t in rosbag.Bag(inbag).read_messages(): }
#         while n: }
#             if start <= 0 : }
#                 outbagfile.write(topic, msg, t) }
#                 n -= 1 }
#             else: }
#                 start -= 1 }


def parse_args():
    parser = argparse.ArgumentParser(description="A parser for this script")
    parser.add_argument("-i","--inbag",
                        help="Input bag file") 

    parser.add_argument("-o","--outbag",default="out.bag",
                        help="Output bag file") 

    parser.add_argument("-n","--nummessages",default=100,type=int,
                        help="Number of messages") 

    parser.add_argument("-s","--start",default=0,type=int,
                        help="Message to start on") 

    args = parser.parse_args()

    if not args.inbag:
        print("You must specify --inbag INBAG_FILE", file=sys.stderr )
        sys.exit(1)

    return args

   
if __name__ == "__main__":
    args = parse_args()
    prune_bag(args.inbag,args.outbag,args.nummessages,args.start)
   
