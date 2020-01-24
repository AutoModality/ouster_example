#!/usr/bin/env python
"""ouster test program that calls existing test suite and saves results"""

from __future__ import print_function
import sys
import argparse
import subprocess
import locale
import re
from os.path import expanduser
import os
import ConfigParser
import shutil
import time
import datetime
from os.path import basename
import tempfile
import glob
import os
import datetime

def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('-O', '--output_file',default="ouster_test" + datetime.datetime.now().strftime("_%Y%m%d-%H%M%S") + ".bag",
                        help='Output Bagfile name')
    parser.add_argument('-D', '--output_directory',default="/home/nvidia",
                        help='Output bag file directory')
    parser.add_argument('-d', '--duration',type=int,
                        help='Duration for the test [Default=60 seconds]',default=60 )
    parser.add_argument('-o', '--organized', action="store_true",default=False,
                        help='Organized or Unorganized pointcloud')

    parser.add_argument('-H', '--hz' , type=int, help="frequency to run ouster at")
    parser.add_argument('-E', '--hzerror' , type=float, default=0.5, help="Hertz error")
    parser.add_argument('-r', '--resolution', type=int, help="Resolution to run scan (512,1024,2048)")
    
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)
    args = parser.parse_args()
    if args.hz not in [10,20]:
        print("Error: hz=%d  is not an allowed range %s" % ( args.hz, [10,20]), file=sys.stderr )
    if args.resolution not in [512,1024,2048]:
        print("Error: resolution=%d is not an allowed range %s" % ( args.resolution, [512,1024,2048]), file=sys.stderr )
    if args.resolution == 2048 and args.hz == 20:
        print("Error: Resolution=%d can only run at %d hz" % (args.resolution, 10 ),file=sys.stderr)
        sys.exit(1)

    if args.organized:
        args.organized = "organized"
    else:
        args.organized = "unorganized"
    return args

if __name__ == "__main__":
    args = parse_args()
    cmd = 'rostest ouster_ros hztest_fixture.launch  hz:=%d hzerror:=%f resolution:=%d organized:=%s test_duration:=%d out_file:=%s output_directory:=%s' % ( args.hz,args.hzerror,args.resolution, args.organized,args.duration,args.output_file, args.output_directory )
    print("Running '%s'" % (cmd)) 
    os.system(cmd)
    
