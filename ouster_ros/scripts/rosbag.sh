#!/bin/bash

sleep $1
shift
rosbag play -l  $@
