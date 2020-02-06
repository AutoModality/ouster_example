#!/bin/bash
nc 192.168.1.55 7501 < ouster-config-txt
sleep 1
nc 192.168.1.55 7501 < ouster-sensor-info
sleep 1
nc 192.168.1.55 7501 < ouster-lidar-mode
