# OS-1 Example Code
Sample code for connecting to and configuring the OS-1, reading and visualizing
data, and interfacing with ROS.

See the `README.md` in each subdirectory for details.


| Github Action  | Debian Package |
| ------------- | ------------- |
| [![Story Development](https://github.com/AutoModality/ouster_example/workflows/Story%20Development/badge.svg)](https://github.com/AutoModality/ouster_example/actions?query=workflow%3A%22Story+Development%22) | --  |
| [![Release Candidate](https://github.com/AutoModality/ouster_example/workflows/Release%20Candidate/badge.svg)](https://github.com/AutoModality/ouster_example/actions)  | [![Latest Version @ Cloudsmith](https://api-prd.cloudsmith.io/badges/version/automodality/dev/deb/ros-kinetic-am-ouster/latest/d=ubuntu%252Fxenial;t=1/?render=true&badge_token=gAAAAABeK1Aoz_Jygzl0L-9q4ZfB8NO8MXSiM3CB1AG6Lh7mHtwjjXaP0MrSgvhwxjlkOwax2Te7PHnlZMDXkkfSA52wWfjkRTiuU_ZD5xcaxgImwJTKW_k%3D)](https://cloudsmith.io/~automodality/repos/dev/packages/detail/deb/ros-kinetic-am-ouster/latest/d=ubuntu%252Fxenial;t=1/)  |
| [![Release](https://github.com/AutoModality/ouster_example/workflows/Release/badge.svg)](https://github.com/AutoModality/ouster_example/actions?query=workflow%3A%22Release)  | [![Latest Version @ Cloudsmith](https://api-prd.cloudsmith.io/badges/version/automodality/release/deb/ros-kinetic-am-ouster/latest/d=ubuntu%252Fxenial;t=1/?render=true&badge_token=gAAAAABeRJL7nERCHN5w9gKPTBzZYkIl7hdWzs8lTyF609cxt8udyVg6Xmsl0P0JD6PVHY1JZ566x18LZ81kkDM-czqewiUpbFo_5bWxiub9xpiC0tifgRU%3D)](https://cloudsmith.io/~automodality/repos/release/packages/detail/deb/ros-kinetic-am-ouster/latest/d=ubuntu%252Fxenial;t=1/) |



## Contents
* [ouster_client/](ouster_client/README.md) contains an example C++ client for the OS-1 sensor
* [ouster_viz/](ouster_viz/README.md) contains a visualizer for the OS-1 sensor
* [ouster_ros/](ouster_ros/README.md) contains example ROS nodes for publishing point cloud messages

## Sample Data
* Sample sensor output usable with the provided ROS code is available
  [here](https://data.ouster.io/sample-data-1.12)
