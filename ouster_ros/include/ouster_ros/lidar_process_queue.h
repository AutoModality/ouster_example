#ifndef LIDAR_PROCESS_QUEUE_H
#define LIDAR_PROCESS_QUEUE_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/circular_buffer.hpp>
#include <ouster_ros/PacketMsg.h>
#include <vector>


enum class LidarStates { PROCESS, QUEUE, SHITCAN };

LidarStates CanProcess( boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> &lidar_packets,
                        boost::circular_buffer<sensor_msgs::Imu> &imu_buf ,
                        uint64_t delta_ns
                        );

std::vector<uint64_t> GetTimes( const ouster_ros::PacketMsg  &pkt );

void SetTimes( ouster_ros::PacketMsg &pkt , const std::vector<uint64_t> &times );

#endif
