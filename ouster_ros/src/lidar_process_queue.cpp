#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster/os1_am.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/PacketMsg.h>
#include <ouster_ros/os1_ros.h>
#include <ouster_ros/lidar_process_queue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/circular_buffer.hpp>
#include <ouster_ros/PacketMsg.h>
#include <ros/ros.h>
#include <vector>

LidarStates CanProcess( boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> &lidar_buf,
                        boost::circular_buffer<sensor_msgs::Imu> &imu_buf , uint64_t delta_ns )
{
    if ( lidar_buf.empty() ) {
        return LidarStates::QUEUE;
    }

    if ( imu_buf.empty() ) {
        return LidarStates::QUEUE;
    }
      
    std::vector<uint64_t> times = GetTimes( *(lidar_buf.front()) );
    ROS_DEBUG_STREAM_THROTTLE(1,"Size imubuf: " << imu_buf.size() );

    auto start = imu_buf.begin();
    auto end = imu_buf.end() - 1;
    if ( (*end).header.stamp.toNSec() < times[0] ) { // All IMUS are too old
        return LidarStates::QUEUE;
    } else if ( (*start).header.stamp.toNSec() - delta_ns <= times[0] && times.back() <= (*end).header.stamp.toNSec()) {
        return LidarStates::PROCESS;
    } else {
        return LidarStates::SHITCAN;
    }
}

std::vector<uint64_t> GetTimes( const ouster_ros::PacketMsg  &pkt )
{
    std::vector<uint64_t> retval;
    const uint8_t *buf = pkt.buf.data();
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer; i ++ ) {
        const uint8_t *packet_buf = buf;
        const uint8_t* col_buf = ouster::OS1::nth_col(i, packet_buf);
        uint64_t first_ts = ouster::OS1::col_timestamp(col_buf);
        retval.push_back( first_ts );
    }
    return retval;
}


void SetTimes( ouster_ros::PacketMsg &pkt , const std::vector<uint64_t> &times )
{
    uint8_t *buf = pkt.buf.data();
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer && i < times.size(); i ++ ) {
        const uint8_t *packet_buf = buf;
        const uint8_t* col_buf = ouster::OS1::nth_col(i, packet_buf);
        *((uint64_t *)col_buf) = times[i];
    }
}
