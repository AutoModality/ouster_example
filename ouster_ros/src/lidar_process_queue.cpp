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
        ROS_DEBUG_STREAM_THROTTLE(1,"Removing IMU(" << std::distance(start,end) << ") elements");
        for ( int i = 0; i < std::distance(start,end); i ++)
            imu_buf.pop_front();
        return LidarStates::QUEUE;
    } else if ( (*start).header.stamp.toNSec() <= times[0] && times.back() <= (*end).header.stamp.toNSec()) {
        auto pos = std::find_if( imu_buf.begin(),end, [&](const sensor_msgs::Imu &val ) {
                                                        auto ts = val.header.stamp.toNSec();
                                                        return (ts >= times[0] && abs(ts - times[0]) <= (int64_t)delta_ns);
                                                      });
        ROS_DEBUG_STREAM_THROTTLE(1,"Removing IMU(" << std::distance(start,pos) << ") elements");
        int toremove = std::distance(start,pos);
        for ( int i = 0; i < toremove; i ++ ) {
            imu_buf.pop_front();
        }
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

void SetMeasurementIndicies( ouster_ros::PacketMsg &pkt, const std::vector<uint16_t> &indicies )
{
    uint8_t *buf = pkt.buf.data();
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer && i < indicies.size() ; i ++ ) {
        const uint8_t *packet_buf = buf;
        uint8_t* col_buf = (uint8_t *)ouster::OS1::nth_col(i, packet_buf);
        ouster::OS1::col_set_measurement_id( col_buf, indicies[i] );
    }
}

std::vector<uint16_t> GetMeasurementIndicies( const ouster_ros::PacketMsg &pkt )
{
    const uint8_t *buf = pkt.buf.data();
    std::vector<uint16_t> retval;
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer ; i ++ ) {
        const uint8_t *packet_buf = buf;
        const uint8_t* col_buf = ouster::OS1::nth_col(i, packet_buf);
        retval.push_back(ouster::OS1::col_measurement_id( col_buf ));
    }
    return retval;
}

void SetFrameIndicies( ouster_ros::PacketMsg &pkt, const std::vector<uint16_t> &indicies )
{
    uint8_t *buf = pkt.buf.data();
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer && i < indicies.size() ; i ++ ) {
        const uint8_t *packet_buf = buf;
        uint8_t* col_buf = (uint8_t *)ouster::OS1::nth_col(i, packet_buf);
        ouster::OS1::col_set_frame_id( col_buf, indicies[i] );
    }
}

std::vector<uint16_t> GetFrameIndicies( ouster_ros::PacketMsg &pkt )
{
    uint8_t *buf = pkt.buf.data();
    std::vector<uint16_t> retval;
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer ; i ++ ) {
        const uint8_t *packet_buf = buf;
        const uint8_t* col_buf = ouster::OS1::nth_col(i, packet_buf);
        retval.push_back(ouster::OS1::col_frame_id( col_buf ));
    }

    return retval;
}


void SetDistances( ouster_ros::PacketMsg &pkt, const std::vector<uint32_t> &ranges )
{
    uint8_t *buf = pkt.buf.data();
    uint32_t position = 0;
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer && i < ranges.size() && position < ranges.size() ; i ++ , position ++) {
        const uint8_t *packet_buf = buf;
        uint8_t* col_buf = (uint8_t *)ouster::OS1::nth_col(i, packet_buf);
        for ( uint8_t ipx = 0; ipx < 64 && position < ranges.size(); ipx++, position ++) {
            uint8_t *px_buf = (uint8_t*)ouster::OS1::nth_px(ipx, col_buf);
            ouster::OS1::set_px_range(px_buf, ranges[i] );
        }
    }
}

std::vector<uint32_t> GetDistances( ouster_ros::PacketMsg &pkt )
{
    uint8_t *buf = pkt.buf.data();
    std::vector<uint32_t> retval;
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer; i ++ ) {
        const uint8_t *packet_buf = buf;
        uint8_t* col_buf = (uint8_t *)ouster::OS1::nth_col(i, packet_buf);
        for ( uint8_t ipx = 0; ipx < 64; ipx++) {
            const uint8_t* px_buf = ouster::OS1::nth_px(ipx, col_buf);
            retval.push_back(ouster::OS1::px_range(px_buf));
        }
    }

    return retval;
}

