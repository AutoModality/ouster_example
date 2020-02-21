#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <ouster_ros/PacketMsg.h>
#include <ouster/os1_packet.h>
#include <ouster/os1_am.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/PacketMsg.h>
#include <ouster_ros/os1_ros.h>

struct ImuLidar : public ::testing::Test
{
    const std::vector<double> beam_altitude_angles = {
                                                      16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
                                                      12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
                                                      8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
                                                      3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
                                                      -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
                                                      -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
                                                      -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
                                                      -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
    };

    const std::vector<double> beam_azimuth_angles = {
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
                                                     3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    };


      
      std::vector<double> xyz_lut;
      boost::circular_buffer<sensor_msgs::Imu> imu_buf;
      boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> cb;
      std::vector<sensor_msgs::PointCloud2> results;

      sensor_msgs::PointCloud2 msg{};
      
      ImuLidar() : imu_buf(1000) , cb(10)  {}
      
      virtual void SetUp(uint32_t W,uint32_t H) { 
          xyz_lut = ouster::OS1::make_xyz_lut(W, H, beam_azimuth_angles,beam_altitude_angles);
          imu_buf.clear();
          results.clear();
      }

      virtual void TearDown() {
      }
};


TEST_F(ImuLidar,CanProcess)
{
    uint32_t W = 512, H = 64;
    SetUp(W,H);
    ros::Time::init();
    // boost::circular_buffer<sensor_msgs::Imu> imu_buf(1000);
    // boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> cb(3);
#include "bufdat.h"
    ouster_ros::PacketMsg lidarpkt;
    ouster_ros::OS1::CloudOS1 cloud{W,H};
    sensor_msgs::Imu imumsg;
    geometry_msgs::Quaternion q;
    auto it = cloud.begin();

    auto batch_and_publish = ouster::OS1::am_batch_to_iter<ouster_ros::OS1::CloudOS1::iterator>(
                               xyz_lut, W, H, {}, &ouster_ros::OS1::PointOS1::make,
                               [&](uint64_t scan_ts) mutable {
                                   msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud, std::chrono::nanoseconds{scan_ts}, "lidar_frame");
                                   results.push_back(msg);
                               });

#include <ouster_ros/lidar_handler.hpp>
#define LIDAR_HANDLER
    q.x = 0;
    q.y = 0;
    q.z = 1;
    q.w = 0;
    imumsg.orientation = q;
    int count = 0;
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);

    for ( count = 0; count < 5 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }

    ASSERT_EQ(sizeof(buf),(size_t)12609);
    for (size_t i = 0; i < ouster::OS1::columns_per_buffer; i ++ ) {
        const uint8_t *packet_buf = buf;
        const uint8_t* col_buf = ouster::OS1::nth_col(i, packet_buf);
        uint64_t first_ts = ouster::OS1::col_timestamp(col_buf);
        ROS_DEBUG_STREAM("Before Time: " << first_ts );
        *((uint64_t *)col_buf) = ros::Time(start.sec,start.nsec).toNSec();
        first_ts = ouster::OS1::col_timestamp(col_buf);
        ROS_DEBUG_STREAM("After Time : " << first_ts );
    }

    for ( size_t i = 0 ; i < sizeof(buf); i ++ ) {
        lidarpkt.buf.push_back( buf[i] );
    }

    lidar_handler( lidarpkt );
    ASSERT_EQ(results.size(), 1);
    // for ( imu_buf.begin(),imu_buf.end() ) {
    // }

}

TEST_F(ImuLidar,ImuSearch)
{
    uint32_t W = 512, H = 64;
    SetUp(W,H);
    ros::Time::init();
    sensor_msgs::Imu imumsg;

#undef IMU_BUF_FIND
#include <ouster_ros/lidar_handler.hpp>

    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);
    int count;
    for ( count = 0; count < 5 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }

    uint64_t rtime = ros::Time(start.sec,start.nsec+10000).toNSec();
    auto tmp = imu_buf_find(10000,rtime);
    auto myit = std::find_if(imu_buf.begin(), 
                             imu_buf.end(), 
                             tmp
                             );
    ASSERT_TRUE( myit != imu_buf.end() );

} 


#include <ros/console.h>
#include <log4cxx/logger.h>

int
main(int argc, char *argv[] )
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "delaystats" );
  return RUN_ALL_TESTS();
}
