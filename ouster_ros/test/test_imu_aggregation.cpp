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

TEST(ImuLidar,CanProcess)
{
    ros::NodeHandle nh("~");    
    boost::circular_buffer<sensor_msgs::Imu> imu_buf(1000);
    boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> cb(3);
    #include "bufdat.h"
    ouster_ros::PacketMsg lidarpkt;
    std::vector<ouster_ros::PacketMsg> results;
    sensor_msgs::Imu imumsg;
    geometry_msgs::Quaternion q;
    uint32_t W = 512, H = 64;
    ouster_ros::OS1::CloudOS1 cloud{W, H};
    sensor_msgs::PointCloud2 msg{};
    // auto imu_frame = "/os1_imu";
    // auto lidar_frame = "/os1_sensor";

    auto it = cloud.begin();
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



    auto xyz_lut = ouster::OS1::make_xyz_lut(W, H, beam_azimuth_angles,beam_altitude_angles);


    auto batch_and_publish = ouster::OS1::am_batch_to_iter<ouster_ros::OS1::CloudOS1::iterator>(
        xyz_lut, W, H, {}, &ouster_ros::OS1::PointOS1::make,
        [&](uint64_t scan_ts) mutable {
            // scan_ts *= 2;
            // results.push_back(msg);
            // lidar_pub.publish(msg);
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
