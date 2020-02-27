#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "gmock/gmock-matchers.h"

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
#include <ouster_ros/lidar_process_queue.h>

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
      geometry_msgs::TransformStamped static_transform;
      sensor_msgs::PointCloud2 msg{};
      
      ImuLidar() : imu_buf(1000) , cb(1000)  {}
      
      virtual void SetUp(uint32_t W,uint32_t H) { 
        ros::Time::init();
        xyz_lut = ouster::OS1::make_xyz_lut(W, H, beam_azimuth_angles,beam_altitude_angles);
          imu_buf.clear();
          cb.clear();
          results.clear();
          static_transform.transform.rotation.x = static_transform.transform.rotation.y = static_transform.transform.rotation.z = 0.0;
          static_transform.transform.rotation.w = 1.0;

          
      }

      virtual void TearDown() {
          cb.clear();
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
                               }, imu_buf, static_transform);

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
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));

    auto times = GetTimes( *cb.front() );

    for ( size_t i = 0; i < times.size() ; i ++) {
      ASSERT_EQ( times[i] , ros::Time(start.sec,start.nsec).toNSec() ) << "Value for index " << i << "\n";
    }
    
    auto retval  = CanProcess( cb, imu_buf, 100000 );

    ASSERT_EQ( retval, LidarStates::PROCESS );
    
    for ( size_t i = 0; i < times.size() ; i ++) {
      times[i] = i;
    }
    SetTimes( *cb.front(), times );

    auto ntimes = GetTimes( *cb.front() );

    for ( size_t i = 0; i < ntimes.size(); i ++ ) {
      ASSERT_EQ( ntimes[i], times[i] );
    }
    
    SetTimes( *cb.front(), std::vector<uint64_t>({1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}) );
    ntimes = GetTimes( *cb.front() );

    for ( size_t i = 0; i < ntimes.size(); i ++ ) {
      ASSERT_EQ( ntimes[i], 1 );
    }

    // Clear out
    imu_buf.clear();
    cb.clear();

    for ( count = 0; count < 64 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec, 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
    auto tmpval = ros::Time(start.sec, 10000*32).toNSec();

    SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval});    
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));
    retval = CanProcess(cb, imu_buf, 100000 );
    ASSERT_EQ(LidarStates::PROCESS,retval );
    ASSERT_EQ(32,imu_buf.size() );

    

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
    // auto tmp = imu_buf_find(10000,rtime);
    // auto myit = std::find_if(imu_buf.begin(), 
    //                          imu_buf.end(), 
    //                          tmp
    //                          );
    // ASSERT_TRUE( myit != imu_buf.end() );

}

TEST_F(ImuLidar, CheckCanProcess)
{
    sensor_msgs::Imu imumsg;
    ros::Time::init();
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);
    ouster_ros::PacketMsg lidarpkt;
    int count;

    for ( count = 0; count < 5 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
#include "bufdat.h"
    for ( size_t i = 0 ; i < sizeof(buf); i ++ ) {
        lidarpkt.buf.push_back( buf[i] );
    }
    
    uint64_t tmpval = ros::Time(start.sec,start.nsec + 3*10000).toNSec();
    SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval});
    
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));


    auto retval  = CanProcess( cb, imu_buf, 100000 );
    ASSERT_EQ( retval, LidarStates::PROCESS );

    cb.pop_front();
    tmpval = 1000;
    SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval});
    retval = CanProcess( cb, imu_buf, 100000 );
    // Should be QUEUE since we have no values in the queue
    ASSERT_EQ( retval, LidarStates::QUEUE );

    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));
    retval = CanProcess( cb, imu_buf, 100000 );
    ASSERT_EQ( retval, LidarStates::SHITCAN );
    
    
    
}

TEST_F(ImuLidar, CheckCanSCAN)
{
    sensor_msgs::Imu imumsg;
    ros::Time::init();
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);
    ouster_ros::PacketMsg lidarpkt;
    int count;

    for ( count = 2; count < 7 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
    ASSERT_EQ( imu_buf.size() , 5 );
    
#include "bufdat.h"
    for ( size_t i = 0 ; i < sizeof(buf); i ++ ) {
        lidarpkt.buf.push_back( buf[i] );
    }
    
    uint64_t tmpval = ros::Time(start.sec,start.nsec + 0).toNSec();
    SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,
                                              tmpval+4,tmpval+4,tmpval+4,tmpval+4,
                                              tmpval+8,tmpval+8,tmpval+8,tmpval+8,
                                              tmpval+12,tmpval+12,tmpval+12,tmpval+12});


    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));

    auto retval  = CanProcess( cb, imu_buf, 1000 );
    ASSERT_EQ(retval, LidarStates::SHITCAN);

}

TEST_F(ImuLidar, CheckCanSetIndex)
{
    sensor_msgs::Imu imumsg;
    ros::Time::init();
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);
    ouster_ros::PacketMsg lidarpkt;
    int count;

    for ( count = 0; count < 7 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
    ASSERT_EQ( imu_buf.size() , 7 );
    
#include "bufdat.h"
    for ( size_t i = 0 ; i < sizeof(buf); i ++ ) {
        lidarpkt.buf.push_back( buf[i] );
    }
    
    uint64_t tmpval = ros::Time(start.sec,start.nsec + 0).toNSec();
    SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,
                                              tmpval+4,tmpval+4,tmpval+4,tmpval+4,
                                              tmpval+8,tmpval+8,tmpval+8,tmpval+8,
                                              tmpval+12,tmpval+12,tmpval+12,tmpval+12});

    
    SetMeasurementIndicies( lidarpkt, std::vector<uint16_t>{20} );
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));

    auto retval  = CanProcess( cb, imu_buf, 1000 );
    ASSERT_EQ(retval, LidarStates::PROCESS);

    auto a = GetMeasurementIndicies(*(cb.back()));
    ASSERT_EQ( 20,a[0] );
    
    std::vector<uint16_t> expected{21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};
    SetMeasurementIndicies( lidarpkt, expected );
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));
    a = GetMeasurementIndicies(*(cb.back()));
    for ( int i = 0; i < expected.size() ; i ++ ) {
      ASSERT_EQ( expected[i], a[i] );
    }

    SetFrameIndicies( lidarpkt, expected );
    cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));
    auto b = GetFrameIndicies(*(cb.back()));
    ASSERT_EQ( expected[0], b[0] );

    ASSERT_EQ( cb.size(), 3 );
    
}



// 64 IMU packets 
// Make 64 Lidar packets ( 2 scans at 512 ) 
// We shoudl reject the packets from 
// the first packet because the time readings won't be
// correct values
TEST_F(ImuLidar, RemoveScanPackets )
{
    sensor_msgs::Imu imumsg;
    ros::Time::init();
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),0);
    ouster_ros::PacketMsg lidarpkt;
    int count;
    uint16_t W = 512,H = 16;
    ouster_ros::OS1::CloudOS1 cloud{W,H};
    geometry_msgs::Quaternion q;
    std::vector<sensor_msgs::PointCloud2> results;
    auto it = cloud.begin();
    SetUp(W,H);

    auto batch_and_publish = ouster::OS1::am_batch_to_iter<ouster_ros::OS1::CloudOS1::iterator>(
                               xyz_lut, W, H, {}, &ouster_ros::OS1::PointOS1::make,
                               [&](uint64_t scan_ts) mutable {
                                   msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud, std::chrono::nanoseconds{scan_ts}, "lidar_frame");
                                   results.push_back(msg);
                               },imu_buf, static_transform);

    
    for ( count = 1; count <= 64 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imumsg.orientation.x = -1;
        imu_buf.push_back(imumsg);
    }
    cb.clear();

#undef IMU_BUF_FIND
#undef LIDAR_HANDLER
#include <ouster_ros/lidar_handler.hpp>

#include "bufdat.h"

    // Start time of the LidarPacket is After the first IMU has been recorded
    uint64_t tmpval = ros::Time(start.sec,start.nsec + 0*10000).toNSec();    
    std::vector<uint32_t> distances;
    for( int i = 0; i < 64*512; i ++ ) {
      distances.push_back(1000);
    }
    for ( uint16_t pktnum = 0; pktnum < 2; pktnum ++ ) { // Two scans
        for ( int count = 0; count < 32; count ++ ) { // 32 packets 
            for ( size_t i = 0 ; i < sizeof(buf); i ++ ) {
                lidarpkt.buf.push_back( buf[i] );
            }
            std::vector<uint16_t> ids{};
            SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval,tmpval});
            SetFrameIndicies( lidarpkt, std::vector<uint16_t>{pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum,pktnum});
            ids.clear();
            for ( int i = 0; i < 16 ; i ++ ) {
                ids.push_back( i + count * 16 );
            }
            SetMeasurementIndicies( lidarpkt, ids );
            SetDistances( lidarpkt, distances );
            ids.clear();

            lidar_handler( lidarpkt );
        }
        tmpval = ros::Time(start.sec,start.nsec + 30*10000).toNSec();    
    }
    ASSERT_EQ(0,cb.size());
    ASSERT_EQ(1,results.size());
    
}



// Make sure that position doesn't change if the IMU buf is long enough 
TEST_F(ImuLidar,CbufPosition)
{
    sensor_msgs::Imu imumsg;
    ros::Time::init();
    ros::Time reftime = ros::Time::now();
    ros::Time start(reftime.toSec(),10000);
    int count;
    imu_buf.clear();
    for ( count = 0; count < 600 ; count ++ ) {
        imumsg.header.seq = count;
        ros::Time ntime(start.sec,start.nsec + 10000*count);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
    auto pos = std::find_if( imu_buf.begin(),imu_buf.end(),[&](const sensor_msgs::Imu &imumsg ) {
                                                               return imumsg.header.stamp >= ros::Time(start.sec,start.nsec + 10000*500 );
                                                           });
    ASSERT_NE(pos,imu_buf.end());
    ASSERT_NE(pos,imu_buf.begin());
    int num_remove = std::distance(imu_buf.begin(),pos );
    ASSERT_EQ(500,num_remove);
    for ( int i = 0; i < num_remove; i ++ ) {
        imu_buf.pop_front();
        imumsg.header.stamp = ros::Time(start.sec, start.nsec + 10000*(++count));
        imu_buf.push_back( imumsg );
    };
    ASSERT_EQ(600,imu_buf.size());
    ASSERT_EQ(ros::Time(start.sec, start.nsec+ 10000*500 ),imu_buf.front().header.stamp );

}


void GenImus( boost::circular_buffer<sensor_msgs::Imu> &imu_buf, ros::Time start, ros::Time end, uint32_t num_imus )
{
    uint64_t stime = start.toNSec(), etime = end.toNSec();
    uint64_t range = ( etime - stime );
    int count = 0;
    uint64_t incr  =  range / (num_imus-1);
    sensor_msgs::Imu imumsg;
    for ( uint64_t cnt = stime; cnt < etime; cnt += incr , count ++) {
        imumsg.header.seq = count;
        ros::Time ntime;
        ntime.fromNSec(cnt);
        imumsg.header.stamp = ntime;
        imu_buf.push_back(imumsg);
    }
}

#define PACKET_BUFFER_SIZE 12609

void GenPackets(boost::circular_buffer<std::shared_ptr<ouster_ros::PacketMsg>> &cb,  
                ros::Time start,
                ros::Time end,
                uint32_t num_frames,
                uint16_t start_frame,
                uint16_t start_mid
                )
{ 
    uint64_t stime = start.toNSec(), etime = end.toNSec();
    uint64_t range = ( etime - stime );
    int count = 0;
    uint64_t incr  =  range / (num_frames-1);
    sensor_msgs::Imu imumsg;
    ouster_ros::PacketMsg lidarpkt;
    for ( size_t i = 0 ; i < PACKET_BUFFER_SIZE ; i ++ ) {
        lidarpkt.buf.push_back( 0 );
    }
    
    for ( uint64_t cnt = stime; cnt <= etime; cnt += incr , count ++) {
        for ( int i = 0; i < 32 ; i ++ ) {
            uint64_t tmpval = cnt;
            int refval;
            SetTimes( lidarpkt, std::vector<uint64_t>{tmpval,tmpval+incr/16,tmpval+incr/8,tmpval+3*incr/16,
                                                        tmpval +incr/4   ,tmpval+5*incr/16   , tmpval+3*incr/8  ,tmpval+7*incr/16,
                                                        tmpval +incr/2   ,tmpval+9*incr/16   , tmpval+5*incr/8  ,tmpval+11*incr/16,
                                                        tmpval +3*incr/4 ,tmpval+13*incr/16  , tmpval+7*incr/8 , tmpval+15*incr/16});

            cb.push_back(std::make_shared<ouster_ros::PacketMsg>(lidarpkt));
        }
    }
    ROS_DEBUG_STREAM("Added " << cb.size() << " packets");
}



ouster_ros::PacketMsg GenPacket(const std::vector<ros::Time> &timestamps, 
                                const std::vector<uint16_t> &mids, 
                                const std::vector<uint16_t> &fids, 
                                const std::vector<uint32_t> &distances)
{
    ouster_ros::PacketMsg lidarpkt;
    for ( size_t i = 0 ; i < PACKET_BUFFER_SIZE; i ++ ) {
        lidarpkt.buf.push_back( 0 );
    }

    SetTimes( lidarpkt, timestamps );
    SetFrameIndicies( lidarpkt, fids);
    SetMeasurementIndicies( lidarpkt, mids );
    SetDistances( lidarpkt, distances );

    return lidarpkt;
}


// Scenario: We have many old packets that are in cb
// new imus filled up our system but we aren't purging
// the newest elements
//
TEST_F(ImuLidar,ScanOldPackets)
{
    ros::Time::init();
    imu_buf.clear();
    cb.clear();
    ros::Time reftime = ros::Time(1582820000,0 );
    ros::Time start(reftime.sec,reftime.nsec);
    ros::Time end(reftime.toSec(),10000000); // 10 ms

    int count;
    uint16_t W = 512,H = 16;
    ouster_ros::OS1::CloudOS1 cloud{W,H};
    geometry_msgs::Quaternion q;
    std::vector<sensor_msgs::PointCloud2> results;
    auto it = cloud.begin();
    SetUp(W,H);

    auto batch_and_publish = ouster::OS1::am_batch_to_iter<ouster_ros::OS1::CloudOS1::iterator>(
                               xyz_lut, W, H, {}, &ouster_ros::OS1::PointOS1::make,
                               [&](uint64_t scan_ts) mutable {
                                   msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud, std::chrono::nanoseconds{scan_ts}, "lidar_frame");
                                   results.push_back(msg);
                               },imu_buf, static_transform);

    cb.clear();
    imu_buf.clear();
    ROS_DEBUG_STREAM( "Cb=" << cb.size() << " imu_buf=" << imu_buf.size() );
#undef LIDAR_HANDLER
#include <ouster_ros/lidar_handler.hpp>

    //////////////////
    /// START TEST 
    /////////////////
    GenImus( imu_buf, start, end,600);
    ROS_DEBUG_STREAM("Imus      = [ " << imu_buf.front().header.stamp.toNSec() << " , " << imu_buf.back().header.stamp.toNSec() << " ]" );
    auto pktstart = ros::Time(start.sec-2,start.nsec);
    GenPackets( cb, pktstart ,start, 8 , 3000, 42 ); // 8 frames before IMU's were processed
    ROS_DEBUG_STREAM("LidarPkts = [ " << GetTimes((*cb.front()))[0] << " , " << GetTimes((*cb.back()))[15] << " ]" );
    auto lidarpkt = GenPacket(std::vector<ros::Time> {ros::Time(start.sec,start.nsec+1000)},std::vector<uint16_t>{10},std::vector<uint16_t> {42}, std::vector<uint32_t>{1000,1000} );
    ROS_DEBUG_STREAM( "Cb=" << cb.size() << " imu_buf=" << imu_buf.size() );
    ROS_DEBUG_STREAM("LidarPkt  = [ " << GetTimes(lidarpkt)[0] << " , " << GetTimes(lidarpkt)[15] << " ] ");
    auto retval  = CanProcess( cb, imu_buf, 1000 );
    ROS_DEBUG_STREAM("\n");
    ROS_DEBUG_STREAM( "Cb=" << cb.size() << " imu_buf=" << imu_buf.size() );
    ROS_DEBUG_STREAM("Imus      = [ " << imu_buf.front().header.stamp.toNSec() << " , " << imu_buf.back().header.stamp.toNSec() << " ]" );
    ROS_DEBUG_STREAM("LidarPkts = [ " << GetTimes((*cb.front()))[0] << " , " << GetTimes((*cb.back()))[15] << " ]" );
    

    lidar_handler( lidarpkt );    
    ASSERT_GE( imu_buf.size(), 0 );
    // ASSERT_EQ( 0, cb.size() );


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


    // auto tmp = *(cb.front());
    // ASSERT_EQ( pktstart.toNSec(),GetTimes(tmp)[0] );
    // GenPackets( cb,  start,end, 2 ,3008, 42 );   // 2 frames After IMUS

    // auto aa = imu_buf.back();
    // auto bb = imu_buf.front();

    // ASSERT_EQ(600,imu_buf.size());
    // ASSERT_EQ(10*32,cb.size());

    // auto retval  = CanProcess( cb, imu_buf, 1000 );
    // ASSERT_EQ( LidarStates::SHITCAN, retval );

    // ASSERT_EQ( 10 * 32, cb.size() );
    // // Create the new packet
    // uint16_t mid = 30;
    // uint16_t fid = 333;
    // 
    // // lidar_handler(lidarpkt);
    // //Some tests on the genpacket
    // auto ts = GetTimes( lidarpkt );
    // ASSERT_EQ( ros::Time(start.sec,start.nsec+1000000).toNSec(), ts[0] );
    // ASSERT_EQ( ros::Time(start.sec,start.nsec+1000000).toNSec(), ts.back() );
    // ASSERT_EQ( mid, GetMeasurementIndicies(lidarpkt)[0] );
    // ASSERT_EQ( fid, GetFrameIndicies(lidarpkt)[0] );
    // ASSERT_EQ( 1000,GetDistances(lidarpkt)[0] );


    // ASSERT_EQ( 0, results.size() );
    // Currently have 320 packets  .
    // We add one more that is going to be valid
    // Should have 2 Full packets and then we start the next packet.


    // ASSERT_EQ( 1, results.size() );
