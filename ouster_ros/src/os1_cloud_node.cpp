/**
 * @file
 * @brief Example node to publish OS-1 point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <boost/circular_buffer.hpp>
#include <vector>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"
#include <latency_testing/DelayStatistics.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using PacketMsg = ouster_ros::PacketMsg;
using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;

void setXYZW(tf2::Quaternion &elem, float x, float y, float z , float w ) {
  elem = tf2::Quaternion{x,y,z,w};
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_cloud_node");
    ros::NodeHandle nh("~");
    ros::Publisher channel_pubs[OS1::columns_per_buffer];
    std::string pcl_channel;
    auto imu_topic   = nh.param("imu_topic", std::string{"/dji_sdk/imu"});
    nh.param<std::string>("pcl_channel", pcl_channel, "pcl_channel");
    auto tf_prefix   = nh.param("tf_prefix", std::string{});

    auto base_tf   = nh.param("base_tf", std::string{"base"});
    auto to_tf     = nh.param("to_tf", std::string{"ouster"});

    am::DEFAULT_UPDATE_DELAY=1;// Update every second



    auto ouster_orientation = nh.param<std::vector<float>>("ouster_orientation", {0.0,0.0,0.0,1.0});
        

    auto sensor_frame = tf_prefix + "/body_Level_FLU";
    auto imu_frame = tf_prefix + "/os1_imu";
    auto lidar_frame = tf_prefix + "/os1_sensor";

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("/os1_node/os1_config");
    client.waitForExistence(ros::Duration(10));
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped static_transform;
    int count = 0;
    ros::Rate rate(10.0);
    bool found_xform = false;
    while (nh.ok() && count < 100){
        try{
            static_transform = tfBuffer.lookupTransform(base_tf, to_tf,ros::Time(1));
            count = 100;
            found_xform = true;
            ROS_DEBUG_STREAM("Got " << static_transform );

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.1).sleep();
            count ++;
            continue;
        }
        rate.sleep();
    }
    if ( !found_xform ) {
        ROS_ERROR_STREAM("Couldn't get static_transform from base to tf" );
        return EXIT_FAILURE;
    }

    tf2::Quaternion static_rotate(static_transform.transform.rotation.x,
                                  static_transform.transform.rotation.y,
                                  static_transform.transform.rotation.z,
                                  static_transform.transform.rotation.w);

    tf2::Quaternion static_rotate_prime(-static_transform.transform.rotation.x,
                                        -static_transform.transform.rotation.y,
                                        -static_transform.transform.rotation.z,
                                        static_transform.transform.rotation.w);

    
    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

    CloudOS1 cloud{W, H};
    CloudOS1 send_cloud{W*H, 1};
    tf2::Quaternion tweakq{0,0,0,1};

    std::vector<sensor_msgs::Imu> imu_entries(W*H+1);
    boost::circular_buffer<sensor_msgs::Imu> imu_buf(10);
    std::vector<CloudOS1> channel_pcl(OS1::columns_per_buffer, CloudOS1{W*H,1});
    for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
      channel_pcl[i].clear();
    }

    auto average_imus = []( const sensor_msgs::Imu &first, const sensor_msgs::Imu &second )  {
      sensor_msgs::Imu tmp;
      tf2::Quaternion q1{first.orientation.x,first.orientation.y,first.orientation.z,first.orientation.w};
      tf2::Quaternion q2{second.orientation.x,second.orientation.y,second.orientation.z,second.orientation.w};
      tf2::Quaternion q3 = (q1 + q2) / 2;
      
              
      tmp.angular_velocity.x = ( first.angular_velocity.x + second.angular_velocity.x ) /2;
      tmp.angular_velocity.y = ( first.angular_velocity.y + second.angular_velocity.y ) /2;
      tmp.angular_velocity.z = ( first.angular_velocity.z + second.angular_velocity.z ) /2;
      tmp.linear_acceleration.x = ( first.linear_acceleration.x + second.linear_acceleration.x ) /2;
      tmp.linear_acceleration.y = ( first.linear_acceleration.y + second.linear_acceleration.y ) /2;
      tmp.linear_acceleration.z = ( first.linear_acceleration.z + second.linear_acceleration.z ) /2;
      if ( q3.length () < 0.1 ) {
        tmp.orientation.x = tmp.orientation.y = tmp.orientation.z = 0;
        tmp.orientation.w = 1;
      } else {
        tmp.orientation.x  = q3.x() / q3.length();
        tmp.orientation.y  = q3.y() / q3.length();
        tmp.orientation.z  = q3.z() / q3.length();
        tmp.orientation.w  = q3.w() / q3.length();
      }

      return tmp;
    };

    auto external_imu_cb = [&]( const sensor_msgs::Imu::ConstPtr &imu_msg) {
      auto tmp = std::sqrt( imu_msg->orientation.x*imu_msg->orientation.x +
                            imu_msg->orientation.y*imu_msg->orientation.y +
                            imu_msg->orientation.z*imu_msg->orientation.z +
                            imu_msg->orientation.w*imu_msg->orientation.w );
      if ( tmp < 0.6 ) {
        sensor_msgs::Imu tmsg;
        tmsg.orientation.x = tmsg.orientation.y = tmsg.orientation.z = 0;
        tmsg.orientation.w = 1;
        // ROS_WARN_THROTTLE(1,"Default IMUS");
        imu_buf.push_back(tmsg);
      } else {
        ROS_DEBUG_THROTTLE(1,"Getting IMUs");
        imu_buf.push_back(*imu_msg);
      }
    };

    auto external_imu_handler = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, external_imu_cb );
    
    send_cloud.clear();
    imu_entries.clear();
    auto it = cloud.begin();
    //auto insertit = send_cloud.begin();
    sensor_msgs::PointCloud2 msg{};

    auto batch_and_publish = OS1::batch_to_iter<CloudOS1::iterator>(xyz_lut,
                                  W,
                                  H,
                                  {},
                                  &PointOS1::make,
                                  [&](uint64_t scan_ts) mutable {
                                    // Last one IMU
                                      for ( uint32_t i = 0; i < send_cloud.size() ; i ++ ) {
                                          auto imu = average_imus( imu_entries[i],imu_entries[i+1] );
                                          geometry_msgs::Point outmsg;
                                          tf2::Quaternion qimu(imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w);
                                          tf2::Matrix3x3 m(qimu);
                                          tf2Scalar roll, pitch, yaw;
                                          
                                          m.getRPY(roll,pitch,yaw);
                                          qimu.setRPY( roll,pitch,0 );
                                          
                                          geometry_msgs::TransformStamped tfimu{};

                                          tfimu.transform.rotation.x = qimu.x();
                                          tfimu.transform.rotation.y = qimu.y();
                                          tfimu.transform.rotation.z = qimu.z();
                                          tfimu.transform.rotation.w = qimu.w();
                                          geometry_msgs::Point b;
                                          b.x = send_cloud[i].x;
                                          b.y = send_cloud[i].y;
                                          b.z = send_cloud[i].z;
#ifdef TWEAK
                                          geometry_msgs::TransformStamped tweak{};
                                          setXYZW(tweakq,tweakq.x(),tweakq.y(),tweakq.z(),tweakq.w());

                                          tweak.transform.rotation.x = tweakq.x();
                                          tweak.transform.rotation.y = tweakq.y();
                                          tweak.transform.rotation.z = tweakq.z();
                                          tweak.transform.rotation.w = tweakq.w();

                                          tf2::doTransform( b,b, tweak );
#endif

                                          ROS_DEBUG_STREAM_THROTTLE(0.2,ros::this_node::getName() << "\n" << imu );

                                          tf2::doTransform( b,outmsg, static_transform );
                                          tf2::doTransform( outmsg,outmsg, tfimu );
                                         
                                          send_cloud[i].x = static_cast<float>(outmsg.x);
                                          send_cloud[i].y = static_cast<float>(outmsg.y);
                                          send_cloud[i].z = static_cast<float>(outmsg.z);
                                      }

                                    msg = ouster_ros::OS1::cloud_to_cloud_msg(
                                                                              send_cloud,
                                                                              std::chrono::nanoseconds{scan_ts},
                                                                              lidar_frame
                                                                              );

                                    for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
                                        if ( channel_pubs[i].getNumSubscribers() >= 1 ) {
                                            // ROS_INFO_STREAM("Have a subscriber for channel " << i );
                                            auto tmp = ouster_ros::OS1::cloud_to_cloud_msg(
                                                                                           channel_pcl[i],
                                                                                           std::chrono::nanoseconds{scan_ts},
                                                                                           lidar_frame
                                                                                           );
                                            channel_pubs[i].publish(tmp);
                                        }
                                    }
                                    msg.header.frame_id = "body_Level_FLU";
                                    lidar_pub.publish(msg);
                                    send_cloud.clear();
                                    imu_entries.clear();
                                    for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
                                        channel_pcl[i].clear();
                                    }

                                      
                                    am::MeasureDelayStop(ros::this_node::getName() + "/lidar_cb" );
                                  },
                                  //
                                  // Callback on Channel pt
                                  //
                                  [&](auto pt, int ichannel ) {
                                    if ( std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) >= 0.5 ) {
                                      if ( send_cloud.size() < W*H ) {
                                        send_cloud.push_back(pt);
                                        if ( channel_pcl[ichannel].size() < W*H) {
                                          channel_pcl[ichannel].push_back(pt);
                                        }
                                        if ( imu_buf.empty() ) {
                                            sensor_msgs::Imu a;
                                            a.orientation.w = 1;
                                            imu_entries.push_back(a);
                                        } else {
                                          if ( !imu_buf.empty() ) 
                                            imu_entries.push_back(imu_buf.back());
                                        }
                                      } else {
                                        ROS_ERROR("Size is greater than cloud capacity\n");
                                      }
                                    }                                    
                                  }
                             );

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
      am::MeasureDelayStart(ros::this_node::getName() + "/lidar_cb" );
      batch_and_publish(pm.buf.data(), it );
    };

    auto imu_handler = [&](const PacketMsg& p) {
      auto imu = ouster_ros::OS1::packet_to_imu_msg(p, imu_frame);
      if ( !(imu.orientation.x > 0.01 || 
             imu.orientation.y > 0.01 ||
             imu.orientation.z > 0.01 ||
             imu.orientation.w > 0.01) ) {
        imu.orientation.x = imu.orientation.y = imu.orientation.z = 0;
        imu.orientation.w=1;
      }
      // imu_buf.push_back(imu);
      imu_pub.publish(imu);
    };


    
    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    for ( int i = 0 ; i < OS1::columns_per_buffer;  i ++ ) { 
      channel_pubs[i] = nh.advertise<sensor_msgs::PointCloud2 >(pcl_channel + "_" + std::to_string(i)  , 100);
    }
    

    
    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
                                        
