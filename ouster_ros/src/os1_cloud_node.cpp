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
#include <tf2/convert.h>
#include <geometry_msgs/Point.h>

using PacketMsg = ouster_ros::PacketMsg;
using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;


int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_cloud_node");
    ros::NodeHandle nh("~");
    ros::Publisher channel_pubs[OS1::columns_per_buffer];
    std::string pcl_channel;
    auto imu_topic   = nh.param("imu_topic", std::string{});
    nh.param<std::string>("pcl_channel", pcl_channel, "pcl_channel");
    auto tf_prefix   = nh.param("tf_prefix", std::string{});
    am::DEFAULT_UPDATE_DELAY=1;// Update every second
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    // try {
    //   transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",ros::Time(0));
    // } catch ( tf2::TransformException &ex ) {
    //   transformStamped.transform.translation.x =
    //     transformStamped.transform.translation.y =
    //     transformStamped.transform.translation.z = 0;
    // }

    auto ouster_orientation = nh.param<std::vector<float>>("ouster_orientation", {0.0,0.0,0.0});
    tf2::Quaternion rotate;
    tf2::Quaternion rotate_prime;
    rotate.setRPY(ouster_orientation[0],ouster_orientation[1],ouster_orientation[2]);
    rotate_prime.setRPY(-ouster_orientation[0],-ouster_orientation[1],-ouster_orientation[2]);

    auto sensor_frame = tf_prefix + "/body_Level_FLU";
    auto imu_frame = tf_prefix + "/os1_imu";
    auto lidar_frame = tf_prefix + "/os1_sensor";

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("/os1_node/os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    // auto imu_subscriber = nh.subscribe<PacketMsg, const PacketMsg&>("lidar_packets", 2048, lidar_handler);
    
    auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

    CloudOS1 cloud{W, H};
    CloudOS1 send_cloud{W*H, 1};
    std::vector<sensor_msgs::Imu> imu_entries(W*H+1);
    boost::circular_buffer<sensor_msgs::Imu> imu_buf(10);
    std::vector<CloudOS1> channel_pcl(OS1::columns_per_buffer, CloudOS1{W*H,1});
    for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
      channel_pcl[i].clear();
    }

    auto average_imus = []( const sensor_msgs::Imu &first, const sensor_msgs::Imu &second )  {
      sensor_msgs::Imu tmp;
      tmp.orientation.x = ( first.orientation.x + second.orientation.x ) /2;
      tmp.orientation.y = ( first.orientation.y + second.orientation.y ) /2;
      tmp.orientation.z = ( first.orientation.z + second.orientation.z ) /2;
      tmp.orientation.w = ( first.orientation.w + second.orientation.w ) /2;
      tmp.angular_velocity.x = ( first.angular_velocity.x + second.angular_velocity.x ) /2;
      tmp.angular_velocity.y = ( first.angular_velocity.y + second.angular_velocity.y ) /2;
      tmp.angular_velocity.z = ( first.angular_velocity.z + second.angular_velocity.z ) /2;
      tmp.linear_acceleration.x = ( first.linear_acceleration.x + second.linear_acceleration.x ) /2;
      tmp.linear_acceleration.y = ( first.linear_acceleration.y + second.linear_acceleration.y ) /2;
      tmp.linear_acceleration.z = ( first.linear_acceleration.z + second.linear_acceleration.z ) /2;
      return tmp;
    };
    
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
                                    // ROS_INFO_STREAM("Size of imu is " << imu_entries.size() );
                                    // Last one IMU
                                    if ( !imu_buf.empty() ) {
                                      imu_entries.push_back(imu_buf.back());
                                    }
                                    msg = ouster_ros::OS1::cloud_to_cloud_msg(
                                                                                send_cloud,
                                                                                std::chrono::nanoseconds{scan_ts},
                                                                                lidar_frame
                                                                                );
                                    for ( uint32_t i = 0; i < send_cloud.size() ; i ++ ) {
                                      auto imu = average_imus( imu_entries[i],imu_entries[i+1] );
                                      // ROS_INFO("Applying transform to the point");
                                      //  send_cloud[i] = send_cloud[i];
                                    }
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
                                    send_cloud.clear();
                                    imu_entries.clear();
                                    for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
                                      channel_pcl[i].clear();
                                    }
                                    msg.header.frame_id = "body_Level_FLU";
                                    lidar_pub.publish(msg);
                                    am::MeasureDelayStop(ros::this_node::getName() + "/lidar_cb" );
                                  },
                                  //
                                  // Callback on Channel pt
                                  //
                                  [&](auto pt, int ichannel ) {
                                    if ( std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) >= 0.5 ) {
                                      tf2::Quaternion q{pt.x,pt.y,pt.z,0};
                                      auto length = q.length();
                                      auto uvec = (rotate * (q.normalize()))*rotate_prime;
                                      pt.x = uvec.x() * length;
                                      pt.y = uvec.y() * length;
                                      pt.z = uvec.z() * length;
                                      if ( send_cloud.size() < W*H ) {
                                        send_cloud.push_back(pt);
                                        if ( channel_pcl[ichannel].size() < W*H) {
                                          channel_pcl[ichannel].push_back(pt);
                                        }
                                        if ( imu_buf.empty() ) {
                                          sensor_msgs::Imu a;
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
      imu_buf.push_back(imu);
      imu_pub.publish(imu);
    };

    auto external_imu_cb = [&]( const sensor_msgs::Imu::ConstPtr &imu_msg) {
      imu_buf.push_back(*imu_msg);
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    for ( int i = 0 ; i < OS1::columns_per_buffer;  i ++ ) { 
      channel_pubs[i] = nh.advertise<sensor_msgs::PointCloud2 >(pcl_channel + "_" + std::to_string(i)  , 100);
    }
    
    auto external_imu_handler = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, external_imu_cb );
    
    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
