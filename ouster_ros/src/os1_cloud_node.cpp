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
#include <atomic>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"
#include <latency_testing/DelayStatistics.h>
#include <latency_testing/Concerns.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <latency_testing/DelayStatistics.h>

using PacketMsg = ouster_ros::PacketMsg;
using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;

void setXYZW(tf2::Quaternion &elem, float x, float y, float z , float w ) {
  elem = tf2::Quaternion{x,y,z,w};
}

void filter_pointcloud(CloudOS1 &incloud, CloudOS1 &out_pc)
{
    assert(out_pc.height == 16 );
    int count = 0;
    for(size_t w = 0 ; w < incloud.width; w ++)
    {
      // for(size_t height = 2; height < 64; height+=4)
      for ( size_t h = 2 ; h < incloud.height; h +=4 ) 
      {
          int from_index = w * 64 + h;
          int to_index   = w * 16 + h / 4;
          if(incloud.points[from_index].x == 0 && incloud.points[from_index].y == 0 &&
             incloud.points[from_index].z == 0)
          {
                out_pc.points[to_index].x = std::numeric_limits<double>::quiet_NaN();
                out_pc.points[to_index].y = std::numeric_limits<double>::quiet_NaN();
                out_pc.points[to_index].z = std::numeric_limits<double>::quiet_NaN();
          }
          else
          {
              out_pc.points[to_index] = incloud.points[from_index];
          }
          count++;
        }
    }
    // return out_pc;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_cloud_node");
    ros::NodeHandle nh("~");
    ros::Publisher channel_pubs[OS1::columns_per_buffer];
    
    // ros::Publisher
    
    auto imu_topic        = nh.param("imu_topic", std::string{"/dji_sdk/imu"});
    auto pcl_channel      = nh.param("pcl_channel", std::string{"/sensor/lidar"});
    auto tf_prefix        = nh.param("tf_prefix", std::string{});

    auto base_tf          = nh.param("base_tf", std::string{"base"});
    auto to_tf            = nh.param("to_tf", std::string{"ouster"});
    auto imu_timeout      = nh.param("imu_timeout", int{5});
    auto publish_raw_pc2  = nh.param("publish_raw_pointcloud", bool{false});
    auto organized        = nh.param("organized", bool{false});
    auto self_test        = nh.param("self_test", bool{false});
    ROS_ERROR_STREAM("ORganized is " << organized );
    auto raw              = nh.param("raw"      , bool{false} ); // Use the raw point cloud
    auto min_distance     = nh.param("min_distance", double{0.5} );
    
    
    std::atomic_uint  num_imus{0};

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
    geometry_msgs::TransformStamped static_transform; // TODO 
    int count = 0;
    ros::Rate rate(10.0);
    bool found_xform = false;
    if ( self_test ) {
      static_transform.transform.rotation.x = static_transform.transform.rotation.y = static_transform.transform.rotation.z = 0.0;
      static_transform.transform.rotation.w = 1.0;
    } else {
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
        ROS_ERROR_STREAM("Could not get static_transform from base to tf" );
        return EXIT_FAILURE;
      }
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
    CloudOS1 scaled_cloud{W,H/4};
    CloudOS1 send_cloud{W*H, 1};
    CloudOS1 raw_cloud{W*H,1};


    tf2::Quaternion tweakq{0,0,0,1};

    std::vector<sensor_msgs::Imu> imu_entries(W*H+1);
    boost::circular_buffer<sensor_msgs::Imu> imu_buf(10);
    std::vector<CloudOS1> channel_pcl(OS1::columns_per_buffer, CloudOS1{W*H,1});
    // for( int i = 0 ; i < OS1::columns_per_buffer; i ++ ) {
    //   channel_pcl[i].clear();
    // }
    ROS_ERROR_STREAM("Columns per buffer:" << OS1::columns_per_buffer );
    
    auto imu_rate_pub = nh.advertise<latency_testing::Concerns>("imu_rate", 100 );

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
                             num_imus ++;
                             // am::MeasureDelayStop(ros::this_node::getName() + "/external_imu_cb" );
                             //am::MeasureDelayStart(ros::this_node::getName() + "/external_imu_cb" );
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
    raw_cloud.clear();
    imu_entries.clear();
    auto it = cloud.begin();
    sensor_msgs::PointCloud2 msg{};
    sensor_msgs::PointCloud2 msg_raw{};

    auto batch_and_publish = OS1::batch_to_iter<CloudOS1::iterator>(xyz_lut,
                                  W,
                                  H,
                                  {},                                  
								  &PointOS1::make,
        [&](uint64_t scan_ts) mutable {
                                    CloudOS1 *thiscloud;
                                    if ( organized ) {
                                        filter_pointcloud( cloud, scaled_cloud );
                                        thiscloud = &scaled_cloud;
                                    } else {
                                        thiscloud = &send_cloud;
                                    }
                                    tf2::Quaternion result;
                                    for ( uint32_t i = 0; i < (*thiscloud).size() ; i ++ ) {
					  if ( !raw ) {
					    if (std::sqrt((*thiscloud)[i].x*(*thiscloud)[i].x + (*thiscloud)[i].y*(*thiscloud)[i].y + (*thiscloud)[i].z*(*thiscloud)[i].z) >= min_distance) {
					      auto imu = average_imus( imu_entries[i],imu_entries[i+1] );
					      // auto imu = imu_entries[i];
					      geometry_msgs::Point outmsg;
					      tf2::Quaternion qimu(imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w);
					      tf2::Matrix3x3 m(qimu);
					      tf2Scalar roll, pitch, yaw;
					      m.getRPY(roll,pitch,yaw);
					      qimu.setRPY( roll,pitch,0 );
					      tf2::Quaternion ntransform = tf2::Quaternion{qimu.x(),qimu.y(),qimu.z(),qimu.w()}*
					       				   tf2::Quaternion{static_transform.transform.rotation.x,
											   static_transform.transform.rotation.y,
											   static_transform.transform.rotation.z,
											   static_transform.transform.rotation.w};

					      tf2::Quaternion ntprime = tf2::Quaternion{-static_transform.transform.rotation.x,
					      						-static_transform.transform.rotation.y,
					      						-static_transform.transform.rotation.z,
					      						static_transform.transform.rotation.w} * 
								        tf2::Quaternion{-qimu.x(),
					      						-qimu.y(),
					      						-qimu.z(),
					      						qimu.w()};
					      result = ntransform * tf2::Quaternion{(*thiscloud)[i].x,(*thiscloud)[i].y,(*thiscloud)[i].z,0}*ntprime;
					    } else {
                                              result.setX(0);
                                              result.setY(0);
                                              result.setZ(0);
					    }
					  } else {
					    result.setX((*thiscloud)[i].x);
					    result.setY((*thiscloud)[i].y);
					    result.setZ((*thiscloud)[i].z);
					  }
					  (*thiscloud)[i].x = result.x();
					  (*thiscloud)[i].y = result.y();
					  (*thiscloud)[i].z = result.z();			  
                                      }

                                    msg = ouster_ros::OS1::cloud_to_cloud_msg(
                                                                              *thiscloud,
                                                                              std::chrono::nanoseconds{scan_ts},
                                                                              lidar_frame
                                                                              );
                                    if (publish_raw_pc2) {
                                        msg_raw = ouster_ros::OS1::cloud_to_cloud_msg(
                                                                                      raw_cloud,
										      std::chrono::nanoseconds{scan_ts},
										      lidar_frame
										      );
                                        msg_raw.header.frame_id = "body_Level_FLU";
                                        // raw_pub.publish(msg_raw);
                                    }

          msg.header.frame_id = "body_Level_FLU";
				    //am::MeasureDelayStop (ros::this_node::getName() + "/ouster_pcl_delay" );
				    //am::MeasureDelayStart(ros::this_node::getName() + "/ouster_pcl_delay" );
          lidar_pub.publish(msg);
          send_cloud.clear();
          imu_entries.clear();
          
        },
        //
        // Callback on Channel pt
        //
        [&](auto pt, int ichannel ) {
          if ( std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) >= min_distance ) {
            // ROS_ERROR_THROTTLE(1, "Called here !");
            if ( send_cloud.size() < W*H ) {
              send_cloud.push_back(pt);
              if ( send_cloud.size() > 1000 ) {
                ROS_INFO_STREAM_THROTTLE(1, "cloud cap:" << send_cloud.points.capacity() << " size:" << send_cloud.size() );
              }
	    } else {
	      ROS_ERROR_STREAM_THROTTLE(1, "Size of send_cloud:" << send_cloud.size() );
	    }
	  }
	  if ( imu_buf.empty() ) {
	    sensor_msgs::Imu a;
	    a.orientation.w = 1;
	    imu_entries.push_back(a);
	  } else {
	    if ( !imu_buf.empty() ) 
	      imu_entries.push_back(imu_buf.back());
	  }
	});

    
    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        batch_and_publish(pm.buf.data(), it);
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
      imu_pub.publish(imu);
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), [&]( const ros::TimerEvent &ev ) {
                                                                      (void)ev;
                                                                      auto val = num_imus.load();
                                                                      if ( val == 0 ) {
									//am::MeasureDelayStart(ros::this_node::getName() + "/external_imu_cb" );
                                                                          ros::Duration(imu_timeout).sleep();
                                                                          //am::MeasureDelayStop(ros::this_node::getName() + "/external_imu_cb" );
                                                                          ROS_ERROR_STREAM("IMU has taken longer than " << imu_timeout << " seconds"  );
                                                                          sensor_msgs::Imu tmsg;
                                                                          tmsg.orientation.x = tmsg.orientation.y = tmsg.orientation.z = 0;
                                                                          tmsg.orientation.w = 1;
                                                                          imu_buf.push_back(tmsg);
                                                                          num_imus ++;
                                                                      }
                                                                  },
      false
      );
    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}

