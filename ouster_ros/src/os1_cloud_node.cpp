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
#include <fstream>
#include <sstream>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster/os1_am.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"
//#include <latency_testing/DelayStatistics.h>
//#include <latency_testing/Concerns.h>
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


// OS1::make_xyz_lut               // 
void debug_configuration(std::vector<double> &xyz_lut, 
                         std::vector<double> &beam_azimuth_angles,
                         std::vector<double> &beam_altitude_angles )
{
#ifdef DEBUG_LUT
    std::ofstream myfile;
    myfile.open("/home/jdamon/xyz_lut1.yaml");
    myfile << "---\n";
    myfile.unsetf ( std::ios::floatfield );
    for ( auto it=xyz_lut.begin() ; it < xyz_lut.end(); it++ ) {
      myfile << "- " << std::setprecision(4) << *it << "\n";
    }
    myfile.close();
#endif
    std::ostringstream ss,ss2;
    ss << "BEAM_AZIMUTH_ANGLES = [ ";
    for ( auto it=beam_azimuth_angles.begin() ; it + 1< beam_azimuth_angles.end(); it++ ) {
      ss << *it << ", ";
    }
    auto it2 = beam_azimuth_angles.end() - 1;
    ss << *it2 << " ]";
    ROS_DEBUG(ss.str().c_str());
    ss2.clear();
    ss2 << "BEAM_ALTITUDE_ANGLES = [ ";
    for ( auto it=beam_altitude_angles.begin() ; it + 1< beam_altitude_angles.end(); it++ ) {
      ss2 << *it << ", ";
    }
    it2 = beam_altitude_angles.end() - 1;
    ss2 << *it2 << " ]";
    ROS_DEBUG(ss2.str().c_str());
    
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
    auto raw              = nh.param("raw"      , bool{false} ); // Use the raw point cloud
    auto min_distance     = nh.param("min_distance", double{0.5} );
    
    boost::circular_buffer<std::shared_ptr<PacketMsg>> cb(3);
// const PacketMsg& pm

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

    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

   
    auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

    debug_configuration( xyz_lut, cfg.response.beam_azimuth_angles, cfg.response.beam_altitude_angles );


    CloudOS1 cloud{W, H};
    auto it = cloud.begin();
    sensor_msgs::PointCloud2 msg{};

    boost::circular_buffer<sensor_msgs::Imu> imu_buf(1000);

    auto batch_and_publish = OS1::am_batch_to_iter<CloudOS1::iterator>(
        xyz_lut, W, H, {}, &PointOS1::make,
        [&](uint64_t scan_ts) mutable {
            msg = ouster_ros::OS1::cloud_to_cloud_msg(
                cloud, std::chrono::nanoseconds{scan_ts}, lidar_frame);
            lidar_pub.publish(msg);
        });

    auto lidar_handler = [&](const PacketMsg& pm) mutable {

        cb.push_back(std::make_shared<PacketMsg>(pm));

        std::shared_ptr<PacketMsg> tmppm;

        while ( cb.size() > 0 ) {
            tmppm = cb.front();
            batch_and_publish(tmppm->buf.data(), it);
            cb.pop_front();
        }
            
        };

    auto external_imu_cb = [&]( const sensor_msgs::Imu::ConstPtr &imu_msg) {
                               auto tmp = std::sqrt( imu_msg->orientation.x*imu_msg->orientation.x +
                                                     imu_msg->orientation.y*imu_msg->orientation.y +
                                                     imu_msg->orientation.z*imu_msg->orientation.z +
                                                     imu_msg->orientation.w*imu_msg->orientation.w );
                               if ( tmp < 0.6 ) {
                                   ROS_ERROR_STREAM_THROTTLE(1,"Getting invalid IMU (" << 
                                                             imu_msg->orientation.x << "," << 
                                                             imu_msg->orientation.y << "," << 
                                                             imu_msg->orientation.z << "," << 
                                                             imu_msg->orientation.w << ")" );
                               } else {
                                   ROS_DEBUG_THROTTLE(1,"Getting IMUs");
                                   imu_buf.push_back(*imu_msg);
                               }
                           };

    auto external_imu_handler = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, external_imu_cb );



    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::OS1::packet_to_imu_msg(p, imu_frame));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
