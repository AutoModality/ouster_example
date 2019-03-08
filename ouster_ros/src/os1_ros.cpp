#include <pcl_conversions/pcl_conversions.h>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster_ros/os1_ros.h"

namespace ouster_ros {
namespace OS1 {

using namespace ouster::OS1;

ns timestamp_of_imu_packet(const PacketMsg& pm) {
    return ns(imu_gyro_ts(pm.buf.data()));
}

ns timestamp_of_lidar_packet(const PacketMsg& pm) {
    return ns(col_timestamp(nth_col(0, pm.buf.data())));
}

bool read_imu_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(imu_packet_bytes + 1);
    return read_imu_packet(cli, m.buf.data());
}

bool read_lidar_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(lidar_packet_bytes + 1);
    return read_lidar_packet(cli, m.buf.data());
}

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& p) {
    const double standard_g = 9.80665;
    sensor_msgs::Imu m;
    const uint8_t* buf = p.buf.data();

    m.header.stamp.fromNSec(imu_gyro_ts(buf));
    m.header.frame_id = "os1_imu";

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 0;

    m.linear_acceleration.x = imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = imu_la_z(buf) * standard_g;

    m.angular_velocity.x = imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }
    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }

    return m;
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const CloudOS1& cloud, ns timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp.fromNSec(timestamp.count());
    return msg;
}

static PointOS1 nth_point(int ind, const uint8_t* col_buf) {
    float h_angle_0 = col_h_angle(col_buf);
    auto tte = trig_table[ind];
    const uint8_t* px_buf = nth_px(ind, col_buf);
    float r = px_range(px_buf) / 1000.0;
    float h_angle = tte.beam_azimuth_angles + h_angle_0;

    PointOS1 point;
    point.reflectivity = px_reflectivity(px_buf);
    point.intensity = px_signal_photons(px_buf);
    point.x = -r * tte.cos_beam_altitude_angles * cosf(h_angle);
    point.y = r * tte.cos_beam_altitude_angles * sinf(h_angle);
    point.z = r * tte.sin_beam_altitude_angles;
    point.ring = ind;

    return point;
}


int azimuth_angles_processed =0 ;
int first_angle = -1;
int prev_angle = -1;
  

inline unsigned int rounded_angle(unsigned int colval )
{
  return (colval / 1408)*1408;
}
#define MAX_ANGLE_VALUE  1408*64
  
inline unsigned int angle_distance( unsigned int angle1, unsigned int angle2)
{
  unsigned int distance = abs( angle1 - angle2);
  return ( angle1 < angle2 ? distance  :  MAX_ANGLE_VALUE - distance );
}
  
void add_packet_to_cloud(ns scan_start_ts, ns scan_duration,
                         const PacketMsg& pm, CloudOS1& cloud) {
    const uint8_t* buf = pm.buf.data();

    for (int icol = 0; icol < columns_per_buffer; icol++) {
        const uint8_t* col_buf = nth_col(icol, buf);
        float ts = (col_timestamp(col_buf) - scan_start_ts.count()) /
          (float)scan_duration.count();
        
        //
        // Only add the packets that aren't zero or less than 0.5
        //
        for (int ipx = 0; ipx < pixels_per_column; ipx++) {
            auto p = nth_point(ipx, col_buf);
            p.t = ts;

            if ( !(std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) < 0.5 )) { 
                // std::cout << "(" << p.x << "," << p.y << "," << p.z  << ")\n";
		// ROS_INFO("Point!\n");
		cloud.push_back(p);
            }
        }
    }
    unsigned int cur_angle = rounded_angle( *((unsigned int*)&buf[12]));
    // std::cout << "Angle: " << (uint32_t)col_h_encoder_count( buf ) << "\n";
    if ( first_angle < 0 ) {
        first_angle = cur_angle;
        azimuth_angles_processed ++;
    } else if ( cur_angle  == (unsigned int)first_angle  ) { 
	azimuth_angles_processed = 0;
    } else if( first_angle >= 0 && angle_distance( prev_angle, cur_angle ) > angle_distance( prev_angle, first_angle )) { 
	azimuth_angles_processed = 0;
    } else {
        azimuth_angles_processed ++;
    }
    prev_angle = cur_angle;
    // ROS_INFO("Angle: %d\t%d\n", first_angle, cur_angle);
}

void spin(const client& cli,
          const std::function<void(const PacketMsg& pm)>& lidar_handler,
          const std::function<void(const PacketMsg& pm)>& imu_handler) {
    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(lidar_packet_bytes + 1);
    imu_packet.buf.resize(imu_packet_bytes + 1);

    while (ros::ok()) {
        auto state = poll_client(cli);
        if (state & ERROR) {
            ROS_ERROR("spin: poll_client returned error");
            return;
        }
        if (state & LIDAR_DATA) {
            if (read_lidar_packet(cli, lidar_packet.buf.data()))
                lidar_handler(lidar_packet);
        }
        if (state & IMU_DATA) {
            if (read_imu_packet(cli, imu_packet.buf.data()))
                imu_handler(imu_packet);
        }
        ros::spinOnce();
    }
}

static ns nearest_scan_dur(ns scan_dur, ns ts) {
    return ns((ts.count() / scan_dur.count()) * scan_dur.count());
};

std::function<void(const PacketMsg&)> batch_packets(
    ns scan_dur, const std::function<void(ns, const CloudOS1&)>& f) {
    auto cloud = std::make_shared<OS1::CloudOS1>();
    auto scan_ts = ns(-1L);

    return [=](const PacketMsg& pm) mutable {
        ns packet_ts = OS1::timestamp_of_lidar_packet(pm);
        if (scan_ts.count() == -1L)
            scan_ts = nearest_scan_dur(scan_dur, packet_ts);

        // @TODO
        // Keep track of the number of points in the
        // cloud...
        // Maintain a backup point cloud when we haven't
        // fully completed a scan
        OS1::add_packet_to_cloud(scan_ts, scan_dur, pm, *cloud);

        auto batch_dur = packet_ts - scan_ts;

        if ( azimuth_angles_processed == 0 ) {
            f(scan_ts, *cloud);
            cloud->clear();
            scan_ts = ns(-1L);
        }
    };
}
}
}
