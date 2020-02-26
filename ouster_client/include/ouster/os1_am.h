/**
 * @file
 * @brief Utilities to interpret data returned from the sensor
 */

#pragma once

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>
#include <ros/ros.h>
#include "ouster/os1_packet.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace ouster {
namespace OS1 {

/**
 * Make a function that batches a single scan (revolution) of data to a
 * random-access iterator. The callback f() is invoked with the timestamp of the
 * first column in the scan before adding data from a new scan. Timestamps for
 * each column are ns relative to the scan timestamp. XYZ coordinates in meters
 * are computed using the provided lookup table.
 *
 * The value type is assumed to be constructed from 9 values: x, y, z,
 * (padding), intensity, ts, reflectivity, noise, range (in mm) and
 * default-constructible. It should be compatible with PointOS1 in the
 * ouster_ros package.
 *
 * @param xyz_lut a lookup table generated from make_xyz_lut, above
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param H number of rows in the lidar scan. 64 for the OS1 family of sensors.
 * @param empty value to insert for mossing data
 * @param c function to construct a value from x, y, z (m), i, ts, reflectivity,
 * ring, noise, range (mm). Needed to use with Eigen datatypes.
 * @param f callback invoked when batching a scan is done.
 * @return a function taking a lidar packet buffer and random-access iterator to
 * which data is added for every point in the scan.
 */
template <typename iterator_type, typename F, typename C>
std::function<void(const uint8_t*, iterator_type it ,boost::circular_buffer<sensor_msgs::Imu> &imu_buf, bool, geometry_msgs::TransformStamped &)> am_batch_to_iter(
    const std::vector<double>& xyz_lut, int W, int H,
    const typename iterator_type::value_type& empty, C&& c, F&& f, boost::circular_buffer<sensor_msgs::Imu> &imu_buf, geometry_msgs::TransformStamped &static_transform ) {
    int next_m_id{W};
    int32_t cur_f_id{-1};
    uint64_t scan_ts{0L};
    bool local_s_can{false};

    return [=](const uint8_t* packet_buf, iterator_type it, boost::circular_buffer<sensor_msgs::Imu> &imu_buf,bool s_can ,geometry_msgs::TransformStamped &static_transform ) mutable {
             local_s_can = s_can;
        for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
            const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
            const uint16_t m_id = OS1::col_measurement_id(col_buf);
            const uint16_t f_id = OS1::col_frame_id(col_buf);
            const uint64_t ts = OS1::col_timestamp(col_buf);
            const bool valid = OS1::col_valid(col_buf) == 0xffffffff;


            // drop invalid / out-of-bounds data in case of misconfiguration
            if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

            if (f_id != cur_f_id) {
                // if not initializing with first packet
                if (scan_ts != 0 ) {
                    // zero out remaining missing columns
                    std::fill(it + (H * next_m_id), it + (H * W), empty);
                    if ( !local_s_can ) {
                      f(scan_ts);
                    } else {
                      local_s_can = false;
                    }
                }

                // start new frame
                scan_ts = ts;
                next_m_id = 0;
                cur_f_id = f_id;
            }

            // zero out missing columns if we jumped forward
            if (m_id >= next_m_id) {
                std::fill(it + (H * next_m_id), it + (H * m_id), empty);
                next_m_id = m_id + 1;
            }

            // index of the first point in current packet
            const int idx = H * m_id;
            if ( !local_s_can ) { 
              for (uint8_t ipx = 0; ipx < H; ipx++) {
                const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
                uint32_t r = OS1::px_range(px_buf);
                int ind = 3 * (idx + ipx);
                auto nearest_imu = std::find_if(imu_buf.begin(),imu_buf.end(),
                                                [&](const sensor_msgs::Imu &imu ) {
                                                    return imu.header.stamp.toNSec() >= scan_ts;
                                                }
                                                            );
                if ( nearest_imu == imu_buf.end() ) {
                  local_s_can = true;
                  ROS_ERROR_THROTTLE(0.5,"SERIOUS ERROR: should have found IMU but didn't");
                  // break;
                }
                tf2::Quaternion q1{(*nearest_imu).orientation.x,(*nearest_imu).orientation.y,(*nearest_imu).orientation.z,(*nearest_imu).orientation.w};

                tf2::Quaternion qimu((*nearest_imu).orientation.x,(*nearest_imu).orientation.y,(*nearest_imu).orientation.z,(*nearest_imu).orientation.w);
                tf2::Matrix3x3 m(qimu);
                tf2Scalar roll, pitch, yaw;
                tf2::Quaternion result;
                m.getRPY(roll,pitch,yaw);
                qimu.setRPY( roll,pitch,0 );

                tf2::Quaternion ntransform = tf2::Quaternion{qimu.x(),qimu.y(),qimu.z(),qimu.w()}*                                                           
                                                             tf2::Quaternion{static_transform.transform.rotation.x,                                                          
                                                                             static_transform.transform.rotation.y,                                                          
                                                                             static_transform.transform.rotation.z,                                                          
                                                                             static_transform.transform.rotation.w};
                tf2::Quaternion ntransformp = tf2::Quaternion{-static_transform.transform.rotation.x,                                                          
                                                              -static_transform.transform.rotation.y,                                                          
                                                              -static_transform.transform.rotation.z,                                                          
                                                              static_transform.transform.rotation.w}*tf2::Quaternion{-qimu.x(),-qimu.y(),-qimu.z(),qimu.w()};
                result = ntransform * tf2::Quaternion{r * 0.001f * xyz_lut[ind + 0],r * 0.001f * xyz_lut[ind + 1],r * 0.001f * xyz_lut[ind + 2],0}*ntransformp; 
                
                // x, y, z(m), i, ts, reflectivity, ring, noise, range (mm)
                it[idx + ipx] = c(result.x(),
                                  result.y(),
                                  result.z(),
                                  OS1::px_signal_photons(px_buf), ts - scan_ts,
                                  OS1::px_reflectivity(px_buf), ipx,
                                  OS1::px_noise_photons(px_buf), r);
                // it[idx + ipx] = c(r * 0.001f * xyz_lut[ind + 0],
                //                   r * 0.001f * xyz_lut[ind + 1],
                //                   r * 0.001f * xyz_lut[ind + 2],
                //                   OS1::px_signal_photons(px_buf), ts - scan_ts,
                //                   OS1::px_reflectivity(px_buf), ipx,
                //                   OS1::px_noise_photons(px_buf), r);
              }
            }
        }
    };
}
}
}
