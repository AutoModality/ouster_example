#ifndef IMU_BUF_FIND
#define IMU_BUF_FIND
    auto imu_buf_find = [&]( uint64_t error, uint64_t fts ) {
                          return [error,fts]( const sensor_msgs::Imu &imu ) { 
                                   auto tval = abs(imu.header.stamp.toNSec() - fts  ); 
                                   return ( tval < error );
                                 };
                        };
#endif 
#ifndef LIDAR_HANDLER
#define LIDAR_HANDLER
    auto lidar_handler = [&](const ouster_ros::PacketMsg& pm) mutable {

        cb.push_back(std::make_shared<ouster_ros::PacketMsg>(pm));

        std::shared_ptr<ouster_ros::PacketMsg> tmppm;
        const uint8_t *packet_buf = pm.buf.data();
        const uint8_t* col_buf = ouster::OS1::nth_col(0, packet_buf);        
        const uint64_t first_ts = ouster::OS1::col_timestamp(col_buf);
        const uint8_t *last_col_buf = ouster::OS1::nth_col(ouster::OS1::columns_per_buffer-1,packet_buf);
        uint64_t last_ts = ouster::OS1::col_timestamp(last_col_buf);

        ROS_DEBUG_STREAM_THROTTLE(1,"First ts: " << first_ts << " Last ts: " << last_ts );
        auto iit = std::find_if(imu_buf.begin(),
                                imu_buf.end(),
                                [&](const sensor_msgs::Imu &imu) {
                                  return ( abs(imu.header.stamp.toNSec() - first_ts ) < 10000 );
                                });

        while ( cb.size() > 0 ) {// && first_ts && iit != imu_buf.end()  ) {
            tmppm = cb.front();
            batch_and_publish(tmppm->buf.data(), it);
            ROS_DEBUG_THROTTLE(1,"LIDAR !");                           
            cb.pop_front();
        }
            
        };
#endif
