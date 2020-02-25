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
        LidarStates status;
        std::shared_ptr<ouster_ros::PacketMsg> tmppm;
        ROS_DEBUG_STREAM_THROTTLE(1,"Running");
        do { 
            status = CanProcess( cb, imu_buf, 10000 );
            ROS_DEBUG_STREAM_THROTTLE(1,"cb(" << cb.size() << ")" << " imu(" << imu_buf.size() << ")");
            switch (status) {
                case LidarStates::PROCESS:
                  tmppm = cb.front();
                  batch_and_publish(tmppm->buf.data(), it , false);
                  cb.pop_front();
                  break;
                case LidarStates::QUEUE:
                  break;
                case LidarStates::SHITCAN:
                  batch_and_publish(tmppm->buf.data(), it , true );
                  break;
                default:
                  break;
            }
        } while ( status == LidarStates::PROCESS );
                         };
#endif
