#ifndef LIDAR_HANDLER
#define LIDAR_HANDLER
    auto lidar_handler = [&](const ouster_ros::PacketMsg& pm) mutable {
        static bool local_scan {false};                   
        cb.push_back(std::make_shared<ouster_ros::PacketMsg>(pm));
        LidarStates status;
        std::shared_ptr<ouster_ros::PacketMsg> tmppm;
        do { 
          status = CanProcess( cb, imu_buf, 10000 );

          ROS_DEBUG_STREAM_THROTTLE(1,"cb(" << cb.size() << ")" << " imu(" << imu_buf.size() << ")");
            switch (status) {
                case LidarStates::PROCESS:
                  tmppm = cb.front();
                  batch_and_publish(tmppm->buf.data(), it , local_scan);
                  cb.pop_front();
                  break;
                case LidarStates::QUEUE:
                  break;
                case LidarStates::SHITCAN:
                  
                  tmppm = cb.front();
                  local_scan = true;
                  batch_and_publish(tmppm->buf.data(), it , local_scan );
                  cb.pop_front();
                  break;
                default:
                  break;
            }
        } while ( status == LidarStates::PROCESS || status == LidarStates::SHITCAN );
                         };
#endif
