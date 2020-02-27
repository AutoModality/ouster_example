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
                  
                  tmppm = cb.front();
                  batch_and_publish(tmppm->buf.data(), it , true );
                  cb.pop_front();
                  // Find the first lidarpacket that is usable
                  // auto pos = std::find_if( lidar_buf.begin(), lidar_buf.end(), [&]( const std::shared_ptr<ouster_ros::PacketMsg> &ppkt) {
                  //                                                                std::vector<uint64_t> times = GetTimes(*ppkt);
                  //                                                                return (*start).header.stamp.toNSec() >= times[0];
                  //                                                              });
                  // if ( pos != lidar_buf.end() ) {
                  //   int toremove = std::distance(lidar_buf.begin(),pos);
                  //   for ( int i = 0; i < toremove ; i ++ )
                  //     lidar_buf.pop_front();
                  // } 
                  break;
                default:
                  break;
            }
        } while ( status == LidarStates::PROCESS || status == LidarStates::SHITCAN );
                         };
#endif
