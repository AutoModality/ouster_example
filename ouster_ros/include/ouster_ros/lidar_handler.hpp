#ifndef LIDAR_HANDLER
#define LIDAR_HANDLER
    auto lidar_handler = [&](const ouster_ros::PacketMsg& pm) mutable {
        static bool local_scan {false};                   
        cb.push_back(std::make_shared<ouster_ros::PacketMsg>(pm));
        LidarStates status;
        std::shared_ptr<ouster_ros::PacketMsg> tmppm;
        do { 
          status = CanProcess( cb, imu_buf, 10000 );

          // ROS_DEBUG_STREAM_THROTTLE(1,"cb(" << cb.size() << ")" << " imu(" << imu_buf.size() << ")");
	  if ( cb.size() >= 1 && imu_buf.size() >= 1 ) 
	    ROS_DEBUG_STREAM_THROTTLE(1,
				      "cb(" << cb.size() << ")" << " imu(" << imu_buf.size() << ")\n" <<
				      "LidarPkts = [ " << GetTimes((*cb.front()))[0] << " , " << GetTimes((*cb.back()))[15] << " ]\n" << 
				      "Imus      = [ " << imu_buf.front().header.stamp.toNSec() << " , " << imu_buf.back().header.stamp.toNSec() << " ]\n");
	  
	  switch (status) {
                case LidarStates::PROCESS:
		  ROS_DEBUG_THROTTLE(1,"Processing");
                  tmppm = cb.front();
                  batch_and_publish(tmppm->buf.data(), it , local_scan);
                  cb.pop_front();
                  break;
                case LidarStates::QUEUE:
                  break;
                case LidarStates::SHITCAN:
                  
                  tmppm = cb.front();
                  local_scan = false;
                  batch_and_publish(tmppm->buf.data(), it , local_scan );
                  cb.pop_front();
                  break;
                default:
                  break;
            }
        } while ( status == LidarStates::PROCESS || status == LidarStates::SHITCAN );
                         };
#endif
