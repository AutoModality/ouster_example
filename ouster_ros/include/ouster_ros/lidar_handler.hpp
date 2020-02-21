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
        ROS_DEBUG_STREAM("First ts: " << first_ts << " Last ts: " << last_ts );
        while ( cb.size() > 0 && first_ts  ) {
            tmppm = cb.front();
            batch_and_publish(tmppm->buf.data(), it);
            ROS_DEBUG_THROTTLE(1,"LIDAR !");                           
            cb.pop_front();
        }
            
        };
#endif
