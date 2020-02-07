#include <ros/ros.h>
#include <vb_util_lib/imu_class.h>
#include <vb_util_lib/vb_main.h>
#include <memory>


ros::Publisher pc_pub_;
std::shared_ptr<am::ImuClass> imu_transformer_;


void pc2CB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	sensor_msgs::PointCloud2 res;
	int result = imu_transformer_->transform(*msg, res, msg->header.stamp);
	//ROS_INFO("imu transform result: %d", result);
	res.header = msg->header;
	pc_pub_.publish(res);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_imu_transformation_test");
	
	ros::NodeHandle nh;
	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sensor/lidar/pointcloud2/imu_levelled",100);
	ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/lidar/pointcloud2",1, &pc2CB);	
	
	imu_transformer_ = std::make_shared<am::ImuClass>(nh);
	
	ros::spin();
	return 0;
}
