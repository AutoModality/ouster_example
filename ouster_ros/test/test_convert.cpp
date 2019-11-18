#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <ouster_ros/os1_ros.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


using CloudOS1 = ouster_ros::OS1::CloudOS1;
namespace OS1 = ouster::OS1;


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

CloudOS1 *filter_pointcloud(CloudOS1 &incloud, uint32_t width, uint32_t height)
{
    CloudOS1 *out_pc = new CloudOS1{width,height};
    // CloudOS1 *out_pc(new CloudOS1);
    int count = 0;
    for(size_t w = 0 ; w < incloud.width; w++)
    {
      // for(size_t height = 2; height < 64; height+=4)
      for ( size_t h = 2 ; h < incloud.height; h +=4 ) 
      {
          int from_index = w * 64 + h;
          int to_index   = w * 16 + h / 4;
          if(incloud.points[from_index].x == 0 && incloud.points[from_index].y == 0 &&
             incloud.points[from_index].z == 0)
          {
                out_pc->points[to_index].x = std::numeric_limits<double>::quiet_NaN();
                out_pc->points[to_index].y = std::numeric_limits<double>::quiet_NaN();
                out_pc->points[to_index].z = std::numeric_limits<double>::quiet_NaN();
          }
          else
          {
              out_pc->points[to_index] = incloud.points[from_index];
          }
          count++;
        }
    }
    return out_pc;
}


struct PCLFixture : public ::testing::Test
{
  pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr cloud;
  
  PCLFixture() : cloud(new pcl::PointCloud<ouster_ros::OS1::PointOS1>) {
    std::cout <<"Setting up \n";
  }
  
  virtual void SetUp(std::string file_name)
  {                
    if (pcl::io::loadPCDFile<ouster_ros::OS1::PointOS1>(file_name, *cloud) == -1) {
      ROS_ERROR_STREAM("Couldn't read file " << file_name << "\n");
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    // for (std::size_t i = 0; i < cloud->points.size (); ++i)
    //     std::cout << "    " << cloud->points[i].x
    //               << " "    << cloud->points[i].y
    //               << " "    << cloud->points[i].z << std::endl;
  }
  
  virtual void TearDown()
  {
    std::cout << "TEARING DOWN";
  }
};


TEST_F(PCLFixture,Test)
{
    SetUp("flightops.pcd");
    CloudOS1 *ncloud = filter_pointcloud( *cloud, cloud->width, cloud->height / 4 );

    // if (pcl::io::loadPCDFile<ouster_ros::OS1::PointOS1>(file_name, *cloud) == -1) {
    //   ROS_ERROR_STREAM("Couldn't read file " << file_name << "\n");
    // }
    // for (std::size_t i = 0; i < ncloud->points.size (); ++i)
    //     std::cout << "    " << ncloud->points[i].x
    //               << " "    << ncloud->points[i].y
    //               << " "    << ncloud->points[i].z << std::endl;
    
    pcl::io::savePCDFileASCII("out.pcd", *ncloud);

    ASSERT_EQ(1,1);

}

TEST_F(PCLFixture,Test2)
{
    SetUp("flightops.pcd");
    CloudOS1 ncloud{512,16};
    filter_pointcloud( *cloud, ncloud );
    // CloudOS1 *ncloud = filter_pointcloud( *cloud, cloud->width, cloud->height / 4 );
    // if (pcl::io::loadPCDFile<ouster_ros::OS1::PointOS1>(file_name, *cloud) == -1) {
    //   ROS_ERROR_STREAM("Couldn't read file " << file_name << "\n");
    // }
    // for (std::size_t i = 0; i < ncloud.points.size (); ++i)
    //     std::cout << "    " << ncloud.points[i].x
    //               << " "    << ncloud.points[i].y
    //               << " "    << ncloud.points[i].z << std::endl;
    
    pcl::io::savePCDFileASCII("out.pcd", ncloud);

    ASSERT_EQ(1,1);

}




int
main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "delaystats" );
  return RUN_ALL_TESTS();
}
