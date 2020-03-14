cmake_minimum_required(VERSION 3.1.0)
project(ouster_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
add_definitions(-DEXTRA_POINT_CLOUDS)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  roscpp
  ouster_client
  tf2
  tf2_ros
  tf2_geometry_msgs
  latency_testing
  rostest
)

add_message_files(FILES PacketMsg.msg)
add_service_files(FILES OS1ConfigSrv.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ouster_ros
  CATKIN_DEPENDS
    roscpp message_runtime pcl_ros
    std_msgs sensor_msgs geometry_msgs
    ouster_client
)

include_directories(${catkin_INCLUDE_DIRS} ../ouster_client/include )

add_library(ouster_ros STATIC src/os1_ros.cpp)
target_link_libraries(ouster_ros ${catkin_LIBRARIES})
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

add_executable(os1_node src/os1_node.cpp)
target_link_libraries(os1_node ouster_ros ${catkin_LIBRARIES})
add_dependencies(os1_node ${PROJECT_NAME}_gencpp)

add_executable(os1_cloud_node src/os1_cloud_node.cpp)
target_link_libraries(os1_cloud_node ouster_ros ${catkin_LIBRARIES})
add_dependencies(os1_cloud_node ${PROJECT_NAME}_gencpp)

# add_executable(viz_node src/viz_node.cpp)
# target_link_libraries(viz_node ${catkin_LIBRARIES})
# add_dependencies(viz_node ${PROJECT_NAME}_gencpp)

add_executable(img_node src/img_node.cpp)
target_link_libraries(img_node ouster_ros ${catkin_LIBRARIES})
add_dependencies(img_node ${PROJECT_NAME}_gencpp)

install(TARGETS os1_node
                os1_cloud_node
                img_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES os1.launch os1_bottom.launch  os1_bottom_pitch_-90degrees.launch os1_tmux.launch  os1_top_pitch30degress_rotate90.launch  os1_top_pitch_90degrees.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS )

#message("ARGCH is ${CMAKE_SYSTEM_PROCESSOR}")

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING )
  # Simple unit tests with catkin_add_gtest
  if ( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" ) 
    catkin_add_gtest(testpcl ./test/test_convert.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test )
    # target_compile_definitions(testpcl PUBLIC TEST)
    target_link_libraries(testpcl ouster_ros )
    install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif ( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64") 

  if ( CATKIN_RUNNING_TESTS )
    
  add_rostest(launch/hztest.launch  ARGS hz:=20 resolution:=512 organized:=organized test_duration:=10 replaydelay:=10 hzerror:=3.0)
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=512 organized:=organized test_duration:=10 replaydelay:=10 hzerror:=3.0)
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=1024 organized:=organized test_duration:=10 replaydelay:=10 hzerror:=3.0)
  add_rostest(launch/hztest.launch  ARGS hz:=20 resolution:=1024 organized:=organized test_duration:=10 replaydelay:=10 hzerror:=3.0)
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=2048 organized:=organized test_duration:=10 replaydelay:=10 hzerror:=3.0 )
  add_rostest(launch/hztest.launch  ARGS hz:=20 resolution:=512 organized:=unorganized test_duration:=10 replaydelay:=10 hzerror:=3.0 )
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=512 organized:=unorganized test_duration:=10 replaydelay:=10 hzerror:=3.0 )
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=1024 organized:=unorganized test_duration:=10 replaydelay:=10 hzerror:=3.0 )
  add_rostest(launch/hztest.launch  ARGS hz:=20 resolution:=1024 organized:=unorganized test_duration:=10 replaydelay:=10 hzerror:=3.0 )
  add_rostest(launch/hztest.launch  ARGS hz:=10 resolution:=2048 organized:=unorganized test_duration:=10 replaydelay:=10 hzerror:=3.0 )

  endif ( CATKIN_RUNNING_TESTS )
  
  if ( CATKIN_ENABLE_TESTBENCH_TESTING )
    
  endif( CATKIN_ENABLE_TESTBENCH_TESTING ) 

endif(CATKIN_ENABLE_TESTING)


install(DIRECTORY data    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
