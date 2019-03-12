cmake_minimum_required(VERSION 2.8.3)
project(ouster_ros)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED
  COMPONENTS
  am_utils
  latency_testing
  message_generation
  ouster_client
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  )

add_message_files(DIRECTORY msg FILES PacketMsg.msg)
generate_messages(DEPENDENCIES  sensor_msgs std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime pcl_conversions pcl_ros  roscpp  sensor_msgs std_msgs
)

add_library(${PROJECT_NAME} STATIC src/os1_ros.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(os1_node src/os1_node.cpp)

target_link_libraries(os1_node ${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS os1_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES os1.launch
   DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
   )
