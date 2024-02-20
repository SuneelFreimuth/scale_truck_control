cmake_minimum_required(VERSION 2.8.3)
project(scale_truck_control)

set(CMAKE_CXX_STANDARD 17)

set(ENABLE_DRAFT ON)

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp 
    rospy 
    std_msgs
    message_generation
    sensor_msgs
    image_transport
    geometry_msgs
    cv_bridge
    obstacle_detector 
    scale_truck_control_msgs
    scale_truck_control_lane_detector
)

# add_message_files(
#   FILES
#     lane_coef.msg
#     lane.msg
#     xav2lrc.msg
#     lrc2xav.msg
#     lrc2ocr.msg
#     ocr2lrc.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    obstacle_detector
    scale_truck_control_msgs
    scale_truck_control_lane_detector
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
  src/lane_detect.cpp
  src/lrc.cpp
  src/ScaleTruckController.cpp
)

#
# Build libs
#

add_library(${PROJECT_NAME}_lib
  ${PROJECT_LIB_FILES}	
)

add_executable(${PROJECT_NAME}
  nodes/control.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  pthread
  stdc++
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

add_library(LRC_lib
  ${PROJECT_LIB_FILES}	
)

add_executable(LRC
  nodes/lrc.cpp
)

add_dependencies(LRC
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(LRC_lib
  pthread
  stdc++
  ${catkin_LIBRARIES}
)

target_link_libraries(LRC
  LRC_lib
)

