cmake_minimum_required(VERSION 2.8.3)
project(schunk_rrt)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fcl
  roscpp
  schunk_kinematics
  std_msgs
  message_generation
  roslib
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)

find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

add_message_files(
	FILES 
	RRTPath.msg
	RRTPoint.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  rrt_kdtree

  CATKIN_DEPENDS 
  roslib
  fcl
  roscpp
  schunk_kinematics

  DEPENDS
  system_lib 
  Eigen 
  Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

ADD_LIBRARY(rrt_kdtree src/kdtree.c)

ADD_EXECUTABLE(dynamic_obstacle src/dynamic_obstacle.cpp)
TARGET_LINK_LIBRARIES(dynamic_obstacle ${catkin_LIBRARIES})

ADD_LIBRARY(mobileRRT src/mobile_rrt/mobile_rrt.cpp)
TARGET_LINK_LIBRARIES(mobileRRT ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} rrt_kdtree)

ADD_EXECUTABLE(test_mobile_rrt src/mobile_rrt/test_mobile_rrt.cpp)
TARGET_LINK_LIBRARIES(test_mobile_rrt mobileRRT ${catkin_LIBRARIES})

ADD_LIBRARY(DynamicRRT src/mobile_rrt/dynamic_rrt.cpp)
TARGET_LINK_LIBRARIES(DynamicRRT ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} mobileRRT)

ADD_EXECUTABLE(test_dynamic_rrt src/mobile_rrt/test_dynamic_rrt.cpp)
TARGET_LINK_LIBRARIES(test_dynamic_rrt mobileRRT ${catkin_LIBRARIES} DynamicRRT)

ADD_EXECUTABLE(obstacle_publisher src/mobile_rrt/obstacle.cpp)
TARGET_LINK_LIBRARIES(obstacle_publisher ${catkin_LIBRARIES})

ADD_LIBRARY(cubicSpline src/arm_rrt/cubic.cpp)

ADD_LIBRARY(ArmRRT src/arm_rrt/arm_rrt.cpp)
TARGET_LINK_LIBRARIES(ArmRRT cubicSpline rrt_kdtree ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

ADD_EXECUTABLE(test_arm_rrt src/arm_rrt/test_arm_rrt.cpp)
TARGET_LINK_LIBRARIES(test_arm_rrt ArmRRT ${catkin_LIBRARIES} ${Boost_LIBRARIES})

ADD_LIBRARY(DynamicArmRRT src/arm_rrt/dynamic_rrt.cpp)
TARGET_LINK_LIBRARIES(DynamicArmRRT ArmRRT ${catkin_LIBRARIES} ${Boost_LIBRARIES})

ADD_EXECUTABLE(test_dynamic_arm_rrt src/arm_rrt/test_dynamic_rrt.cpp)
TARGET_LINK_LIBRARIES(test_dynamic_arm_rrt DynamicArmRRT ArmRRT ${catkin_LIBRARIES} ${Boost_LIBRARIES})

ADD_EXECUTABLE(realtime_publisher src/arm_rrt/realtime_publisher.cpp)
TARGET_LINK_LIBRARIES(realtime_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
