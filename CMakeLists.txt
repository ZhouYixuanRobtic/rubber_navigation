cmake_minimum_required(VERSION 2.8.3)
project(rubber_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib geometric_shapes geometry_msgs
  message_generation move_base_msgs nav_msgs
  roscpp rospy sensor_msgs std_msgs tf2_eigen tf2_geometry_msgs roslib
  tf2_ros tf
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
        FILES
        WheelStatus.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        roscpp
        tf
        nav_msgs
    DEPENDS
        YAML
        EIGEN3
        Boost
)


###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)
link_directories(
        ${catkin_LIBRARY_DIRS}
        /user/local/lib
        ${YAML_LIBRARIES}
)

add_library(Navi_SerialManager src/SerialManager.cpp)
target_link_libraries(Navi_SerialManager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(NaviJoyTeleop src/JoyTeleop.cpp)
target_link_libraries(NaviJoyTeleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(NaviSerialManager src/NaviSerialManager.cpp)
target_link_libraries(NaviSerialManager ${catkin_LIBRARIES} ${Boost_LIBRARIES} Navi_SerialManager)

add_library(BaseController src/BaseController.cpp)
target_link_libraries(BaseController ${catkin_LIBRARIES} ${Boost_LIBRARIES} NaviSerialManager yaml-cpp)
add_dependencies(BaseController ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_library(NavCore src/NavCore.cpp)
target_link_libraries(NavCore ${catkin_LIBRARIES} ${Boost_LIBRARIES} BaseController)


add_executable(odomFilter src/odomFilter.cpp)
target_link_libraries(odomFilter ${catkin_LIBRARIES})

add_executable(baseOnly src/BaseOnly.cpp)
target_link_libraries(baseOnly ${catkin_LIBRARIES} BaseController NaviJoyTeleop)

add_executable(simpleNav src/simpleNav.cpp)
target_link_libraries(simpleNav ${catkin_LIBRARIES} BaseController NavCore ${Boost_LIBRARIES} NaviJoyTeleop)

add_executable(goalSaver src/goalSaver.cpp)
target_link_libraries(goalSaver ${catkin_LIBRARIES} NaviJoyTeleop)