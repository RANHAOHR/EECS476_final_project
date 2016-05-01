cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)

#find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    actionlib)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(cwru_pcl_utils REQUIRED)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
        ${catkin_INCLUDE_DIRS}
    LIBRARIES
        motion_planning_lib
    CATKIN_DEPENDS
        actionlib)


add_library(motion_planning_lib src/motion_planning_lib.cpp)

#catkin_simple()

# example boost usage
#find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
#cs_add_library(motion_planning_lib src/motion_planning_lib.cpp)

# Executables
# cs_add_executable(example src/example.cpp)


#target_link_library(motion_planning_lib ${catkin_LIBRARIES})

#cs_install()
#cs_export()
