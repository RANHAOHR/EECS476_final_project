cmake_minimum_required(VERSION 2.8.3)
project(gripper_controller)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

cs_add_library(gripper_controller src/gripper_controller.cpp)  

cs_add_executable(gripper_controller_main src/gripper_controller_main.cpp)
 
target_link_libraries(gripper_controller_main gripper_controller ${catkin_LIBRARIES})

cs_install()
cs_export()
