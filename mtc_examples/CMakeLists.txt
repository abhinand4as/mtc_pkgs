cmake_minimum_required(VERSION 2.8.3)
project(mtc_examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        moveit_core
        moveit_task_constructor_core
        moveit_ros_planning_interface
        rosparam_shortcuts
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(generate_pose_with_constrain src/generate_pose_with_constrain)
target_link_libraries(generate_pose_with_constrain ${catkin_LIBRARIES})
