cmake_minimum_required(VERSION 2.8.3)
project(codebase)

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

add_executable(cartesian_tutorial src/cartesian_tutorial.cpp)
target_link_libraries(cartesian_tutorial ${catkin_LIBRARIES})

add_executable(modular_tutorial src/modular_tutorial.cpp)
target_link_libraries(modular_tutorial ${catkin_LIBRARIES})

add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES})

add_executable(mtc_test src/mtc_test.cpp)
target_link_libraries(mtc_test ${catkin_LIBRARIES})

add_executable(pick_place_tutorial2 src/pick_place_tutorial2.cpp)
target_link_libraries(pick_place_tutorial2 ${catkin_LIBRARIES})

