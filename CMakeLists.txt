cmake_minimum_required(VERSION 3.0)
project(teb_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    std_msgs
    trajectory_msgs
    sensor_msgs
    message_generation
    kdl_parser
    robot_state_publisher
    tf tf_conversions
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    visualization_msgs
    )

find_package(Eigen3 REQUIRED)

find_package(cartesian_interface REQUIRED)

find_package(g2o REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES teb_test
    CATKIN_DEPENDS roscpp message_runtime
    DEPENDS
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(environment SHARED 
            src/environment/obstacle.cpp 
            src/environment/se3.cpp)
            
target_link_libraries(environment ${catkin_LIBRARIES})

add_executable(main_test src/nodes/test.cpp)
target_link_libraries(main_test ${catkin_LIBRARIES} ${teb_local_planner_LIBRARIES} ${g2o_LIBRARIES})

install(TARGETS main_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS environment
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


