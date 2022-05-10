cmake_minimum_required(VERSION 3.1.0)
project(muvt_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    tf
    tf_conversions
    eigen_conversions
    visualization_msgs
    interactive_markers
    muvt_core
    )
find_package(cartesian_interface REQUIRED)
add_service_files(
    FILES
    SetWalkingDirection.srv)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS})

add_executable(test_node src/planner_executor.cpp src/planner_executor_node.cpp)
target_link_libraries(test_node ${cartesian_interface_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS test_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
