cmake_minimum_required(VERSION 3.1.0)
project(muvt_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
    eigen_conversions
    visualization_msgs
    interactive_markers
    muvt_core
    )
find_package(cartesian_interface REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS})

add_executable(test_node src/planner_executor.cpp src/planner_executor_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} yaml-cpp)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE
        )

install(TARGETS test_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
