cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)

find_package(PCL REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS})

add_executable(vision_node src/nodes/vision_node.cpp src/vision/point_cloud_manager.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS vision_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
