cmake_minimum_required(VERSION 3.1.0)
project(muvt_core)

add_compile_options(-std=c++14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    std_msgs
    interactive_markers
    visualization_msgs
    geometry_msgs
    eigen_conversions
    )

find_package(G2O REQUIRED)

find_package(CSparse REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    )
    
include_directories(
    include
    ${G2O_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR})

find_path(COMPILE_JOINT_LIBRARY base_dynamic_vertex.h PATHS ${g2o_INCLUDE_DIRS}/g2o/core/)
if(COMPILE_JOINT_LIBRARY)
    find_package(fcl 0.6.1 REQUIRED)
    find_package(OpenSoT REQUIRED)
    message("Found base_dynamic_vertex.h, compiling joint library")
    add_library(joint SHARED
                src/environment/joint/unary_edge.cpp
                src/environment/joint/robot_pos.cpp
                src/environment/joint/vertex_robot_pos.cpp
                src/environment/joint/edge_robot_pos.cpp
                src/environment/joint/edge_collision.cpp
                src/environment/joint/edge_robot_vel.cpp
                src/environment/joint/edge_joint_limits.cpp
                src/environment/joint/edge_task.cpp
                src/environment/joint/edge_trajectory_vel.cpp
                src/environment/joint/edge_kinematic.cpp
                src/environment/joint/edge_postural.cpp)
    target_link_libraries(joint ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} OpenSoT::OpenSoT fcl ${G2O_TYPES_SLAM3D})

    install(TARGETS joint
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    message("Did not find base_dynamic_vertex.h, skipping joint library compilation")
endif()

add_library(contact SHARED
            src/environment/contact/contact.cpp
            src/environment/contact/vertex_contact.cpp
            src/environment/contact/edge_relative_pose.cpp
            src/environment/contact/edge_multi_relative_poses.cpp
            src/environment/contact/edge_collision.cpp
            src/environment/contact/edge_steering.cpp)
target_link_libraries(contact ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${G2O_CORE_LIBRARY})

add_library(planner SHARED
            src/planner/dcm_planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} contact)

add_library(optimizer src/optimizer/optimizer.cpp)
target_link_libraries(optimizer ${catkin_LIBRARIES} ${G2O_CORE_LIBRARY} ${CSPARSE_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_CSPARSE})

add_executable(test_node src/nodes/planner_executor_node.cpp src/nodes/planner_executor.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} yaml-cpp optimizer planner contact)

install(TARGETS contact optimizer planner test_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
