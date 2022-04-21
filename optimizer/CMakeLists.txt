cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)


find_package(cartesian_interface REQUIRED)

find_package(g2o REQUIRED)

find_package(fcl 0.6.1 QUIET)

find_package(urdf REQUIRED)

find_package(ompl 1.5.0 REQUIRED)

find_package(matlogger2 REQUIRED)

find_package(CSparse REQUIRED)

include_directories(
    include
    ${OMPL_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${matlogger2_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})

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
target_link_libraries(joint ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} OpenSoT::OpenSoT fcl g2o::types_slam3d)
    ${CSPARSE_INCLUDE_DIR})

add_library(contact SHARED
            src/environment/contact/contact.cpp
            src/environment/contact/vertex_contact.cpp
            src/environment/contact/edge_relative_pose.cpp
            src/environment/contact/edge_collision.cpp
            src/environment/contact/edge_steering.cpp)
target_link_libraries(contact ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} g2o::core)

add_library(planner SHARED
            src/planner/dcm_planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} contact)

add_library(simulator SHARED
    src/simulator/simulator.cpp)         
target_link_libraries(simulator joint
                                g2o::core
                                g2o::types_slam3d
                                g2o::solver_csparse
                                ${catkin_LIBRARIES}
                                ${XBotInterface_LIBRARIES}
                                ${cartesian_interface_LIBRARIES}
                                ${OpenSoT_LIBRARIES}
                                yaml-cpp)

add_library(optimizer_contact src/optimizer/optimizer_contact.cpp)
target_link_libraries(optimizer_contact ${catkin_LIBRARIES}
                                        g2o::core
                                        g2o::solver_csparse)

add_library(controller src/controller/robot_controller.cpp)
target_link_libraries(controller ${cartesian_interface_LIBRARIES})

add_executable(robot_controller_node src/nodes/robot_controller_node.cpp)
target_link_libraries(robot_controller_node controller)

add_executable(urdf_parser_test_node test/urdf_parser_test_node.cpp)
target_link_libraries(urdf_parser_test_node ${catkin_LIBRARIES} ${urdf_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(planner_executor src/nodes/planner_executor_node src/nodes/planner_executor.cpp)
target_link_libraries(planner_executor ${catkin_LIBRARIES}
                                       ${XBotInterface_LIBRARIES}
                                       ${cartesian_interface_LIBRARIES}
                                       matlogger2::matlogger2
                                       planner
                                       optimizer_contact)

install(TARGETS joint
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS contact
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS simulator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS optimizer_contact
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS robot_controller_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS urdf_parser_test_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS planner_executor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


