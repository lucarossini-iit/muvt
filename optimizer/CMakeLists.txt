cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)

find_package(cartesio_planning REQUIRED)

find_package(cartesian_interface REQUIRED)

find_package(g2o REQUIRED)

find_package(fcl 0.6.1 QUIET)

find_package(urdf REQUIRED)

find_package(ompl 1.5.0 REQUIRED)

find_package(gazebo REQUIRED)

include_directories(
    include
    ${OMPL_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${cartesio_planning_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})

add_library(environment SHARED
            src/environment/unary_edge.cpp
            src/environment/edge_xyz.cpp
            src/environment/robot_pos.cpp
            src/environment/vertex_robot_pos.cpp
            src/environment/edge_robot_pos.cpp
            src/environment/edge_collision.cpp
            src/environment/edge_robot_vel.cpp
            src/environment/edge_joint_limits.cpp
            src/environment/edge_task.cpp
            src/environment/edge_trajectory_vel.cpp
            src/environment/edge_kinematic.cpp)
target_link_libraries(environment ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} OpenSoT::OpenSoT fcl g2o::types_slam3d)

add_library(simulator SHARED
    src/simulator/simulator.cpp)         
target_link_libraries(simulator environment
                                g2o::core
                                g2o::types_slam3d
                                g2o::solver_csparse
                                ${catkin_LIBRARIES}
                                ${XBotInterface_LIBRARIES}
                                ${cartesian_interface_LIBRARIES}
                                ${OpenSoT_LIBRARIES}
                                yaml-cpp)

# gazebo model plugin
add_library(move_box gazebo_plugin/move_box.cpp)
target_link_libraries(move_box ${GAZEBO_LIBRARIES})

add_library(controller src/controller/robot_controller.cpp
                       src/controller/robot_controller_car_model.cpp)
target_link_libraries(controller ${cartesian_interface_LIBRARIES})

add_executable(main_test src/nodes/test.cpp)
target_link_libraries(main_test simulator environment g2o::solver_csparse ${catkin_LIBRARIES})

add_executable(optimizer src/nodes/optimizer_node.cpp src/optimizer/optimizer.cpp)
target_link_libraries(optimizer environment
                                simulator
                                ${catkin_LIBRARIES}
                                ${cartesio_planning_LIBRARIES}
                                g2o::core
                                g2o::solver_csparse)

add_executable(car_model src/nodes/car_model_node.cpp src/optimizer/optimizer.cpp)
target_link_libraries(car_model environment
                                simulator
                                ${catkin_LIBRARIES}
                                ${cartesio_planning_LIBRARIES}
                                g2o::core
                                g2o::solver_csparse)

add_executable(robot_controller_node src/nodes/robot_controller_node.cpp)
target_link_libraries(robot_controller_node controller)

add_executable(robot_controller_car_model_node src/nodes/robot_controller_car_model_node.cpp)
target_link_libraries(robot_controller_car_model_node controller)

add_executable(urdf_parser_test_node test/urdf_parser_test_node.cpp)
target_link_libraries(urdf_parser_test_node ${catkin_LIBRARIES} ${urdf_LIBRARIES} ${XBotInterface_LIBRARIES})

install(TARGETS main_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS environment
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS move_box
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

install(TARGETS optimizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS car_model
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS robot_controller_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS robot_controller_car_model_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS urdf_parser_test_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

