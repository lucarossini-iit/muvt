cmake_minimum_required(VERSION 3.1.0)
project(teb_test)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    std_msgs
    interactive_markers
    message_generation
    tf tf_conversions
    visualization_msgs
    geometry_msgs
    eigen_conversions
    urdf
    )

find_package(cartesio_planning REQUIRED)

find_package(cartesian_interface REQUIRED)

find_package(g2o REQUIRED)

find_package(fcl 0.6.1 QUIET)

find_package(PCL REQUIRED)

find_package(urdf REQUIRED)

find_package(ompl 1.5.0 REQUIRED)

add_message_files(
    FILES
    ObjectMessage.msg
    ObjectMessageString.msg)

add_service_files(
    FILES
    SetObstacle.srv)
    
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES teb_test
    CATKIN_DEPENDS roscpp message_runtime geometry_msgs
    DEPENDS
    )
    
include_directories(
    include    
    ${OMPL_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${cartesio_planning_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

add_library(environment SHARED
            src/environment/unary_edge.cpp
            src/environment/edge_xyz.cpp
            src/environment/robot_pos.cpp
            src/environment/vertex_robot_pos.cpp
            src/environment/edge_robot_pos.cpp
            src/environment/edge_collision.cpp
            src/environment/edge_robot_vel.cpp
            src/environment/edge_joint_limits.cpp
            src/environment/edge_task.cpp)
target_link_libraries(environment ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} OpenSoT::OpenSoT fcl g2o::types_slam3d)

add_library(simulator SHARED
            src/simulator/simulator.cpp
            src/simulator/optimizer_test.cpp)
target_link_libraries(simulator environment
                                g2o::core 
                                g2o::types_slam3d 
                                g2o::solver_csparse 
                                ${catkin_LIBRARIES} 
                                ${XBotInterface_LIBRARIES} 
                                ${cartesian_interface_LIBRARIES} 
                                ${OpenSoT_LIBRARIES}
                                yaml-cpp)

add_executable(vision_node src/nodes/vision_node.cpp src/vision/point_cloud_manager.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(main_test src/nodes/test.cpp)
target_link_libraries(main_test simulator environment g2o::solver_csparse ${catkin_LIBRARIES})

#add_executable(optimizer_test src/nodes/optimizer_node.cpp src/simulator/optimizer_test.cpp)
#target_link_libraries(optimizer_test simulator ${catkin_LIBRARIES})

add_executable(object_recognition_test test/object_recognition_test.cpp)
target_link_libraries(object_recognition_test ${PCL_LIBRARIES})

add_executable(optimizer src/nodes/optimizer_node.cpp src/optimizer/optimizer.cpp)
target_link_libraries(optimizer environment
                                simulator
                                ${catkin_LIBRARIES}
                                ${cartesio_planning_LIBRARIES}
                                g2o::core
                                g2o::solver_csparse)

add_executable(robot_controller_node src/nodes/robot_controller_node.cpp src/controller/robot_controller.cpp)
target_link_libraries(robot_controller_node environment ${cartesian_interface_LIBRARIES})

add_executable(urdf_parser_test_node test/urdf_parser_test_node.cpp)
target_link_libraries(urdf_parser_test_node ${catkin_LIBRARIES} ${urdf_LIBRARIES} ${XBotInterface_LIBRARIES})

catkin_install_python(PROGRAMS python/main_window.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS main_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS environment
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(TARGETS simulator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
#install(TARGETS optimizer_test
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS vision_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS object_recognition_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS optimizer
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

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(PROGRAMS scripts/xbotcore.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
