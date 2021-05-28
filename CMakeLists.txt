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
    )
    
find_package(cartesian_interface REQUIRED)

find_package(g2o REQUIRED)

add_service_files(
    FILES
    SetObstacle.srv)
    
generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs# Or other packages containing msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES teb_test
    CATKIN_DEPENDS roscpp message_runtime geometry_msgs
    DEPENDS
    )
    
include_directories(
    include    
    ${g2o_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    )

add_library(environment SHARED
            src/environment/edge_xyz.cpp
            src/environment/robot_pos.cpp
            src/environment/vertex_robot_pos.cpp
            src/environment/edge_robot_pos.cpp
            src/environment/edge_robot_vel.cpp
            src/environment/edge_joint_limits.cpp)
target_link_libraries(environment ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} OpenSoT::OpenSoT)

add_library(simulator SHARED
            src/simulator/simulator.cpp
            src/simulator/optimizer.cpp)
target_link_libraries(simulator environment
                                g2o::core 
                                g2o::types_slam3d 
                                g2o::solver_csparse 
                                ${catkin_LIBRARIES} 
                                ${XBotInterface_LIBRARIES} 
                                ${cartesian_interface_LIBRARIES} 
                                ${OpenSoT_LIBRARIES}
                                yaml-cpp)

add_executable(main_test src/nodes/test.cpp)
target_link_libraries(main_test simulator environment g2o::solver_csparse ${catkin_LIBRARIES})

add_executable(optimizer src/nodes/optimizer_node.cpp src/simulator/optimizer.cpp)
target_link_libraries(optimizer simulator ${catkin_LIBRARIES})

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
        
install(TARGETS optimizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE
        )
