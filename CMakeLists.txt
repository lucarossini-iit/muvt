cmake_minimum_required(VERSION 3.1.0)
project(teb_test)

add_compile_options(-std=c++14)

option(BUILD_VISION "Build vision executables" ON)
option(BUILD_OPTIMIZER "Build optimizer executables and libraries" ON)

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
    ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(PROGRAMS scripts/xbotcore.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(BUILD_VISION)
    add_subdirectory(src/src_vision)
endif()

if(BUILD_OPTIMIZER)
    add_subdirectory(src/src_optimizer)
endif()




