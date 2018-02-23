cmake_minimum_required(VERSION 2.8.3)
project(airsim_interface)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Need to dynamically link Boost's log library for loggers shared across multiple targets/libraries
add_definitions(-DBOOST_LOG_DYN_LINK)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps roscpp roslib nodelet anm_msgs cmake_modules)
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost COMPONENTS REQUIRED log_setup log)
find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include AirLib/include
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS Boost Eigen
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    AirLib/include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} # NOTE: No 'S' for Boost's include directory
    ${EIGEN_INCLUDE_DIRS}
)

# Nodelet
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN_LIBRARIES}
)

# Node executable
add_executable(${PROJECT_NAME}_node
    src/airsim_interface_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_nodelet ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Need to explicitly list nodelet.xml for installation
install(FILES
    nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
