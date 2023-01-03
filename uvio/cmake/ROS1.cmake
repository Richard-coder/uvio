cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_core ov_init ov_msckf nodelet)
find_package(mdek_uwb_driver)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (catkin_FOUND AND ENABLE_ROS)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_core ov_init ov_msckf nodelet mdek_uwb_driver
            INCLUDE_DIRS src/
            LIBRARIES uvio_lib uvio_nodelet
    )
endif ()


# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${mdek_uwb_driver_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
        ${mdek_uwb_driver_LIBRARIES}
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
    src/ros/UVIOROS1Visualizer.cpp
    src/core/UVioManager.cpp
    src/state/UVioState.cpp
)

file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(uvio_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(uvio_lib ${thirdparty_libraries})
target_include_directories(uvio_lib PUBLIC src/)
install(TARGETS uvio_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

##################################################
# Make binary files!
##################################################

if (catkin_FOUND AND ENABLE_ROS)

    add_library(uvio_nodelet src/nodelet_uvio.cpp)
    target_link_libraries(uvio_nodelet uvio_lib ${thirdparty_libraries})
    install(TARGETS uvio_nodelet
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

endif ()


##################################################
# Launch files!
##################################################

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)





