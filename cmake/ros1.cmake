## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  message_generation
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# Add the catkin include directory and tasks to the target
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Generate Service Files
add_service_files(
  FILES
  ${SRV_FILES}
)

# Genertate Message Files
add_message_files(
  DIRECTORY msg
  FILES ${MSG_FILES}
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare the catkin package with it's dependencies
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
)

# Set some variables for the parent CMake
set(ROS_MSCL_COMMON_PACKAGE_LIB_DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
set(ROS_MSCL_COMMON_PACKAGE_BIN_DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}")
set(ROS_MSCL_COMMON_PACKAGE_INCLUDE_DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
set(ROS_MSCL_COMMON_PACKAGE_SHARE_DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

#############
## Testing ##
#############

## Run roslint on only the node files for now
# TODO: Add more files as needed
# TODO: rosling on all the files
#roslint_cpp(src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)