cmake_minimum_required(VERSION 3.0.2)
project(fpp_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp
	mbf_costmap_core
	mbf_msgs
	fpp_msgs
	nav_core
	nav_msgs
	geometry_msgs
	visualization_msgs
	std_srvs
	costmap_2d
	dynamic_reconfigure
	fp_utils)

find_package(Eigen3 REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS 
		include
	LIBRARIES 
		fpp_ros
	CATKIN_DEPENDS 
		roscpp
		pluginlib
		mbf_costmap_core
		mbf_msgs
		fpp_msgs
		nav_core
		geometry_msgs
		tf
		dynamic_reconfigure
		fp_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
set(FPP_ROS_SOURCES
	include/fpp_ros/fpp_ros.h
	src/fpp_ros.cpp

	include/fpp_ros/fpp_controller_base.h
	src/fpp_controller_base.cpp
	
	include/fpp_ros/fpp_controller_master.h
	src/fpp_controller_master.cpp

	include/fpp_ros/fpp_controller_slave.h
	src/fpp_controller_slave.cpp)
add_library(fpp_ros ${FPP_ROS_SOURCES})

set(PATH_PLANNER_SOURCES
	include/fpp_ros/path_planner/splined_relaxed_a_star.h
	src/path_planner/splined_relaxed_a_star.cpp)
add_library(path_planner ${PATH_PLANNER_SOURCES})

set(DATA_CLASS_SOURCES
	include/fpp_ros/data_classes/robot_info.h

	include/fpp_ros/data_classes/path_planner_types.h

	include/fpp_ros/data_classes/fpp_controller_param.h
	
	include/fpp_ros/data_classes/fpp_param_manager.h
	src/data_classes/fpp_param_manager.cpp

	include/fpp_ros/data_classes/ras_param_manager.h
	src/data_classes/ras_param_manager.cpp)
add_library(data_classes ${DATA_CLASS_SOURCES})

set(PLAN_TRANSFORMATION_SOURCES
	include/fpp_ros/plan_transformation/rigid_plan_transformation.h
	src/plan_transformation/rigid_plan_transformation.cpp)
add_library(plan_transformation ${PLAN_TRANSFORMATION_SOURCES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(dyn_reconfig_relay_node src/nodes/dyn_reconfig_relay_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(general_types PROPERTIES LINKER_LANGUAGE CXX)

## This setting of properties is so that I can only use a .h file without a .cpp file
## cpp would be empty as these files only contain types
set_target_properties(data_classes PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(path_planner_types PROPERTIES LINKER_LANGUAGE CXX)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(fpp_ros 
	data_classes
	path_planner
	plan_transformation
	${catkin_LIBRARIES})

target_link_libraries(path_planner 
	data_classes
	${catkin_LIBRARIES})

target_link_libraries(dyn_reconfig_relay_node 
	${catkin_LIBRARIES})

target_link_libraries(plan_transformation 
	${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/nodes/fpp_meta_data_sync_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(
# 	TARGETS fpp_ros
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
# 	DIRECTORY include/fpp_ros/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fpp_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
