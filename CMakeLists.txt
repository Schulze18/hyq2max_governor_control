cmake_minimum_required(VERSION 3.0.2)
project(hyq2max_governor_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_manager
  nav_msgs
  roscpp
  rospy
  std_msgs
  hyq2max_joints_position_controller
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
#   control_msgs#   nav_msgs#   std_msgs
)


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
    hyq2max_governor_control
    hyq2max_jacobian_functions
#  CATKIN_DEPENDS control_msgs controller_manager nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hyq2max_governor_control.cpp
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hyq2max_governor_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_library(hyq2max_jacobian_functions src/hyq2max_jacobian_functions.cpp)
add_dependencies(hyq2max_jacobian_functions ${PROJECT_NAME}_gencpp)
target_link_libraries(hyq2max_jacobian_functions ${catkin_LIBRARIES})


add_library(hyq2max_governor_control src/hyq2max_governor_control.cpp)
add_dependencies(hyq2max_governor_control hyq2max_governor_control_generate_messages_cpp ${PROJECT_NAME}_gencpp)
target_link_libraries(hyq2max_governor_control ${catkin_LIBRARIES})

add_executable(hyq2max_governor_control_node src/hyq2max_governor_control_node.cpp)
target_link_libraries(hyq2max_governor_control_node hyq2max_jacobian_functions ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(hyq2max_governor_control_node hyq2max_governor_control_generate_messages_cpp ${PROJECT_NAME}_gencpp)


## Mark executables and/or libraries for installation
#install(TARGETS
#  hyq2max_jacobian_functions
#  hyq2max_governor_control
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)