cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

find_package(OpenCV 3 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

project(research)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_geometry
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
  tf
  tf2
  darknet_ros_msgs
  pcl_ros
  pcl_conversions
  trajectory_msgs
  moveit_msgs
  kinova_msgs
  moveit_visual_tools
)



find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    kinova_msgs
)
find_package(Eigen REQUIRED)

find_package(kinova_driver REQUIRED)
find_package(kinova_msgs REQUIRED)


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
#   sensor_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES research
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/research.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(test_cv_bridge src/test_cv_bridge.cpp)
target_link_libraries(test_cv_bridge ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(color_detection src/color_detection.cpp)
target_link_libraries(color_detection ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_pickup src/autonomous_pickup.cpp)
target_link_libraries(autonomous_pickup ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_pickup_with_ziptie_surface_detection src/autonomous_pickup_with_ziptie_surface_detection.cpp)
target_link_libraries(autonomous_pickup_with_ziptie_surface_detection ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_pickup_point_cloud_ziptie_line_fitting_pose_estimation_considering_trajectory_optimization src/autonomous_pickup_point_cloud_ziptie_line_fitting_pose_estimation_considering_trajectory_optimization.cpp)
target_link_libraries(autonomous_pickup_point_cloud_ziptie_line_fitting_pose_estimation_considering_trajectory_optimization ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(line_segmentation src/line_segmentation.cpp)
target_link_libraries(line_segmentation ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(line_segmentation_filtered_withBB src/line_segmentation_filtered_withBB.cpp)
target_link_libraries(line_segmentation_filtered_withBB ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(hand_gesture_recognition src/hand_gesture_recognition.cpp)
target_link_libraries(hand_gesture_recognition ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(hand_gesture_recognition_kinect src/hand_gesture_recognition_kinect.cpp)
target_link_libraries(hand_gesture_recognition_kinect ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(hand_gesture_recognition_stable src/hand_gesture_recognition_stable.cpp)
target_link_libraries(hand_gesture_recognition_stable ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(hand_gesture_recognition_test src/hand_gesture_recognition_test.cpp)
target_link_libraries(hand_gesture_recognition_test ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(point_cloud_ziptie src/point_cloud_ziptie.cpp)
target_link_libraries(point_cloud_ziptie ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(example_pcl src/example_pcl.cpp)
target_link_libraries(example_pcl ${catkin_LIBRARIES})

add_executable(ziptie_bend_plan src/ziptie_bend_plan.cpp)
target_link_libraries(ziptie_bend_plan ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_pull_out src/autonomous_pull_out.cpp)
target_link_libraries(autonomous_pull_out ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(circle_detection src/circle_detection.cpp)
target_link_libraries(circle_detection ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(point_cloud_wire src/point_cloud_wire.cpp)
target_link_libraries(point_cloud_wire ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(plane_segmentation_pcl src/plane_segmentation_pcl.cpp)
target_link_libraries(plane_segmentation_pcl ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_adjustment_test src/autonomous_adjustment_test.cpp)
target_link_libraries(autonomous_adjustment_test ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(autonomous_adjustment src/autonomous_adjustment.cpp)
target_link_libraries(autonomous_adjustment ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(SocketPoseEstimation src/SocketPoseEstimation.cpp)
target_link_libraries(SocketPoseEstimation ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(SurpriseTask src/SurpriseTask.cpp)
target_link_libraries(SurpriseTask ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(SurpriseTask_new src/SurpriseTask_new.cpp)
target_link_libraries(SurpriseTask_new ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(SurpriseTask_final src/SurpriseTask_final.cpp)
target_link_libraries(SurpriseTask_final ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(modeling_sim src/modeling_sim.cpp)
target_link_libraries(modeling_sim ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)

add_executable(kinova_cart_vel src/kinova_cart_vel.cpp)
target_link_libraries(kinova_cart_vel ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_imgproc)
add_dependencies(kinova_cart_vel ${catkin_EXPORTED_TARGETS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_research.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
