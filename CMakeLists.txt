cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "swerve_drivetrain_node")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(${PROJECT_NAME})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation ck_ros_msgs_node ck_ros_base_msgs_node ck_utilities_node hmi_agent_node frc_robot_utilities_node dynamic_reconfigure swerve_trajectory_node)

# add_message_files(
#   FILES
#   Swerve_Drivetrain_Diagnostics.msg
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

generate_dynamic_reconfigure_options(
  cfg/DriveTuning.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ck_ros_base_msgs_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS
  roscpp
  message_generation
  message_runtime
  ck_ros_msgs_node
  ck_ros_base_msgs_node
  nav_msgs
  ck_utilities_node
  hmi_agent_node
  dynamic_reconfigure
  frc_robot_utilities_node
  swerve_trajectory_node
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB_RECURSE executable_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/test/src/*.cpp"
)


add_executable(${PROJECT_NAME} ${executable_sources})
target_include_directories(${PROJECT_NAME} PUBLIC "include")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries.
  add_library(t${PROJECT_NAME} ${executable_sources})
  target_include_directories(t${PROJECT_NAME} PUBLIC "include")
  target_link_libraries(t${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)

  add_dependencies(t${PROJECT_NAME} ${PROJECT_NAME} ${t${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(${PROJECT_NAME}-test ${test_sources})
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test t${PROJECT_NAME})
    target_include_directories(${PROJECT_NAME}-test PUBLIC "include" "test/include")
    add_dependencies(${PROJECT_NAME}-test t${PROJECT_NAME} ${t${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
endif()
