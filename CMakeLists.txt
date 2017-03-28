cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_control)

find_package(catkin REQUIRED COMPONENTS
  # lwr_controllers
  roscpp
  kdl_wrapper
  kdl_parser
  gazebo_msgs
  # gazebo_ros
)
# find_package(gazebo REQUIRED)
find_package(Armadillo REQUIRED)

add_definitions (-std=c++11)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES dual_manipulation_control
 CATKIN_DEPENDS 
 # lwr_controllers 
 roscpp 
 kdl_wrapper 
 kdl_parser
 gazebo_msgs
 # gazebo_ros
 # DEPENDS gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${GAZEBO_INCLUDE_DIRS}
  include
)
include_directories(${ARMADILLO_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


# Declare a C++ executable
# add_executable(manager src/force_position_control.cpp)
# ## Add cmake target dependencies of the executable
# ## same as for the library above
# add_dependencies(manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # Specify libraries to link a library or executable target against
# target_link_libraries(manager
#   ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} 
#   # ${GAZEBO_LIBRARIES} 
# )


add_executable(go_home src/home.cpp)
add_dependencies(go_home ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Specify libraries to link a library or executable target against
target_link_libraries(go_home
  ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} 
  # ${GAZEBO_LIBRARIES} 
)

add_executable(single_robot_test src/single_robot_test.cpp)
add_dependencies(single_robot_test ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_robot_test ${catkin_LIBRARIES})


add_executable(dual_robot_test src/dual_robot_test.cpp)
add_dependencies(dual_robot_test ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_robot_test ${catkin_LIBRARIES}  ${GAZEBO_LIBRARIES} )