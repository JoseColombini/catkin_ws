cmake_minimum_required(VERSION 2.8.3)

project(dronecontrol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()
 
include_directories(${catkin_INCLUDE_DIRS})
 
#add_executable(quadrado src/quadrado.cpp)
#add_dependencies(quadrado imav_drone_control_generate_messages_cpp)
#target_link_libraries(quadrado ${catkin_LIBRARIES})

add_executable(grid_search src/grid.cpp)
add_dependencies(grid_search dronecontrol_generate_messages_cpp)
target_link_libraries(grid_search ${catkin_LIBRARIES})
add_executable(Talker src/Talker.cpp)
add_dependencies(Talker dronecontrol_generate_messages_cpp)
target_link_libraries(Talker ${catkin_LIBRARIES})
add_executable(Battery src/Battery.cpp)
add_dependencies(Battery dronecontrol_generate_messeges_cpp)
target_link_libraries(Battery ${catkin_LIBRARIES})
add_executable(Talker_A src/Talker_A.cpp)
add_dependencies(Talker_A dronecontrol_generate_messeges_cpp)
target_link_libraries(Talker_A ${catkin_LIBRARIES})
#add_executable(grid_search_v2 src/grid_search_v2.cpp)
#add_dependencies(grid_search_v2 imav_drone_control_generate_messages_cpp)
#target_link_libraries(grid_search_v2 ${catkin_LIBRARIES})
add_executable(control src/control.cpp)
add_dependencies(control dronecontrol_generate_messeges_cpp)
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(Delivery src/Delivery.cpp)
add_dependencies(Delivery dronecontrol_generate_messeges_cpp)
target_link_libraries(Delivery ${catkin_LIBRARIES})
