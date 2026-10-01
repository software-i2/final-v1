cmake_minimum_required(VERSION 3.0.2)
project(final_v1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # Eigen is ~100x slower unoptimised
endif()
add_compile_options(-std=c++17 -Wall -Wextra)

set(DEPS geometry_msgs roscpp sensor_msgs std_msgs std_srvs urdf visualization_msgs)
find_package(catkin REQUIRED COMPONENTS ${DEPS} message_generation)
find_package(Eigen3 REQUIRED)

add_message_files(FILES PickState.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS ${DEPS} message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/arm.cpp src/bpl.cpp src/cloud.cpp src/collision.cpp src/driver.cpp src/follow.cpp src/fsm.cpp src/mine.cpp
            src/park.cpp src/pick.cpp src/plan.cpp src/track.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# The library is ROS-free; each node file is all the ROS1 around it.
foreach(node driver pick)
  add_executable(${node} src/${node}_node.cpp)
  target_link_libraries(${node} ${PROJECT_NAME})
  add_dependencies(${node} ${PROJECT_NAME}_generate_messages_cpp)
endforeach()

if(CATKIN_ENABLE_TESTING)
  find_package(kdl_parser REQUIRED)
  catkin_add_gtest(sanity test/sanity.cpp)  # does each part give the right answer: catkin test final_v1
  target_include_directories(sanity PRIVATE ${kdl_parser_INCLUDE_DIRS})
  target_link_libraries(sanity ${PROJECT_NAME} ${kdl_parser_LIBRARIES})
  target_compile_definitions(sanity PRIVATE FINAL_V1_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
