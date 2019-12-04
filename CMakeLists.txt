cmake_minimum_required(VERSION 2.8.3)
project(covari)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES covari
  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

add_executable(${PROJECT_NAME} src/Covari.cpp src/main.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

