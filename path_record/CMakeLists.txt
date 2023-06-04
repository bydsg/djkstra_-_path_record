
cmake_minimum_required(VERSION 2.8.3)
project(path_record)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
roscpp
tf

)
catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp  

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node 
  src/path_record.cpp  

)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${can_parser_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
