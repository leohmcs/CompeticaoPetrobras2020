cmake_minimum_required(VERSION 3.2.0)
project(petrobras_challenge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## PG: Tudo que for escrito abaixo eh pra tentar compilar certinho o file cpp, soh o gazebo_ros q era default
## PG: Adicionei os nomes das biliotecas que usei no fst_version.cpp
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  geometry_msgs
  mrs_msgs
  roscpp
  std_msgs
  gazebo_ros_link_attacher
  )

## PG: Coloquei no CATKIN_DEPENDS o nome das bibliotecas novamente
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    mrs_msgs 
    geometry_msgs 
    gazebo_ros
    std_msgs
    gazebo_ros_link_attacher
    cv_bridge
  )

## PG
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(fase4 src/fase4.cpp)
target_link_libraries(fase4 ${catkin_LIBRARIES})
# add_dependencies(fase4 ${catkin_EXPORTED_TARGETS})

add_executable(goto src/goto.cpp)
target_link_libraries(goto ${catkin_LIBRARIES})
# add_dependencies(goto ${catkin_EXPORTED_TARGETS})

add_executable(visao_wrapper src/visao_wrapper.cpp)
target_link_libraries(visao_wrapper ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ZBAR_LIBRARIES} zbar)
# add_dependencies(visao_wrapper ${catkin_EXPORTED_TARGETS})
