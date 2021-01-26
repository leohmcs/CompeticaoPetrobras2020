cmake_minimum_required(VERSION 3.2.0)
project(petrobras_challenge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## PG: Tudo que for escrito abaixo eh pra tentar compilar certinho o file cpp, soh o gazebo_ros q era default
## PG: Adicionei os nomes das biliotecas que usei no fst_version.cpp
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  mrs_msgs
  std_msgs
  roscpp
  )

## PG: Coloquei no CATKIN_DEPENDS o nome das bibliotecas novamente
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    mrs_msgs 
    geometry_msgs 
    std_msgs
    gazebo_ros
  )

## PG
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## PG
add_executable(fst_version src/fst_version.cpp)
target_link_libraries(fst_version ${catkin_LIBRARIES})
## add_dependencies(fst_version petrobras_challenge_generate_messages_cpp)

add_executable(teste_goto src/teste_goto.cpp)
target_link_libraries(teste_goto ${catkin_LIBRARIES})

