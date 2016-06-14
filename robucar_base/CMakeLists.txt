cmake_minimum_required(VERSION 2.8.3)
project(robucar_base)


find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface
  roslaunch roslint  roscpp sensor_msgs control_toolbox)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS hardware_interface roscpp sensor_msgs control_toolbox
    DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#Disable debug
add_definitions(-DNDEBUG)

add_library(pure_client
  src/communication_p/pure_client.cpp
  src/communication_p/pure_devices.cpp
  src/communication_p/pure_interface.cpp
  src/communication_p/framepure.cpp)

add_executable(robucar_node
  src/robucar_base.cpp
  src/robucar_hardware.cpp)
target_link_libraries(robucar_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} pure_client)


#roslaunch_add_file_check(launch)

#install(TARGETS robucar_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES SOURCES ${EXTRA_FILES})
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
