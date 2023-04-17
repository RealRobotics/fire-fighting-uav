cmake_minimum_required(VERSION 3.0.2)
project(fire_detection)

find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "\n\n OpenMP is missing!\n\n")
endif()

find_package(realsense2 2.50.0)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  uav_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime uav_msgs
)

include_directories(
  include
  include/mlx90640-library/headers/
  ${catkin_INCLUDE_DIRS}
)


add_executable(fire_detector_node src/fire_detector_node.cpp src/fire_detector.cpp src/thermal_camera.cpp)
add_dependencies(fire_detector_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

if (OPENMP_FOUND)
  target_compile_options(fire_detector_node PRIVATE ${OpenMP_FLAGS})
  target_link_libraries(fire_detector_node ${OpenMP_LIBS})
endif()

target_link_libraries(fire_detector_node
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mlx90640-library/libMLX90640_API.so
  ${realsense2_LIBRARY}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS fire_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS
  scripts/fire_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
