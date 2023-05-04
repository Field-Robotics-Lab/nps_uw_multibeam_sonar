cmake_minimum_required(VERSION 3.0.2)
project(nps_uw_multibeam_sonar)

## Make sure you have installed CUDA and include in paths
## LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/cuda-11.1/lib64
## PATH=$PATH:/usr/local/cuda-11.1/bin
set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA CXX)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++17)
endif()

find_package(catkin REQUIRED COMPONENTS
 tf
 cv_bridge
 gazebo_plugins
 marine_acoustic_msgs
 xacro)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SENSOR_ROS_PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  marine_acoustic_msgs
 )

## Plugins
add_library(nps_multibeam_sonar_ros_plugin
            src/gazebo_multibeam_sonar_raster_based.cpp
            src/sonar_calculation_cuda.cu
            src/MaterialSwitcher.cc  # From gazebo/rendering/selection_buffer
            src/SelectionBuffer.cc  # From gazebo/rendering/selection_buffer
            src/SelectionRenderListener.cc  # From gazebo/rendering/selection_buffer
  )
set_target_properties(nps_multibeam_sonar_ros_plugin
                      PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(nps_multibeam_sonar_ros_plugin
                      ${OGRE_LIBRARIES} ${catkin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${CUDA_CUFFT_LIBRARIES})
add_dependencies(nps_multibeam_sonar_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND SENSOR_ROS_PLUGINS_LIST nps_multibeam_sonar_ros_plugin)

add_library(nps_multibeam_sonar_ray_ros_plugin
            src/gazebo_multibeam_sonar_ray_based.cpp
            src/sonar_calculation_cuda.cu
            src/MaterialSwitcher.cc  # From gazebo/rendering/selection_buffer
            src/SelectionBuffer.cc  # From gazebo/rendering/selection_buffer
            src/SelectionRenderListener.cc  # From gazebo/rendering/selection_buffer
  )
set_target_properties(nps_multibeam_sonar_ray_ros_plugin
                      PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
                      ${OGRE_LIBRARIES} ${catkin_LIBRARIES}
                      ${CUDA_LIBRARIES} ${PCL_LIBRARIES}
                      ${CUDA_CUFFT_LIBRARIES})
add_dependencies(nps_multibeam_sonar_ray_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND SENSOR_ROS_PLUGINS_LIST nps_multibeam_sonar_ray_ros_plugin)


# Install plugins
install(
  TARGETS ${SENSOR_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# for launch
install(DIRECTORY launch worlds urdf models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
