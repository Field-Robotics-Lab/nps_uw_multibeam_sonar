cmake_minimum_required(VERSION 3.0.2)
project(gpgpu)

## Make sure you have installed CUDA and include in paths
## LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/cuda-11.1/lib64
## PATH=$PATH:/usr/local/cuda-11.1/bin
#set(CMAKE_CUDA_COMPILER "nvcc")
#enable_language(CUDA CXX)

# if(NOT DEFINED CMAKE_CUDA_STANDARD)
#     set(CMAKE_CUDA_STANDARD 17)
#     set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# endif()

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
 gpgpu_cuda
 )

#find_package(gpgpu_cuda REQUIRED)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)


#find_package(CUDA REQUIRED)
#include_directories(${CUDA_INCLUDE_DIRS})
#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")
# set(CMAKE_CUDA_ARCHITECTURE all)
# set(CUDA_FORCE_PTX_JIT=1)

# set(CUDA_NVCC_FLAGS "-rdc=false" CACHE STRING "nvcc flags" FORCE)
# set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

# set(CUDA_VERBOSE_BUILD ON)
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --cudart shared")

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS}
    /tmp/gpgpu-sim/gpgpu-sim_distribution/lib/gcc-7.5.0/cuda-11010/release
    )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SENSOR_ROS_PLUGINS_LIST "")


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  gpgpu_cuda
 )

## Plugins
# Test


# set(CUDA_NVCC_FLAGS
#     -DCUDA_PTX
# )
# if(MSVC)
#     list(APPEND CUDA_NVCC_FLAGS
#         -D_WINDOWS
#     )
# endif(MSVC)

# cuda_compile_ptx(ptx_objects
# src/hello.cu)

# CUDA_GET_SOURCES_AND_OPTIONS(cmake_options options ${ARGN})
# CUDA_WRAP_SRCS(gpgpu_test PTX generated_files ${source_files} ${cmake_options}
# OPTIONS ${options})

# CUDA_WRAP_SRCS(gpgpu_test PTX generated_files
# src/hello.cu)
# add_library(myptx OBJECT src/hello.cu)
# set_property(TARGET myptx PROPERTY CUDA_PTX_COMPILATION ON)

#add_library(
#    gpgpu_test_lib
#    src/sonar_test.cu
#)
#target_link_libraries(gpgpu_test_lib "libcudart.so")
#target_link_libraries(gpgpu_test_lib
#                      ${OGRE_LIBRARIES} ${catkin_LIBRARIES}
#                      ${CUDA_LIBRARIES}
#                      ${CUDA_CUFFT_LIBRARIES})
#set_target_properties(gpgpu_test_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(
    gpgpu_test
    src/sonar_wrapper.cpp)
target_link_libraries(gpgpu_test gpgpu_test_lib)
target_link_libraries(gpgpu_test
                      ${OGRE_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gpgpu_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpgpu_test "libcudart.so")


# set_target_properties(gpgpu_test PROPERTIES CUDA_ARCHITECTURES 60)
# target_compile_definitions(gpgpu_test PRIVATE
#     CUDA_PTX)
# target_compile_options(gpgpu_test PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
#                         -rdc=false;
#                        >)
# target_link_libraries(gpgpu_test_ptx "libcudart.so")
# set_target_properties(gpgpu_test_ptx PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
# set_target_properties(gpgpu_test_ptx PROPERTIES CUDA_ARCHITECTURES 60)

list(APPEND SENSOR_ROS_PLUGINS_LIST gpgpu_test)

# Install plugins
install(
  TARGETS ${SENSOR_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
