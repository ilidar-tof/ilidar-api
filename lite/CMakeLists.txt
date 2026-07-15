# CMake 3.13+ is required for the documented -S/-B build workflow.
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(itfs_lite_examples_cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(SRC_DIR "${SDK_ROOT}/src")
set(OUT_DIR "${CMAKE_CURRENT_LIST_DIR}/bin")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")

set(LITE_API_SRC
  "${SRC_DIR}/ilidar_lite.cpp"
)

add_executable(lite_helloworld_cmake lite_helloworld.cpp ${LITE_API_SRC})
add_executable(lite_opencv_example_cmake lite_opencv_example.cpp ${LITE_API_SRC})
add_executable(lite_pcl_example_cmake lite_pcl_example.cpp ${LITE_API_SRC})

set_target_properties(lite_helloworld_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")
set_target_properties(lite_opencv_example_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")
set_target_properties(lite_pcl_example_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")

include_directories("${SRC_DIR}")

if(WIN32)
  set(OpenCV_DIR "C:/opencv/build/x64/vc15/lib" CACHE PATH "OpenCV CMake package directory")
else()
  set(CMAKE_THREAD_LIBS_INIT "-lpthread")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  set(CMAKE_HAVE_THREADS_LIBRARY 1)
  set(CMAKE_USE_WIN32_THREADS_INIT 0)
  set(CMAKE_USE_PTHREADS_INIT 1)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
endif()

find_package(OpenCV 4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
target_link_libraries(lite_opencv_example_cmake ${OpenCV_LIBS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(lite_pcl_example_cmake ${PCL_LIBRARIES})

if(WIN32)
  target_link_libraries(lite_helloworld_cmake ws2_32 iphlpapi)
  target_link_libraries(lite_opencv_example_cmake ws2_32 iphlpapi)
  target_link_libraries(lite_pcl_example_cmake ws2_32 iphlpapi)
endif()
