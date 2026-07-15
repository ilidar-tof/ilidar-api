# CMake 3.13+ is required for the documented -S/-B build workflow.
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(itfs_examples_cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(SRC_DIR "${SDK_ROOT}/src")
set(OUT_DIR "${CMAKE_CURRENT_LIST_DIR}/bin")
file(MAKE_DIRECTORY "${OUT_DIR}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")

set(ITFS_API_SRC
  "${SRC_DIR}/ilidar.cpp"
)

add_executable(itfs_helloworld_cmake helloworld.cpp ${ITFS_API_SRC})
add_executable(itfs_opencv_example_cmake opencv_example.cpp ${ITFS_API_SRC})
add_executable(itfs_pcl_example_cmake pcl_example.cpp ${ITFS_API_SRC})

set_target_properties(itfs_helloworld_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")
set_target_properties(itfs_opencv_example_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")
set_target_properties(itfs_pcl_example_cmake PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUT_DIR}")

foreach(CALIBRATION_FILE iTFS-110.dat iTFS-80.dat)
  add_custom_command(TARGET itfs_pcl_example_cmake POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${SDK_ROOT}/itfs_pywrapper/${CALIBRATION_FILE}"
      "$<TARGET_FILE_DIR:itfs_pcl_example_cmake>/${CALIBRATION_FILE}"
    COMMENT "Copying ${CALIBRATION_FILE} beside the PCL example"
  )
endforeach()

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
target_link_libraries(itfs_opencv_example_cmake ${OpenCV_LIBS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(itfs_pcl_example_cmake ${PCL_LIBRARIES})

if(WIN32)
  target_link_libraries(itfs_helloworld_cmake ws2_32)
  target_link_libraries(itfs_opencv_example_cmake ws2_32)
  target_link_libraries(itfs_pcl_example_cmake ws2_32)
endif()
