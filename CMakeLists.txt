cmake_minimum_required(VERSION 2.8.3)
project(easydepthcalibration)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs cv_bridge image_transport nodelet)

# Fix for: /usr/bin/ld: cannot find -lopencv_dep_cudart
# https://devtalk.nvidia.com/default/topic/970325/cmake-cannot-find-lopencv_dep_cudart/
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
SET(CMAKE_C_FLAGS "-O3")

# Find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message("-- OpenCV ${OpenCV_VERSION} was found")
    include_directories(${OpenCV_INCLUDE_DIRS})
else(OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV was not found")
endif(OpenCV_FOUND)

# PCL
find_package(PCL REQUIRED)
if(${PCL_FOUND})
    message("-- PCL ${PCL_VERSION} was found")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else(${PCL_FOUND})
    message(FATAL_ERROR "PCL was not found")
endif(${PCL_FOUND})

# Find Boost
find_package(Boost 1.54 REQUIRED system chrono)
if (Boost_FOUND)
    message("-- Boost ${Boost_VERSION} was found")
    include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
    message(FATAL_ERROR "Boost 1.54 was not found")
endif(Boost_FOUND)

# Catkin package
catkin_package()

# Local includes
include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/types/ ${PROJECT_SOURCE_DIR}/src/calibration/)

# Depth dumper node
add_executable(depthDumper_node src/dumper/depthDumper.cpp)
target_link_libraries(depthDumper_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


#CALIBRATION UTILS
add_library(calibration SHARED    src/calibration/calibration.h src/calibration/calibration.cpp
                                  src/calibration/calibrationMatrix.h src/calibration/calibrationMatrix.cpp)
target_link_libraries(calibration ${PCL_LIBRARIES} ${OpenCV_LIBS})


#REAL TIME DRIVER NODELET
add_library(driver_nodelet src/driver/driver_nodelet.cpp src/driver/driver_nodelet.h)
target_link_libraries(driver_nodelet                calibration
                                                    ${PCL_LIBRARIES}
                                                    ${OpenCV_LIBS}
                                                    ${catkin_LIBRARIES})

#REAL TIME DRIVER NODE
add_executable(driver_node src/driver/driver_node.cpp)
target_link_libraries(driver_node                   calibration
                                                    ${PCL_LIBRARIES}
                                                    ${OpenCV_LIBS}
                                                    ${catkin_LIBRARIES})

#Offline calibration
add_executable(offlineCalibration	src/offlineCalibration/offlineCalibration.cpp)
target_link_libraries(offlineCalibration types calibration ${PCL_LIBRARIES} ${OpenCV_LIBS})

#Batch calibration
add_executable(batchCalibration	src/offlineCalibration/batchCalibration.cpp)
target_link_libraries(batchCalibration types calibration ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TYPES SHARED LIBRARY
add_library(types SHARED    
src/types/planeCloud.h 		src/types/planeCloud.cpp
src/types/pointCloud.h 		src/types/pointCloud.cpp
src/types/laserData.h 		src/types/laserData.cpp)

target_link_libraries(types ${PCL_LIBRARIES} ${OpenCV_LIBS})
