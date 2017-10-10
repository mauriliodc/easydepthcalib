cmake_minimum_required(VERSION 2.8.3)
project(easydepthcalibration)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs cv_bridge image_transport nodelet)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
SET(CMAKE_C_FLAGS "-O3")

FIND_PACKAGE(OpenCV REQUIRED)
find_package(PCL)
find_package(Boost)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} /usr/include/eigen3 ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src/types/
${PROJECT_SOURCE_DIR}/src/calibration/)

catkin_package()

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(depthDumper_node src/dumper/depthDumper.cpp)
target_link_libraries(depthDumper_node ${catkin_LIBRARIES} ${OpenCV_LIBS} )


#CALIBRATION UTILS
add_library(calibration SHARED    src/calibration/calibration.h src/calibration/calibration.cpp
                                  src/calibration/calibrationMatrix.h src/calibration/calibrationMatrix.cpp)
target_link_libraries(calibration ${PCL_LIBRARIES} ${OpenCV_LIBS})


#REAL TIME DRIVER NODE
add_executable(driver_node src/driver/main_driver_node.cpp src/driver/driver.cpp src/driver/driver.h)
target_link_libraries(driver_node                   calibration
                                                    ${PCL_LIBRARIES}
                                                    ${OpenCV_LIBS}
                                                    ${catkin_LIBRARIES})

#REAL TIME DRIVER NODELET
add_library(DriverNodelet src/driver/driver.cpp src/driver/driver.h src/driver/driver_nodelet.cpp src/driver/driver_nodelet.h)
target_link_libraries(DriverNodelet                 calibration
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
