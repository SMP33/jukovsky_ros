cmake_minimum_required(VERSION 2.7)
project(aruco_home_estimator)

set(LIBRARIES_FROM_REFERENCES "")
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(image_transport REQUIRED)
include_directories(${image_transport_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED )
message(${OpenCV_LIBRARIES})

find_package(GeoMath REQUIRED)
include_directories(${GeoMath_INCLUDE_DIRS})

find_package(jukovsky_ros REQUIRED)
include_directories( ${jukovsky_ros_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} aruco_home_estimator.cpp)

target_link_libraries(${PROJECT_NAME}  ${roscpp_LIBRARIES}  ${image_transport_LIBRARIES}  ${cv_bridge_LIBRARIES} /usr/local/lib/libGeoMath_lib.a ${OpenCV_LIBRARIES} "${LIBRARIES_FROM_REFERENCES}")

