#Generated by VisualGDB project wizard.
#Note: VisualGDB will automatically update this file when you add new sources to the project.

cmake_minimum_required(VERSION 2.7)
project(EmlidPub)
set(LIBRARIES_FROM_REFERENCES "")
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(jukovsky_ros REQUIRED)
include_directories( ${jukovsky_ros_INCLUDE_DIRS})

find_package(LinuxChrono REQUIRED)
include_directories( ${LinuxChrono_INCLUDE_DIRS})

file(GLOB source_cpp *.cpp)
add_executable(EmlidPub ${source_cpp})
target_link_libraries(EmlidPub ${roscpp_LIBRARIES} /usr/local/lib/libLinuxChrono_lib.a "${LIBRARIES_FROM_REFERENCES}")
