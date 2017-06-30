cmake_minimum_required (VERSION 3.1)
project (quat)
set (CMAKE_CXX_STANDARD 11)

if(MSVC)
    add_definitions(-D_SCL_SECURE_NO_WARNINGS)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${CMAKE_SOURCE_DIR})

add_executable (1-2-rotazioni "${CMAKE_SOURCE_DIR}/test/1-2-rotazioni")
add_executable (3-inertial2d "${CMAKE_SOURCE_DIR}/test/3-inertial2d")
add_executable (4-inertial3d "${CMAKE_SOURCE_DIR}/test/4-inertial3d")

add_library(quat ${CMAKE_SOURCE_DIR}/src/quat.cpp)

target_link_libraries(1-2-rotazioni quat) 
target_link_libraries(3-inertial2d quat) 
target_link_libraries(4-inertial3d quat) 

install (TARGETS 1-2-rotazioni 3-inertial2d 4-inertial3d DESTINATION ${CMAKE_SOURCE_DIR}/bin/)
