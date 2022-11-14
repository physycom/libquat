cmake_minimum_required (VERSION 3.1)

if(DEFINED ENV{VCPKG_DEFAULT_TRIPLET})
  message(STATUS "Setting default vcpkg target triplet to $ENV{VCPKG_DEFAULT_TRIPLET}")
  set(VCPKG_TARGET_TRIPLET $ENV{VCPKG_DEFAULT_TRIPLET})
endif()

if(ENABLE_TEST)
  list(APPEND VCPKG_MANIFEST_FEATURES "test")
endif()

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
  message(STATUS "VCPKG found: $ENV{VCPKG_ROOT}")
  message(STATUS "Using VCPKG integration")
  message(STATUS "VCPKG_MANIFEST_FEATURES: ${VCPKG_MANIFEST_FEATURES}")
elseif(DEFINED CMAKE_TOOLCHAIN_FILE)
  message(STATUS "Using toolchain: ${CMAKE_TOOLCHAIN_FILE}")
  if(CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg.cmake")
    message(STATUS "Toolchain uses VCPKG integration")
    message(STATUS "VCPKG_MANIFEST_FEATURES: ${VCPKG_MANIFEST_FEATURES}")
  endif()
else()
  message(WARNING "vcpkg not found, toolchain not defined, build might fail")
endif()

project (quat)
set (CMAKE_CXX_STANDARD 11)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Install prefix" FORCE)
endif()

set(INSTALL_BIN_DIR      "bin"                     CACHE PATH "Path where exe and dll will be installed")
set(INSTALL_LIB_DIR      "lib"                     CACHE PATH "Path where lib will be installed")
set(INSTALL_INCLUDE_DIR  "include"                 CACHE PATH "Path where headers will be installed")
set(INSTALL_CMAKE_DIR    "share/${PROJECT_NAME}"   CACHE PATH "Path where cmake configs will be installed")

add_library(quat ${CMAKE_CURRENT_LIST_DIR}/src/quat.h ${CMAKE_CURRENT_LIST_DIR}/src/quat.cpp)
target_include_directories(quat PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src> $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)
set_target_properties(quat PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_LIST_DIR}/src/quat.h")

install(TARGETS quat EXPORT QuatTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
  COMPONENT dev
)

if(ENABLE_TEST)
  find_package(Boost REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})
  add_executable (1-2-rotazioni "${CMAKE_CURRENT_LIST_DIR}/test/1-2-rotazioni.cpp")
  add_executable (3-inertial2d "${CMAKE_CURRENT_LIST_DIR}/test/3-inertial2d.cpp")
  add_executable (4-inertial3d "${CMAKE_CURRENT_LIST_DIR}/test/4-inertial3d.cpp")
  target_link_libraries(1-2-rotazioni quat)
  target_link_libraries(3-inertial2d quat)
  target_link_libraries(4-inertial3d quat)
  target_link_libraries(1-2-rotazioni Boost::boost)
  target_link_libraries(3-inertial2d Boost::boost)
  target_link_libraries(4-inertial3d Boost::boost)
  if(MSVC)
    target_compile_definitions(1-2-rotazioni PRIVATE _USE_MATH_DEFINES)
    target_compile_definitions(1-2-rotazioni PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_definitions(3-inertial2d PRIVATE _USE_MATH_DEFINES)
    target_compile_definitions(3-inertial2d PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_definitions(4-inertial3d PRIVATE _USE_MATH_DEFINES)
    target_compile_definitions(4-inertial3d PRIVATE _CRT_SECURE_NO_WARNINGS)
  endif()
  install (TARGETS 1-2-rotazioni 3-inertial2d 4-inertial3d DESTINATION "${INSTALL_BIN_DIR}")
endif()

install(EXPORT QuatTargets
  FILE QuatConfig.cmake
  NAMESPACE Quat::
  DESTINATION "${INSTALL_CMAKE_DIR}"
)
