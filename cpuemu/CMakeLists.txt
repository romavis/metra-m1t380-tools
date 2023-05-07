cmake_minimum_required(VERSION 3.16)
project(M1T380EMU CXX)

set(default_build_type RelWithDebInfo)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_executable(
    m1t380emu
    m1t380emu.cpp
)
set_property(TARGET m1t380emu PROPERTY CXX_STANDARD 17)
target_include_directories(m1t380emu PRIVATE "include/")
target_link_libraries(m1t380emu fmt::fmt spdlog::spdlog)
