cmake_minimum_required(VERSION 2.8)

project(glochidia_counter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

IF(UNIX)
  # add some standard warnings
  ADD_DEFINITIONS(-Wno-variadic-macros -Wno-long-long -Wall -Wextra -Winit-self -Woverloaded-virtual -Wsign-promo -Wno-unused-parameter -pedantic -Woverloaded-virtual -Wno-unknown-pragmas)

  # -ansi does not compile with sjn module
  #ADD_DEFINITIONS(-ansi)

  # if you like to have warinings about conversions, e.g. double->int or double->float etc., or float compare
  #ADD_DEFINITIONS(-Wconversion -Wfloat-equal)
endif(UNIX)

find_package(OpenCV REQUIRED)

if(${OpenCV_VERSION} VERSION_LESS 2.3.1)
  message (FATAL_ERROR "OpenCV version is not compatible: ${OpenCV_VERSION}")
endif()

file(GLOB demo Demo.cpp)


file(GLOB_RECURSE analysis package_analysis/*.cpp)
file(GLOB_RECURSE bgs package_bgs/*.cpp package_bgs/*.c)
file(GLOB_RECURSE bgs_include package_bgs/*.h)
file(GLOB_RECURSE tracking package_tracking/*.cpp package_tracking/*.c)
file(GLOB_RECURSE tracking_include package_tracking/*.h)

include_directories(${CMAKE_SOURCE_DIR})

add_library(glochidia_counter STATIC ${tracking} ${bgs} ${analysis})
target_link_libraries(glochidia_counter ${OpenCV_LIBS})
set_property(TARGET glochidia_counter PROPERTY PUBLIC_HEADER ${bgs_include} ${tracking_include})

add_executable(glochidia_counter_bin ${demo})
target_link_libraries(glochidia_counter_bin ${OpenCV_LIBS} glochidia_counter)
set_target_properties(glochidia_counter_bin
  PROPERTIES OUTPUT_NAME glochidia_counter)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
set(BUILD_SHARED_LIBRARIES OFF)
set(CMAKE_EXE_LINKER_FLAGS "-static")
add_executable(glochidia_counter_demo ${demo})
target_link_libraries(glochidia_counter_demo ${OpenCV_LIBS} glochidia_counter)

#INSTALL(TARGETS glochidia_counter
#   glochidia_counter_demo
#   glochidia_counter_bin
#  RUNTIME DESTINATION bin COMPONENT app
#  LIBRARY DESTINATION lib COMPONENT runtime
#  ARCHIVE DESTINATION lib COMPONENT runtime
#  PUBLIC_HEADER DESTINATION include/package_bgs COMPONENT dev
#  FRAMEWORK DESTINATION "/Library/Frameworks"
