cmake_minimum_required(VERSION 3.1)
project(beez80)

option(BUILD_Z80_TESTS "Enables the BeeZ80 test suite." OFF)

if (WIN32)
	message(STATUS "Operating system is Windows.")
elseif(UNIX AND NOT APPLE)
	message(STATUS "Operating system is Linux.")
elseif(APPLE)
	message(STATUS "Operating system is Mac.")
else()
	message(SEND_ERROR "Operating system not supported.")
	return()
endif()

set(BEEZ80_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(BEEZ80_TEST_SOURCES
	beez80-tests.cpp)

set(BEEZ80_HEADER
	beez80.h)

set(BEEZ80_SOURCE
	beez80.cpp)

if (WIN32)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-format-security")
		message(STATUS "Compiler is MinGW.")
	else()
		message(SEND_ERROR "Compiler not supported.")
		return()
	endif()
elseif (UNIX AND NOT APPLE)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-format-security")
		message(STATUS "Compiler is GCC.")
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-format-security")
		message(STATUS "Compiler is Clang.")
	else()
		message(SEND_ERROR "Compiler not supported.")
		return()
	endif()
elseif (APPLE)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-format-security")
		message(STATUS "Compiler is AppleClang.")
	else()
		message(SEND_ERROR "Compiler not supported.")
		return()
	endif()
endif()

add_library(beez80 ${BEEZ80_SOURCE} ${BEEZ80_HEADER})
target_include_directories(beez80 PUBLIC ${BEEZ80_INCLUDE_DIR})

add_library(libbeez80 ALIAS beez80)

if (BUILD_Z80_TESTS STREQUAL "ON")
    project(beez80-tests)
    add_executable(beez80-tests ${BEEZ80_TEST_SOURCES})
    include_directories(beez80-tests ${BEEZ80_INCLUDE_DIR})
    target_link_libraries(beez80-tests libbeez80)
endif()