cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(SBUS
	VERSION 1.0
	DESCRIPTION "SBUS encoder and decoder"
	LANGUAGES C CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
  # Fetch core
  include(FetchContent)
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
		GIT_TAG					v0.8.0
	)
	FetchContent_MakeAvailable(core)
	# Add the library target
	add_library(sbus
		src/sbus/sbus.cc
		include/sbus/sbus.h
	)
	# Link libraries
	target_link_libraries(sbus
		PUBLIC
			core
	)
	# Setup include directories 
	target_include_directories(sbus PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
endif()

