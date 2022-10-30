# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.1.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(Sbus
    VERSION 8.1.4
    DESCRIPTION "SBUS encoder and decoder"
    LANGUAGES C CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.1.3
  )
  FetchContent_MakeAvailable(core)
  # Add the library target
  add_library(sbus
    src/sbus.cpp
    src/sbus.h
  )
  # Link libraries
  target_link_libraries(sbus
    PUBLIC
      core 
  )
  # Setup include directories 
  target_include_directories(sbus PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the example target
    add_executable(sbus_example examples/cmake/sbus_example.cc)
    # Add the includes
    target_include_directories(sbus_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(sbus_example
      PRIVATE 
        sbus
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(sbus_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
