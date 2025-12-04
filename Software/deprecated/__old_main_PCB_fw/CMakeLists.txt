# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(main_PCB_fw C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(main_PCB_fw main_PCB_fw.cpp )

pico_set_program_name(main_PCB_fw "main_PCB_fw")
pico_set_program_version(main_PCB_fw "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main_PCB_fw 0)
pico_enable_stdio_usb(main_PCB_fw 1)

target_sources(main_PCB_fw 
        PUBLIC
        lib/IMU_lib/json.hpp
        lib/IMU_lib/ICM42688.hpp
        lib/IMU_lib/ICM42688.cpp
    )

# Add the standard library to the build
target_link_libraries(main_PCB_fw
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(main_PCB_fw PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

include(lib/RF24/CMakeLists.txt)

# Add any user requested libraries
target_link_libraries(main_PCB_fw 
        hardware_spi
        hardware_i2c
        hardware_pwm
        RF24
        )

pico_add_extra_outputs(main_PCB_fw)

