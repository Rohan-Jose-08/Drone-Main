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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(lib/RF24/CMakeLists.txt)

project(BAE-Drone-Main C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(BAE-Drone-Main BAE-Drone-Main.cpp pid.cpp mixer.cpp motors.cpp attitude.cpp filters.cpp altitude.cpp baro.cpp gps.cpp compass.cpp position.cpp bno.cpp bno055.c)

pico_set_program_name(BAE-Drone-Main "BAE-Drone-Main")
pico_set_program_version(BAE-Drone-Main "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(BAE-Drone-Main 0)
pico_enable_stdio_usb(BAE-Drone-Main 1)

# Drop USB serial output after 1 ms instead of waiting up to 500 ms, so a stalled
# serial monitor can't block the control loop long enough to trip the watchdog
target_compile_definitions(BAE-Drone-Main PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)

# Add the standard library to the build
target_link_libraries(BAE-Drone-Main pico_stdlib hardware_spi hardware_i2c hardware_pwm hardware_uart RF24)


target_include_directories(BAE-Drone-Main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}

)

pico_add_extra_outputs(BAE-Drone-Main)
