cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME sentry_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        user/module/bmi088/BMI088driver.c
        user/module/bmi088/BMI088driver.h
        user/module/bmi088/BMI088Middleware.c
        user/module/bmi088/BMI088Middleware.h
        user/module/bmi088/BMI088reg.h
        user/algorithm/vqf/vqf.c
        user/algorithm/vqf/vqf.h
        user/algorithm/calibration/calibration.c
        user/algorithm/calibration/calibration.h
        user/algorithm/calibration/calibration_session.c
        user/algorithm/calibration/calibration_session.h
        user/algorithm/calibration/bias_config.h
        user/algorithm/pid/pid.c
        user/algorithm/pid/pid.h
        user/calib_task.c
        user/calib_task.h
        user/ins_task.c
        user/module/remote/remote.c
        user/module/remote/remote.h
        user/peripheral/uart/user_uart.c
        user/peripheral/uart/user_uart.h
        user/remote_task.c
        user/peripheral/pwm/user_pwm.c
        user/peripheral/pwm/user_pwm.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/user/module/bmi088/BMI088driver.c
        ${CMAKE_SOURCE_DIR}/user/module/bmi088/BMI088Middleware.c
        ${CMAKE_SOURCE_DIR}/user/algorithm/vqf/vqf.c
        ${CMAKE_SOURCE_DIR}/user/algorithm/calibration/calibration.c
        ${CMAKE_SOURCE_DIR}/user/algorithm/calibration/calibration_session.c
        ${CMAKE_SOURCE_DIR}/user/algorithm/pid/pid.c
        ${CMAKE_SOURCE_DIR}/user/calib_task.c
        ${CMAKE_SOURCE_DIR}/user/module/remote/remote.c
        ${CMAKE_SOURCE_DIR}/user/peripheral/uart/user_uart.c
        ${CMAKE_SOURCE_DIR}/user/peripheral/pwm/user_pwm.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/user
        ${CMAKE_SOURCE_DIR}/user/algorithm/calibration
        ${CMAKE_SOURCE_DIR}/user/algorithm/pid
        ${CMAKE_SOURCE_DIR}/user/algorithm/vqf
        ${CMAKE_SOURCE_DIR}/user/module/bmi088
        ${CMAKE_SOURCE_DIR}/user/module/remote
        ${CMAKE_SOURCE_DIR}/user/peripheral/pwm
        ${CMAKE_SOURCE_DIR}/user/peripheral/uart
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
