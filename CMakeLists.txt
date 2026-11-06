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
        user/task/calib_task.c
        user/task/calib_task.h
        user/task/ins_task.c
        user/module/remote/remote.c
        user/module/remote/remote.h
        user/peripheral/uart/user_uart.c
        user/peripheral/uart/user_uart.h
        user/task/remote_task.c
        user/task/judge_task.c
        user/task/judge_task.h
        user/peripheral/dwt/user_dwt.c
        user/peripheral/dwt/user_dwt.h
        user/module/motor/motor.c
        user/module/motor/motor.h
        user/peripheral/watchdog/watchdog.c
        user/peripheral/watchdog/watchdog.h
        user/task/watch_task.c
        user/task/watch_task.h
        user/module/buzzer/buzzer.c
        user/module/buzzer/buzzer.h
        user/module/led/board_led.c
        user/module/led/board_led.h
        user/task/text/motor_test_task.c
        user/task/text/motor_test_task.h
        user/peripheral/can/user_can.c
        user/peripheral/can/user_can.h
        user/task/chassis_task.c
        user/algorithm/ramp/ramp.c
        user/algorithm/ramp/ramp.h
        user/module/judge/judge.c
        user/module/judge/judge.h
        user/module/supercapacitor/super_capacitor.c
        user/module/supercapacitor/super_capacitor.h
        user/control/chassis/chassis_controller/chassis_controller.c
        user/control/chassis/chassis_controller/chassis_controller.h
        user/control/heater/heater.c
        user/control/heater/heater.h
        user/control/chassis/power_controller/power_controller.c
        user/control/chassis/power_controller/power_controller.h
        user/algorithm/rls/rls.c
        user/algorithm/rls/rls.h
        user/control/chassis/chassis_external/chassis.c
        user/control/chassis/chassis_external/chassis.h
        user/control/chassis/chassis_config.h
        user/control/chassis/chassis_port/chassis_port.c
        user/control/chassis/chassis_port/chassis_port.h
        user/task/power_task.c
        user/task/power_task.h
        user/task/remote_task.h
        user/task/ins_task.h
        user/task/chassis_task.h
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
        ${CMAKE_SOURCE_DIR}/user/task/calib_task.c
        ${CMAKE_SOURCE_DIR}/user/module/remote/remote.c
        ${CMAKE_SOURCE_DIR}/user/peripheral/uart/user_uart.c
        ${CMAKE_SOURCE_DIR}/user/peripheral/dwt/user_dwt.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/user
        ${CMAKE_SOURCE_DIR}/user/algorithm/calibration
        ${CMAKE_SOURCE_DIR}/user/algorithm/pid
        ${CMAKE_SOURCE_DIR}/user/algorithm/vqf
        ${CMAKE_SOURCE_DIR}/user/module/bmi088
        ${CMAKE_SOURCE_DIR}/user/module/motor
        ${CMAKE_SOURCE_DIR}/user/module/remote
        ${CMAKE_SOURCE_DIR}/user/peripheral/uart
        ${CMAKE_SOURCE_DIR}/user/peripheral/dwt
        ${CMAKE_SOURCE_DIR}/user/peripheral/watchdog
        ${CMAKE_SOURCE_DIR}/user/module/buzzer
        ${CMAKE_SOURCE_DIR}/user/module/led
        ${CMAKE_SOURCE_DIR}/user/peripheral/can
        ${CMAKE_SOURCE_DIR}/user/task
        ${CMAKE_SOURCE_DIR}/user/control/chassis
        ${CMAKE_SOURCE_DIR}/user/control/chassis/chassis_external
        ${CMAKE_SOURCE_DIR}/user/control/chassis/chassis_port
        ${CMAKE_SOURCE_DIR}/user/control/chassis/chassis_controller
        ${CMAKE_SOURCE_DIR}/user/control/chassis/power_controller
        ${CMAKE_SOURCE_DIR}/user/control/heater
        ${CMAKE_SOURCE_DIR}/user/algorithm/ramp
        ${CMAKE_SOURCE_DIR}/user/algorithm/rls
        ${CMAKE_SOURCE_DIR}/user/module/judge
        ${CMAKE_SOURCE_DIR}/user/module/supercapacitor
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
