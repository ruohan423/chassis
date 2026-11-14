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
        user/task/ins_task/calib_task.c
        user/task/ins_task/calib_task.h
        user/task/ins_task/ins_task.c
        user/module/remote/remote.c
        user/module/remote/remote.h
        user/control/uart/user_uart.c
        user/control/uart/user_uart.h
        user/task/remote_task/remote_task.c
        user/task/judge_task/judge_task.c
        user/task/judge_task/judge_task.h
        user/module/dwt/user_dwt.c
        user/module/dwt/user_dwt.h
        user/module/motor/motor.c
        user/module/motor/motor.h
        user/module/watchdog/watchdog.c
        user/module/watchdog/watchdog.h
        user/task/watch_task/watch_task.c
        user/task/watch_task/watch_task.h
        user/module/buzzer/buzzer.c
        user/module/buzzer/buzzer.h
        user/module/led/board_led.c
        user/module/led/board_led.h
        user/test/motor_test_task.c
        user/test/motor_test_task.h
        user/test/stack_test.c
        user/test/stack_test.h
        user/control/can/user_can.c
        user/control/can/user_can.h
        user/task/chassis_task/chassis_task.c
        user/algorithm/ramp/ramp.c
        user/algorithm/ramp/ramp.h
        user/module/judge/judge.c
        user/module/judge/judge.h
        user/module/supercapacitor/super_capacitor.c
        user/module/supercapacitor/super_capacitor.h
        user/control/heater/heater.c
        user/control/heater/heater.h
        user/algorithm/rls/rls.c
        user/algorithm/rls/rls.h
        user/algorithm/qcp/qcp_solver.c
        user/algorithm/qcp/qcp_solver.h
        user/control/chassis/chassis.c
        user/control/chassis/chassis.h
        user/control/chassis/chassis_config.h
        user/control/chassis/chassis_port.c
        user/control/chassis/chassis_port.h
        user/task/remote_task/remote_task.h
        user/task/ins_task/ins_task.h
        user/task/chassis_task/chassis_task.h
        user/control/commun/usb_commun.c
        user/control/commun/usb_commun.h
        user/control/commun/can_commun.c
        user/control/commun/can_commun.h
        user/task/commun_task/usbcom_task.c
        user/task/commun_task/usbcom_task.h
        user/task/commun_task/cancom_task.c
        user/task/commun_task/cancom_task.h
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
        ${CMAKE_SOURCE_DIR}/user/task/ins_task/calib_task.c
        ${CMAKE_SOURCE_DIR}/user/module/remote/remote.c
        ${CMAKE_SOURCE_DIR}/user/control/uart/user_uart.c
        ${CMAKE_SOURCE_DIR}/user/module/dwt/user_dwt.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/user
        ${CMAKE_SOURCE_DIR}/user/control/commun
        ${CMAKE_SOURCE_DIR}/user/task/commun_task
        ${CMAKE_SOURCE_DIR}/user/algorithm/calibration
        ${CMAKE_SOURCE_DIR}/user/algorithm/pid
        ${CMAKE_SOURCE_DIR}/user/algorithm/vqf
        ${CMAKE_SOURCE_DIR}/user/module/bmi088
        ${CMAKE_SOURCE_DIR}/user/module/motor
        ${CMAKE_SOURCE_DIR}/user/module/remote
        ${CMAKE_SOURCE_DIR}/user/control/uart
        ${CMAKE_SOURCE_DIR}/user/module/dwt
        ${CMAKE_SOURCE_DIR}/user/module/watchdog
        ${CMAKE_SOURCE_DIR}/user/module/buzzer
        ${CMAKE_SOURCE_DIR}/user/module/led
        ${CMAKE_SOURCE_DIR}/user/control/can
        ${CMAKE_SOURCE_DIR}/user/task
        ${CMAKE_SOURCE_DIR}/user/task/chassis_task
        ${CMAKE_SOURCE_DIR}/user/task/ins_task
        ${CMAKE_SOURCE_DIR}/user/task/judge_task
        ${CMAKE_SOURCE_DIR}/user/task/remote_task
        ${CMAKE_SOURCE_DIR}/user/task/watch_task
        ${CMAKE_SOURCE_DIR}/user/task/test
        ${CMAKE_SOURCE_DIR}/user/control/chassis
        ${CMAKE_SOURCE_DIR}/user/control/heater
        ${CMAKE_SOURCE_DIR}/user/algorithm/ramp
        ${CMAKE_SOURCE_DIR}/user/algorithm/rls
        ${CMAKE_SOURCE_DIR}/user/algorithm/qcp
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
