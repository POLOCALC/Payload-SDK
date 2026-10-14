cmake_minimum_required(VERSION 3.5)
project(psdk_telemetry C)

# POLOCALC telemetry bridge: streams Matrice 400 flight-controller telemetry to lager.
# Reuses the Raspberry Pi sample's UART/I2C HAL and its configuration headers
# (raspberry_pi/hal/hal_uart.h: UART device; raspberry_pi/application/dji_sdk_app_info.h: credentials),
# so both programs always use the same settings.

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -std=gnu99 -Wall")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
add_definitions(-D_GNU_SOURCE)

if (NOT USE_SYSTEM_ARCH)
    add_definitions(-DSYSTEM_ARCH_LINUX)
endif ()

set(PACKAGE_NAME payloadsdk)

execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

set(PSDK_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../../..)
set(RPI_SAMPLE_DIR ${CMAKE_CURRENT_LIST_DIR}/../raspberry_pi)

file(GLOB APP_SRC application/*.c)
file(GLOB OSAL_SRC ../common/osal/*.c)
set(HAL_SRC
        ${RPI_SAMPLE_DIR}/hal/hal_uart.c
        ${RPI_SAMPLE_DIR}/hal/hal_i2c.c)

include_directories(
        application
        ../common
        ${RPI_SAMPLE_DIR}/hal
        ${RPI_SAMPLE_DIR}/application
        ${PSDK_ROOT}/psdk_lib/include)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

add_executable(${PROJECT_NAME} ${APP_SRC} ${OSAL_SRC} ${HAL_SRC})
target_link_libraries(${PROJECT_NAME}
        ${PSDK_ROOT}/psdk_lib/lib/${TOOLCHAIN_NAME}/lib${PACKAGE_NAME}.a
        m dl)
