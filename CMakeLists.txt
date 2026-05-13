cmake_minimum_required(VERSION 3.20.0)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/prj_debug.conf CACHE STRING "Extra Kconfig fragments" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nrf52840-sd-card+rtc-test)

target_sources(app PRIVATE
	src/main.c
	src/ble.c
	src/ble_sensor.c
	src/data_logger.c
)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_include_directories(app PRIVATE inc)
