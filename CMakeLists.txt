cmake_minimum_required(VERSION 3.20.0)
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
