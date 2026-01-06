# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(beacon_1)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/ble.h)
# target_sources(app PRIVATE src/sys_off.h)
target_sources(app PRIVATE src/ws2812.h)
if(CONFIG_APP_USE_RETAINED_MEM)
  target_sources(app PRIVATE src/retained.c)
endif()