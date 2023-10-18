#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)
set(SHIELD ssd1306_128x32)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

zephyr_cc_option(-ffunction-sections -fdata-sections -gcc-sections)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)


# NORDIC SDK APP END
zephyr_library_include_directories(.)
