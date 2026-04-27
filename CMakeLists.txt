# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hpadv2)

target_sources(app PRIVATE
	src/main.c
	src/radio_esb.c
	src/radio_identity.c
	src/usb_hid_consumer.c
	src/dongle_config.c
)
