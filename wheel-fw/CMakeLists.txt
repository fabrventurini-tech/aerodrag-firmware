# AeroDrag Wheel — Zephyr application (skeleton)
# NB: firmware SEPARATO (nRF52840). NON fa parte del build ESP-IDF della repo madre.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aerodrag_wheel)

target_sources(app PRIVATE src/main.c)
