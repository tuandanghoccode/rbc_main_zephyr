cmake_minimum_required(VERSION 3.20)

# Tim custom board definition trong thu muc boards/ cua project
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rbc_r1_blinky)

target_sources(app PRIVATE
    src/main.c
    src/bno055.c
    src/bno055_zephyr.c
    src/pca9685_zephyr.c
)

# Include src/ de bno055.c tim thay bno055.h va bno055_zephyr.h
target_include_directories(app PRIVATE src)