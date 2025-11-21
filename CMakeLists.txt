cmake_minimum_required(VERSION 3.14)
project(tl_sram_main_mem)

find_package(verilator QUIET)
if(NOT verilator_FOUND)
    find_package(Verilator REQUIRED)
endif()

set(RTL_SOURCES
    rtl/tl_sram_ctrl.v
    rtl/tl_sram_storage.v
)

set(VERILATOR_COMMON_ARGS
    -Wall
    -Wno-PINCONNECTEMPTY
    -Wno-UNUSED
    --trace
    --cc
    -O3
    -CFLAGS "-std=c++14"
)

add_executable(run_sanity dv/verilator/tb_sanity.cpp)

verilate(run_sanity
    SOURCES ${RTL_SOURCES}
    TOP_MODULE tl_sram_ctrl
    PREFIX Vtl_sram_ctrl
    VERILATOR_ARGS ${VERILATOR_COMMON_ARGS}
)

enable_testing()
add_test(NAME 01_sanity COMMAND run_sanity)
