cmake_minimum_required(VERSION 3.14)
project(tl_sram_main_mem)

find_package(Verilator FIXED)
if(NOT Verilator_FOUND)
    message(FATAL_ERROR "Verilator was not found. Please install it.")
endif()

set(RTL_SOURCES
    rtl/tl_sram_ctrl.v
    rtl/tl_sram_storage.v
)

set(TB_SOURCES
    dv/verilator/tb_top.cpp
)

add_executable(run_sim ${TB_SOURCES})

verilate(run_sim
    SOURCES ${RTL_SOURCES}
    TOP_MODULE tl_sram_ctrl
    PREFIX Vtop
    VERILATOR_ARGS
        -Wall
        -Wno-PINCONNECTEMPTY
        -Wno-UNUSED
        --trace
        --cc
        -O3
        -CFLAGS "-std=c++14"
)

enable_testing()
add_test(NAME basic_boot_test COMMAND run_sim)
