#include "Vtl_sram_ctrl.h"
#include <iomanip>
#include <iostream>
#include <verilated.h>

#ifdef TRACE_ENABLED
#include <verilated_vcd_c.h>
#endif

//==============================================================================
// Configuration
//==============================================================================
#define MAX_CYCLES 5000
#define MEM_SIZE_BYTES (16 * 1024 * 1024)

#define OP_PUTFULL 0
#define OP_GET 4
#define OP_ACK 0
#define OP_ACKDATA 1

//==============================================================================
// Testbench Class
//==============================================================================
class TL_SRAM_Sanity {
private:
  Vtl_sram_ctrl *dut;
  uint64_t cycle;
  int errors;

#ifdef TRACE_ENABLED
  VerilatedVcdC *trace;
#endif

public:
  TL_SRAM_Sanity() {
    dut = new Vtl_sram_ctrl;
    cycle = 0;
    errors = 0;

#ifdef TRACE_ENABLED
    Verilated::traceEverOn(true);
    trace = new VerilatedVcdC;
    dut->trace(trace, 99);
    trace->open("sanity.vcd");
#endif
  }

  ~TL_SRAM_Sanity() {
#ifdef TRACE_ENABLED
    if (trace) {
      trace->close();
      delete trace;
    }
#endif
    delete dut;
  }

  //==========================================================================
  // Basic Infrastructure
  //==========================================================================

  void tick() {
    dut->clk_i = 0;
    dut->eval();
#ifdef TRACE_ENABLED
    if (trace)
      trace->dump(cycle * 10);
#endif

    dut->clk_i = 1;
    dut->eval();
#ifdef TRACE_ENABLED
    if (trace)
      trace->dump(cycle * 10 + 5);
#endif

    cycle++;

    if (cycle > MAX_CYCLES) {
      std::cerr << "[ERROR] Timeout at cycle " << cycle << std::endl;
      errors++;
      exit(1);
    }
  }

  void reset() {
    dut->rst_ni = 0;
    dut->tl_a_valid = 0;
    dut->tl_d_ready = 1;

    for (int i = 0; i < 10; i++)
      tick();

    dut->rst_ni = 1;
    tick();

    std::cout << "[INFO] Reset complete at cycle " << cycle << std::endl;
  }

  //==========================================================================
  // TileLink Helpers
  //==========================================================================

  void write_single(uint32_t addr, uint64_t data) {
    // Send request
    dut->tl_a_valid = 1;
    dut->tl_a_opcode = OP_PUTFULL;
    dut->tl_a_address = addr;
    dut->tl_a_data = data;
    dut->tl_a_mask = 0xFF;
    dut->tl_a_size = 3; // 8 bytes
    dut->tl_a_source = 0;
    dut->tl_a_param = 0;

    while (!dut->tl_a_ready)
      tick();
    tick();

    dut->tl_a_valid = 0;

    // Wait for response
    dut->tl_d_ready = 1;
    while (!dut->tl_d_valid)
      tick();

    if (dut->tl_d_opcode != OP_ACK) {
      std::cerr << "[ERROR] Write expected ACK, got " << (int)dut->tl_d_opcode
                << std::endl;
      errors++;
    }

    if (dut->tl_d_denied) {
      std::cerr << "[ERROR] Write denied at 0x" << std::hex << addr
                << std::endl;
      errors++;
    }

    tick();
  }

  uint64_t read_single(uint32_t addr) {
    // Send request
    dut->tl_a_valid = 1;
    dut->tl_a_opcode = OP_GET;
    dut->tl_a_address = addr;
    dut->tl_a_data = 0;
    dut->tl_a_mask = 0;
    dut->tl_a_size = 3; // 8 bytes
    dut->tl_a_source = 0;
    dut->tl_a_param = 0;

    while (!dut->tl_a_ready)
      tick();
    tick();

    dut->tl_a_valid = 0;

    // Wait for response
    dut->tl_d_ready = 1;
    while (!dut->tl_d_valid)
      tick();

    if (dut->tl_d_opcode != OP_ACKDATA) {
      std::cerr << "[ERROR] Read expected ACKDATA, got "
                << (int)dut->tl_d_opcode << std::endl;
      errors++;
    }

    if (dut->tl_d_corrupt) {
      std::cerr << "[WARN] Read returned corrupt flag at 0x" << std::hex << addr
                << std::endl;
    }

    uint64_t data = dut->tl_d_data;
    tick();

    return data;
  }

  //==========================================================================
  // Sanity Tests
  //==========================================================================

  void test_basic_rw() {
    std::cout << "\n[TEST] Basic Read/Write" << std::endl;

    uint32_t addr = 0x1000;
    uint64_t wdata = 0xDEADBEEFCAFEBABE;

    write_single(addr, wdata);
    uint64_t rdata = read_single(addr);

    if (rdata == wdata) {
      std::cout << "[PASS] Data matched: 0x" << std::hex << rdata << std::endl;
    } else {
      std::cerr << "[FAIL] Expected 0x" << std::hex << wdata << ", got 0x"
                << rdata << std::endl;
      errors++;
    }
  }

  void test_multiple_locations() {
    std::cout << "\n[TEST] Multiple Locations" << std::endl;

    uint32_t addrs[] = {0x0, 0x1000, 0x2000, 0x3000};
    uint64_t data[] = {0x1111111111111111, 0x2222222222222222,
                       0x3333333333333333, 0x4444444444444444};

    // Write all
    for (int i = 0; i < 4; i++) {
      write_single(addrs[i], data[i]);
    }

    // Read and verify all
    bool pass = true;
    for (int i = 0; i < 4; i++) {
      uint64_t rdata = read_single(addrs[i]);
      if (rdata != data[i]) {
        std::cerr << "[FAIL] Location " << i << " mismatch" << std::endl;
        errors++;
        pass = false;
      }
    }

    if (pass) {
      std::cout << "[PASS] All locations verified" << std::endl;
    }
  }

  void test_counters() {
    std::cout << "\n[TEST] Performance Counters" << std::endl;

    uint32_t r_before = dut->stat_read_cnt;
    uint32_t w_before = dut->stat_write_cnt;

    write_single(0x4000, 0xAAAAAAAAAAAAAAAA);
    read_single(0x4000);

    uint32_t r_after = dut->stat_read_cnt;
    uint32_t w_after = dut->stat_write_cnt;

    if ((r_after == r_before + 1) && (w_after == w_before + 1)) {
      std::cout << "[PASS] Counters incrementing correctly" << std::endl;
    } else {
      std::cerr << "[FAIL] Counter mismatch" << std::endl;
      errors++;
    }
  }

  //==========================================================================
  // Main Test Runner
  //==========================================================================

  void run() {
    std::cout << "\n================================================"
              << std::endl;
    std::cout << "  TileLink SRAM Controller - Sanity Tests" << std::endl;
    std::cout << "================================================\n"
              << std::endl;

    reset();

    test_basic_rw();
    test_multiple_locations();
    test_counters();

    std::cout << "\n================================================"
              << std::endl;
    std::cout << "  Summary" << std::endl;
    std::cout << "================================================"
              << std::endl;
    std::cout << "  Cycles: " << cycle << std::endl;
    std::cout << "  Errors: " << errors << std::endl;

    if (errors == 0) {
      std::cout << "  Status: \033[1;32mPASS\033[0m" << std::endl;
    } else {
      std::cout << "  Status: \033[1;31mFAIL\033[0m" << std::endl;
    }

    std::cout << "================================================\n"
              << std::endl;
  }

  int get_errors() { return errors; }
};

//==============================================================================
// Main Entry Point
//==============================================================================
int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);

  TL_SRAM_Sanity tb;
  tb.run();

  return (tb.get_errors() == 0) ? 0 : 1;
}
