#include "Vtl_sram_ctrl.h"
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>

vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

void tick(Vtl_sram_ctrl *top, VerilatedVcdC *tfp) {
  top->clk_i = 1;
  top->eval();
  tfp->dump(main_time++);
  top->clk_i = 0;
  top->eval();
  tfp->dump(main_time++);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Verilated::traceEverOn(true);

  Vtl_sram_ctrl *top = new Vtl_sram_ctrl;
  VerilatedVcdC *tfp = new VerilatedVcdC;

  top->trace(tfp, 99);
  tfp->open("sanity.vcd");

  top->rst_ni = 0;
  top->tl_d_ready = 1;
  for (int i = 0; i < 10; i++)
    tick(top, tfp);
  top->rst_ni = 1;
  for (int i = 0; i < 5; i++)
    tick(top, tfp);

  // Write 0xDEADBEEF to 0x100
  printf("[INFO] Write...\n");
  top->tl_a_valid = 1;
  top->tl_a_opcode = 1; // PutFull
  top->tl_a_size = 3;   // 8 bytes
  top->tl_a_source = 1;
  top->tl_a_address = 0x100;
  top->tl_a_data = 0xDEADBEEF;
  top->tl_a_mask = 0xFF;

  while (!top->tl_a_ready)
    tick(top, tfp);
  tick(top, tfp);
  top->tl_a_valid = 0;

  while (!top->tl_d_valid)
    tick(top, tfp);
  tick(top, tfp); // Consume Ack

  // Read 0x100
  printf("[INFO] Read...\n");
  top->tl_a_valid = 1;
  top->tl_a_opcode = 0; // Get
  top->tl_a_size = 3;
  top->tl_a_source = 2;
  top->tl_a_address = 0x100;

  while (!top->tl_a_ready)
    tick(top, tfp);
  tick(top, tfp);
  top->tl_a_valid = 0;

  while (!top->tl_d_valid)
    tick(top, tfp);

  if (top->tl_d_data == 0xDEADBEEF) {
    printf("[PASS] Sanity Check Passed.\n");
  } else {
    printf("[FAIL] Sanity Check Failed. Got 0x%lx\n", top->tl_d_data);
  }

  top->final();
  tfp->close();
  delete top;
  delete tfp;
  return 0;
}
