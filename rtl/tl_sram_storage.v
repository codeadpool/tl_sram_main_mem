`ifndef TL_SRAM_STORAGE_V
`define TL_SRAM_STORAGE_V

module tl_sram_storage #(
    parameter DATA_WIDTH = 64,
    parameter MEM_SIZE_BYTES = 64 * 1024 * 1024,
    parameter INIT_FILE = ""
) (
    input  wire                  clk,
    input  wire                  we_i,
    input  wire [          31:0] addr_i,
    input  wire [DATA_WIDTH-1:0] wdata_i,
    input  wire [           7:0] wmask_i,
    output reg  [DATA_WIDTH-1:0] rdata_o
);

`ifdef SYNTHESIS
  // -----------------------------------------------------
  // SYNTHESIS: Block RAM Inference
  // -----------------------------------------------------
  localparam DEPTH = MEM_SIZE_BYTES / (DATA_WIDTH / 8);
  (* ram_style = "block" *)
  reg [DATA_WIDTH-1:0] mem_array[0:DEPTH-1];

  always @(posedge clk) begin
    if (we_i) begin
      if (wmask_i[0]) mem_array[addr_i][7:0] <= wdata_i[7:0];
      if (wmask_i[1]) mem_array[addr_i][15:8] <= wdata_i[15:8];
      if (wmask_i[2]) mem_array[addr_i][23:16] <= wdata_i[23:16];
      if (wmask_i[3]) mem_array[addr_i][31:24] <= wdata_i[31:24];
      if (wmask_i[4]) mem_array[addr_i][39:32] <= wdata_i[39:32];
      if (wmask_i[5]) mem_array[addr_i][47:40] <= wdata_i[47:40];
      if (wmask_i[6]) mem_array[addr_i][55:48] <= wdata_i[55:48];
      if (wmask_i[7]) mem_array[addr_i][63:56] <= wdata_i[63:56];
    end
    rdata_o <= mem_array[addr_i];
  end
`else
  // -----------------------------------------------------
  // SIMULATION: Sparse / Optimized Model
  // -----------------------------------------------------
  // Cap sim. memory to 16MB to prevent RAM exhaustion
  localparam SIM_LIMIT_BYTES = 16 * 1024 * 1024;
  localparam SIM_DEPTH = (MEM_SIZE_BYTES > SIM_LIMIT_BYTES) ?
                           (SIM_LIMIT_BYTES / 8) :
                           (MEM_SIZE_BYTES / 8);

  reg [DATA_WIDTH-1:0] mem_array[0:SIM_DEPTH-1];

  initial begin
    integer i;
    for (i = 0; i < SIM_DEPTH; i = i + 1) mem_array[i] = 64'h0;
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem_array);
    end
  end

  always @(posedge clk) begin
    if (addr_i < SIM_DEPTH) begin
      if (we_i) begin
        if (wmask_i[0]) mem_array[addr_i][7:0] <= wdata_i[7:0];
        if (wmask_i[1]) mem_array[addr_i][15:8] <= wdata_i[15:8];
        if (wmask_i[2]) mem_array[addr_i][23:16] <= wdata_i[23:16];
        if (wmask_i[3]) mem_array[addr_i][31:24] <= wdata_i[31:24];
        if (wmask_i[4]) mem_array[addr_i][39:32] <= wdata_i[39:32];
        if (wmask_i[5]) mem_array[addr_i][47:40] <= wdata_i[47:40];
        if (wmask_i[6]) mem_array[addr_i][55:48] <= wdata_i[55:48];
        if (wmask_i[7]) mem_array[addr_i][63:56] <= wdata_i[63:56];
      end
      rdata_o <= mem_array[addr_i];
    end else begin
      rdata_o <= 64'hDEADBEEFDEADBEEF;  // OoB
    end
  end
`endif
endmodule
`endif
