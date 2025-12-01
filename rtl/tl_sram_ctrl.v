`ifndef TL_SRAM_CTRL_V
`define TL_SRAM_CTRL_V

module tl_sram_ctrl #(
    // Memory Configuration
    parameter MEM_SIZE_BYTES = 64 * 1024 * 1024,  // 64 MiB default
    parameter DATA_WIDTH     = 64,                // Fixed 64-bit TileLink bus
    parameter ADDR_WIDTH     = 32,                // Byte-addressable space
    parameter SOURCE_WIDTH   = 4,                 // Max 16 outstanding transactions
    parameter QUEUE_DEPTH    = 8,                 // Internal buffering depth

    // DDR Timing Parameters (in controller clock cycles)
    parameter T_CAS = 4,  // Column Access Strobe (min 3 cycles)
    parameter T_RCD = 4,  // Row Command Delay (Activate to Read/Write)
    parameter T_RP  = 4,  // Row Precharge (Close Row to Idle)
    parameter T_WR  = 4,  // Write Recovery (Last Write to Precharge)

    parameter CACHE_LINE_BYTES = 64,
    parameter INIT_FILE        = ""
) (
    input wire clk_i,
    input wire rst_ni,

    // TileLink-UH Channel A (Request)
    input  wire                    tl_a_valid,
    output wire                    tl_a_ready,
    input  wire [             2:0] tl_a_opcode,
    input  wire [             2:0] tl_a_param,
    input  wire [             2:0] tl_a_size,
    input  wire [SOURCE_WIDTH-1:0] tl_a_source,
    input  wire [  ADDR_WIDTH-1:0] tl_a_address,
    input  wire [  DATA_WIDTH-1:0] tl_a_data,
    input  wire [             7:0] tl_a_mask,

    // TileLink-UH Channel D (Response)
    output reg                     tl_d_valid,
    input  wire                    tl_d_ready,
    output reg  [             2:0] tl_d_opcode,
    output reg  [             2:0] tl_d_param,
    output reg  [             2:0] tl_d_size,
    output reg  [SOURCE_WIDTH-1:0] tl_d_source,
    output reg  [  DATA_WIDTH-1:0] tl_d_data,
    output reg                     tl_d_corrupt,
    output reg                     tl_d_denied,

    // Performance Counters
    output reg [31:0] stat_read_cnt,
    output reg [31:0] stat_write_cnt,
    output reg [31:0] stat_row_hits,
    output reg [31:0] stat_row_misses,
    output reg [63:0] stat_total_latency
);

  //===========================================================================
  // Constants
  //===========================================================================

  // TileLink 1.8.1 Opcodes
  localparam OP_PUTFULL = 3'd0;
  localparam OP_PUTPART = 3'd1;
  localparam OP_GET = 3'd4;
  localparam OP_ACK = 3'd0;
  localparam OP_ACKDATA = 3'd1;

  //===========================================================================
  // Parameter Validation
  //===========================================================================

  // synthesis translate_off
  initial begin
    if (T_CAS < 3)
      $fatal(1, "[TL_SRAM_CTRL] T_CAS=%0d too small. Must be >= 3 for single-port BRAM.", T_CAS);
    if (T_RCD < 1) $fatal(1, "[TL_SRAM_CTRL] T_RCD must be >= 1");
    if (T_RP < 1) $fatal(1, "[TL_SRAM_CTRL] T_RP must be >= 1");
    if (T_WR < 1) $fatal(1, "[TL_SRAM_CTRL] T_WR must be >= 1");
    if (DATA_WIDTH != 64) $fatal(1, "[TL_SRAM_CTRL] DATA_WIDTH must be 64 (TileLink-UH spec)");
    if (QUEUE_DEPTH < 2 || QUEUE_DEPTH > 256)
      $fatal(1, "[TL_SRAM_CTRL] QUEUE_DEPTH must be in range [2, 256]");

    $display("[TL_SRAM_CTRL] Configuration:");
    $display("  Memory: %0d MiB", MEM_SIZE_BYTES / (1024 * 1024));
    $display("  Timing: T_CAS=%0d, T_RCD=%0d, T_RP=%0d, T_WR=%0d", T_CAS, T_RCD, T_RP, T_WR);
    $display("  Queue Depth: %0d entries", QUEUE_DEPTH);
  end
  // synthesis translate_on

  //===========================================================================
  // Helper Functions
  //===========================================================================

  function integer clog2;
    input [31:0] value;
    begin
      value = value - 1;
      for (clog2 = 0; value > 0; clog2 = clog2 + 1) value = value >> 1;
    end
  endfunction

  // Calculate burst length from TileLink size field
  function [8:0] calc_beats;
    input [2:0] size;
    begin
      if (size > 3) calc_beats = (1 << (size - 3));
      else calc_beats = 1;
    end
  endfunction

  //===========================================================================
  // DRAM Architecture Model (8 banks, 4KB row buffer per bank)
  //===========================================================================

  localparam NUM_BANKS = 8;
  localparam BANK_BITS = 3;
  localparam PAGE_OFFSET = 12;  // 4KB = 2^12 bytes

  reg [ADDR_WIDTH-1 : PAGE_OFFSET + BANK_BITS] active_row[0:NUM_BANKS-1];
  reg                                          row_open  [0:NUM_BANKS-1];

  function [BANK_BITS-1:0] get_bank;
    input [ADDR_WIDTH-1:0] addr;
    begin
      get_bank = addr[PAGE_OFFSET+BANK_BITS-1 : PAGE_OFFSET];
    end
  endfunction

  function [ADDR_WIDTH-1 : PAGE_OFFSET + BANK_BITS] get_row;
    input [ADDR_WIDTH-1:0] addr;
    begin
      get_row = addr[ADDR_WIDTH-1 : PAGE_OFFSET+BANK_BITS];
    end
  endfunction

  //===========================================================================
  // Unified Request FIFO (prevents read-after-write hazards)
  //===========================================================================

  localparam PTR_WIDTH = clog2(QUEUE_DEPTH);

  reg  [             2:0] q_opcode                          [0:QUEUE_DEPTH-1];
  reg  [             2:0] q_size                            [0:QUEUE_DEPTH-1];
  reg  [SOURCE_WIDTH-1:0] q_source                          [0:QUEUE_DEPTH-1];
  reg  [  ADDR_WIDTH-1:0] q_addr                            [0:QUEUE_DEPTH-1];
  reg  [  DATA_WIDTH-1:0] q_data                            [0:QUEUE_DEPTH-1];
  reg  [             7:0] q_mask                            [0:QUEUE_DEPTH-1];
  reg  [            63:0] q_timestamp                       [0:QUEUE_DEPTH-1];

  reg  [     PTR_WIDTH:0] q_count;
  reg  [   PTR_WIDTH-1:0] q_wr_ptr;
  reg  [   PTR_WIDTH-1:0] q_rd_ptr;

  reg  [            63:0] cycle_count;

  wire                    q_full = (q_count == QUEUE_DEPTH);
  wire                    q_empty = (q_count == 0);

  //===========================================================================
  // FSM States
  //===========================================================================

  localparam S_IDLE = 3'd0;  // Dispatch next transaction
  localparam S_DRAIN_WR = 3'd1;  // Absorb multi-beat write data
  localparam S_LATENCY = 3'd2;  // Wait for DRAM timing
  localparam S_RESP_RD = 3'd3;  // Stream read response beats
  localparam S_RESP_RD_WAIT = 3'd4;  // BRAM stabilization bubble cycle
  localparam S_RESP_WR = 3'd5;  // Send write acknowledgment

  reg  [             2:0] state;
  reg  [             8:0] beat_cnt;
  reg  [             7:0] lat_cnt;

  // Active Transaction Context
  reg  [             2:0] act_opcode;
  reg  [             2:0] act_size;
  reg  [SOURCE_WIDTH-1:0] act_source;
  reg  [  ADDR_WIDTH-1:0] act_addr;
  reg  [            63:0] act_start_time;
  reg                     act_error;

  // BRAM Interface
  reg                     mem_we;
  reg  [  ADDR_WIDTH-1:0] mem_addr;
  reg  [  DATA_WIDTH-1:0] mem_wdata;
  reg  [             7:0] mem_wmask;
  wire [  DATA_WIDTH-1:0] mem_rdata;

  //===========================================================================
  // Error Detection
  //===========================================================================

  reg                     addr_misaligned;
  always @(*) begin
    case (q_size[q_rd_ptr])
      3'd0:    addr_misaligned = 1'b0;
      3'd1:    addr_misaligned = q_addr[q_rd_ptr][0];
      3'd2:    addr_misaligned = |q_addr[q_rd_ptr][1:0];
      3'd3:    addr_misaligned = |q_addr[q_rd_ptr][2:0];
      default: addr_misaligned = |q_addr[q_rd_ptr][2:0];
    endcase
  end

  //===========================================================================
  // Memory Storage Instance
  //===========================================================================

  tl_sram_storage #(
      .DATA_WIDTH(DATA_WIDTH),
      .MEM_SIZE_BYTES(MEM_SIZE_BYTES),
      .INIT_FILE(INIT_FILE)
  ) u_ram (
      .clk    (clk_i),
      .we_i   (mem_we),
      .addr_i ({3'b0, mem_addr[ADDR_WIDTH-1:3]}),
      .wdata_i(mem_wdata),
      .wmask_i(mem_wmask),
      .rdata_o(mem_rdata)
  );

  //===========================================================================
  // Input Queue Logic
  //===========================================================================

  wire do_push = tl_a_valid && !q_full && (state != S_DRAIN_WR);
  wire do_pop = (state == S_IDLE && !q_empty);

  assign tl_a_ready = (state == S_DRAIN_WR) || (!q_full);

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_count     <= 0;
      q_wr_ptr    <= 0;
      cycle_count <= 0;
    end else begin
      cycle_count <= cycle_count + 1;

      if (do_push) begin
        q_opcode[q_wr_ptr] <= tl_a_opcode;
        q_size[q_wr_ptr] <= tl_a_size;
        q_source[q_wr_ptr] <= tl_a_source;
        q_addr[q_wr_ptr] <= tl_a_address;
        q_data[q_wr_ptr] <= tl_a_data;
        q_mask[q_wr_ptr] <= tl_a_mask;
        q_timestamp[q_wr_ptr] <= cycle_count;
        q_wr_ptr <= q_wr_ptr + 1;
      end

      if (do_push && !do_pop) q_count <= q_count + 1;
      else if (!do_push && do_pop) q_count <= q_count - 1;
    end
  end

  //===========================================================================
  // Main Control FSM
  //===========================================================================

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state              <= S_IDLE;
      q_rd_ptr           <= 0;
      tl_d_valid         <= 0;
      tl_d_corrupt       <= 0;
      tl_d_denied        <= 0;
      tl_d_param         <= 0;
      stat_read_cnt      <= 0;
      stat_write_cnt     <= 0;
      stat_row_hits      <= 0;
      stat_row_misses    <= 0;
      stat_total_latency <= 0;
      mem_we             <= 0;
      act_error          <= 0;

      begin : init_banks
        integer i;
        for (i = 0; i < NUM_BANKS; i = i + 1) begin
          row_open[i]   <= 0;
          active_row[i] <= 0;
        end
      end
    end else begin
      // Default: de-assert pulse signals
      mem_we       <= 0;
      tl_d_valid   <= 0;
      tl_d_param   <= 0;
      tl_d_corrupt <= 0;
      tl_d_denied  <= 0;

      case (state)
        //=====================================================================
        // S_IDLE: Dispatch Next Transaction
        //=====================================================================
        S_IDLE: begin
          if (!q_empty) begin
            act_opcode <= q_opcode[q_rd_ptr];
            act_size <= q_size[q_rd_ptr];
            act_source <= q_source[q_rd_ptr];
            act_addr <= q_addr[q_rd_ptr];
            act_start_time <= cycle_count;
            q_rd_ptr <= q_rd_ptr + 1;

            // Error detection
            if (q_addr[q_rd_ptr] >= MEM_SIZE_BYTES ||
                            addr_misaligned                    ||
                           !(q_opcode[q_rd_ptr] == OP_PUTFULL ||
                             q_opcode[q_rd_ptr] == OP_PUTPART ||
                             q_opcode[q_rd_ptr] == OP_GET)) begin
              act_error <= 1'b1;

              // synthesis translate_off
              if (q_addr[q_rd_ptr] >= MEM_SIZE_BYTES)
                $display(
                    "[TL_SRAM] Error: OOB access 0x%h >= 0x%h", q_addr[q_rd_ptr], MEM_SIZE_BYTES
                );
              if (addr_misaligned)
                $display(
                    "[TL_SRAM] Error: Misaligned addr=0x%h size=%0d",
                    q_addr[q_rd_ptr],
                    q_size[q_rd_ptr]
                );
              // synthesis translate_on
            end else begin
              act_error <= 1'b0;
            end

            beat_cnt <= calc_beats(q_size[q_rd_ptr]);

            // DRAM Bank/Row State Machine
            begin
              reg [BANK_BITS-1:0] cur_bank;
              reg [ADDR_WIDTH-1 : PAGE_OFFSET + BANK_BITS] cur_row;
              reg [7:0] dram_latency;

              cur_bank = get_bank(q_addr[q_rd_ptr]);
              cur_row  = get_row(q_addr[q_rd_ptr]);

              // Row buffer hit/miss determination
              if (row_open[cur_bank] && active_row[cur_bank] == cur_row) begin
                dram_latency = T_CAS;  // Row buffer hit
                stat_row_hits <= stat_row_hits + 1;
              end else if (row_open[cur_bank]) begin
                dram_latency = T_RP + T_RCD + T_CAS;  // Row conflict
                stat_row_misses <= stat_row_misses + 1;
                active_row[cur_bank] <= cur_row;
              end else begin
                dram_latency = T_RCD + T_CAS;  // Row closed
                stat_row_misses <= stat_row_misses + 1;
                active_row[cur_bank] <= cur_row;
                row_open[cur_bank] <= 1'b1;
              end

              // Adjust for FSM overhead (2 cycles minimum)
              if (dram_latency > 2) lat_cnt <= dram_latency - 2;
              else lat_cnt <= 0;
            end

            mem_addr <= q_addr[q_rd_ptr];

            if (q_opcode[q_rd_ptr] == OP_GET) begin
              // Read path
              stat_read_cnt <= stat_read_cnt + 1;
              state <= S_LATENCY;
            end else begin
              // Write path - write first beat immediately
              stat_write_cnt <= stat_write_cnt + 1;
              mem_we    <= !act_error;
              mem_wdata <= q_data[q_rd_ptr];
              mem_wmask <= q_mask[q_rd_ptr];
              act_addr  <= q_addr[q_rd_ptr] + 8;

              if (calc_beats(q_size[q_rd_ptr]) > 1) begin
                beat_cnt <= calc_beats(q_size[q_rd_ptr]) - 1;
                state    <= S_DRAIN_WR;
              end else begin
                state <= S_LATENCY;
              end
            end
          end
        end

        //=====================================================================
        // S_DRAIN_WR: Absorb Remaining Write Beats
        //=====================================================================
        S_DRAIN_WR: begin
          if (tl_a_valid) begin
            mem_we    <= !act_error;
            mem_addr  <= act_addr;
            mem_wdata <= tl_a_data;
            mem_wmask <= tl_a_mask;
            act_addr  <= act_addr + 8;
            beat_cnt  <= beat_cnt - 1;

            if (beat_cnt == 1) begin
              lat_cnt <= (T_WR > 1) ? T_WR - 1 : 0;
              state   <= S_LATENCY;
            end
          end
        end

        //=====================================================================
        // S_LATENCY: DRAM Timing Wait State
        //=====================================================================
        S_LATENCY: begin
          if (lat_cnt > 0) begin
            lat_cnt <= lat_cnt - 1;
          end else begin
            stat_total_latency <= stat_total_latency + (cycle_count - act_start_time);

            if (act_opcode == OP_GET) state <= S_RESP_RD;
            else state <= S_RESP_WR;
          end
        end

        //=====================================================================
        // S_RESP_RD: Stream Read Data Beats
        //=====================================================================
        S_RESP_RD: begin
          tl_d_valid   <= 1'b1;
          tl_d_opcode  <= OP_ACKDATA;
          tl_d_size    <= act_size;
          tl_d_source  <= act_source;
          tl_d_corrupt <= act_error;
          tl_d_data    <= act_error ? 64'hDEADBEEFBAD0BAD0 : mem_rdata;

          if (tl_d_ready) begin
            if (beat_cnt == 1) begin
              state <= S_IDLE;
            end else begin
              beat_cnt <= beat_cnt - 1;
              act_addr <= act_addr + 8;
              mem_addr <= act_addr + 8;
              state    <= S_RESP_RD_WAIT;  // Inserting bubble for BRAM stabilization

              // synthesis translate_off
              if ((act_addr + 8) < act_addr) $error("[TL_SRAM] Address overflow in read burst!");
              // synthesis translate_on
            end
          end
        end

        //=====================================================================
        // S_RESP_RD_WAIT: BRAM Stabilization Bubble
        //=====================================================================
        // Waitt one cycle for BRAM output to stabilize after address change
        S_RESP_RD_WAIT: begin
          state <= S_RESP_RD;
        end

        //=====================================================================
        // S_RESP_WR: Send Write Acknowledgment
        //=====================================================================
        S_RESP_WR: begin
          tl_d_valid  <= 1'b1;
          tl_d_opcode <= OP_ACK;
          tl_d_size   <= act_size;
          tl_d_source <= act_source;
          tl_d_denied <= act_error;

          if (tl_d_ready) state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
`endif
