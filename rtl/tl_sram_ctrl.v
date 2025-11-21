`ifndef TL_SRAM_CTRL_V
`define TL_SRAM_CTRL_V

module tl_sram_ctrl #(
    // -------------------------------------------------------
    // Configuration Parameters
    // -------------------------------------------------------
    parameter MEM_SIZE_BYTES = 64 * 1024 * 1024,  // 64 MiB
    parameter DATA_WIDTH     = 64,                // Fixed 64-bit
    parameter ADDR_WIDTH     = 32,
    parameter SOURCE_WIDTH   = 4,
    parameter QUEUE_DEPTH    = 4,                 // Outstanding req depth

    parameter READ_LATENCY     = 1,  // Cycles
    parameter WRITE_LATENCY    = 1,  // Cycles
    parameter CACHE_LINE_BYTES = 64, // Validation only

    parameter INIT_FILE = ""  // Firmware Hex Load
) (
    input wire clk_i,
    input wire rst_ni,

    // -------------------------------------------------------
    // TileLink-UH Interface
    // -------------------------------------------------------
    // Channel A (Request)
    input  wire                    tl_a_valid,
    output wire                    tl_a_ready,
    input  wire [             2:0] tl_a_opcode,
    input  wire [             2:0] tl_a_param,    // Ignored (Uncached)
    input  wire [             2:0] tl_a_size,
    input  wire [SOURCE_WIDTH-1:0] tl_a_source,
    input  wire [  ADDR_WIDTH-1:0] tl_a_address,
    input  wire [  DATA_WIDTH-1:0] tl_a_data,
    input  wire [             7:0] tl_a_mask,

    // Channel D (Response)
    output reg                     tl_d_valid,
    input  wire                    tl_d_ready,
    output reg  [             2:0] tl_d_opcode,
    output reg  [             2:0] tl_d_param,
    output reg  [             2:0] tl_d_size,
    output reg  [SOURCE_WIDTH-1:0] tl_d_source,
    output reg  [  DATA_WIDTH-1:0] tl_d_data,
    output reg                     tl_d_corrupt,

    // -------------------------------------------------------
    // Monitoring Statistics
    // -------------------------------------------------------
    output reg [31:0] stat_read_cnt,
    output reg [31:0] stat_write_cnt,
    output reg [63:0] stat_total_latency
);

  // ---------------------------------------------------------
  // Constants & Functions
  // ---------------------------------------------------------
  localparam OP_GET = 3'b000;
  localparam OP_PUTFULL = 3'b001;
  localparam OP_PUTPART = 3'b010;
  localparam OP_ACK = 3'b000;
  localparam OP_ACKDATA = 3'b001;

  // Calculate burst beats based on size: 2^(size-3)
  function [8:0] calc_beats;
    input [2:0] size;
    begin
      if (size > 3) calc_beats = (1 << (size - 3));
      else calc_beats = 1;
    end
  endfunction

  // ---------------------------------------------------------
  // Parameter Validation
  // ---------------------------------------------------------
  // synthesis translate_off
  initial begin
    if (DATA_WIDTH != 64) $error("TL_SRAM_CTRL: DATA_WIDTH must be 64.");
    if (QUEUE_DEPTH == 0) $error("TL_SRAM_CTRL: QUEUE_DEPTH must be > 0.");
  end
  // synthesis translate_on

  // ---------------------------------------------------------
  // Unified Transaction FIFO
  // ---------------------------------------------------------
  // A singl circular buffer is used to guarantee strict ordering.
  // Prevents Read-After-Write hazards inherent in single-port RAMs.

  reg  [                    2:0] q_opcode                          [0:QUEUE_DEPTH-1];
  reg  [                    2:0] q_size                            [0:QUEUE_DEPTH-1];
  reg  [       SOURCE_WIDTH-1:0] q_source                          [0:QUEUE_DEPTH-1];
  reg  [         ADDR_WIDTH-1:0] q_addr                            [0:QUEUE_DEPTH-1];
  reg  [         DATA_WIDTH-1:0] q_data                            [0:QUEUE_DEPTH-1];
  reg  [                    7:0] q_mask                            [0:QUEUE_DEPTH-1];
  reg  [                   63:0] q_timestamp                       [0:QUEUE_DEPTH-1];

  reg  [  $clog2(QUEUE_DEPTH):0] q_count;
  reg  [$clog2(QUEUE_DEPTH)-1:0] q_wr_ptr;
  reg  [$clog2(QUEUE_DEPTH)-1:0] q_rd_ptr;

  reg  [                   63:0] cycle_count;

  wire                           q_full = (q_count == QUEUE_DEPTH);
  wire                           q_empty = (q_count == 0);

  // ---------------------------------------------------------
  // (FSM) Signals
  // ---------------------------------------------------------
  localparam S_IDLE = 3'd0;
  localparam S_DRAIN_WR = 3'd1;  // Absorb remaining burst beats
  localparam S_LATENCY = 3'd2;  // Simulate access delay
  localparam S_RESP_RD = 3'd3;  // Drive Read Burst
  localparam S_RESP_WR = 3'd4;  // Drive Write Ack

  reg  [             2:0] state;
  reg  [             8:0] beat_cnt;
  reg  [             7:0] lat_cnt;

  // Active Transaction Registers (Latched from FIFO)
  reg  [             2:0] act_opcode;
  reg  [             2:0] act_size;
  reg  [SOURCE_WIDTH-1:0] act_source;
  reg  [  ADDR_WIDTH-1:0] act_addr;
  reg  [            63:0] act_start_time;
  reg                     act_error;

  // Memory Interface
  reg                     mem_we;
  reg  [  ADDR_WIDTH-1:0] mem_addr;
  reg  [  DATA_WIDTH-1:0] mem_wdata;
  reg  [             7:0] mem_wmask;
  wire [  DATA_WIDTH-1:0] mem_rdata;

  // ---------------------------------------------------------
  // Alignment Checker
  // ---------------------------------------------------------
  reg                     alignment_error;
  always @(*) begin
    case (q_size[q_rd_ptr])
      3'd0:    alignment_error = 1'b0;  // 1 Byte
      3'd1:    alignment_error = q_addr[q_rd_ptr][0];  // 2 Byte
      3'd2:    alignment_error = |q_addr[q_rd_ptr][1:0];  // 4 Byte
      3'd3:    alignment_error = |q_addr[q_rd_ptr][2:0];  // 8 Byte
      default: alignment_error = |q_addr[q_rd_ptr][2:0];  // >8 Bytes
    endcase
  end

  // ---------------------------------------------------------
  // Memory Instantiation
  // ---------------------------------------------------------
  tl_sram_storage #(
      .DATA_WIDTH(DATA_WIDTH),
      .MEM_SIZE_BYTES(MEM_SIZE_BYTES),
      .INIT_FILE(INIT_FILE)
  ) u_ram (
      .clk    (clk_i),
      .we_i   (mem_we),
      // padding 29 to 32 for verilator Lint
      .addr_i ({3'b0, mem_addr[ADDR_WIDTH-1:3]}),  // Byte =-> Word Address
      .wdata_i(mem_wdata),
      .wmask_i(mem_wmask),
      .rdata_o(mem_rdata)
  );

  // ---------------------------------------------------------
  // Input Logic (FIFO Enqueue)
  // ---------------------------------------------------------
  // Ready condition: Queue not full, unless we are actively draining Write data.
  assign tl_a_ready = (state == S_DRAIN_WR) || (!q_full);

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_count     <= 0;
      q_wr_ptr    <= 0;
      cycle_count <= 0;
    end else begin
      cycle_count <= cycle_count + 1;

      // Enqueue Request Header & first Beat Data
      if (tl_a_valid && !q_full && state != S_DRAIN_WR) begin
        q_opcode[q_wr_ptr] <= tl_a_opcode;
        q_size[q_wr_ptr] <= tl_a_size;
        q_source[q_wr_ptr] <= tl_a_source;
        q_addr[q_wr_ptr] <= tl_a_address;
        q_data[q_wr_ptr] <= tl_a_data;
        q_mask[q_wr_ptr] <= tl_a_mask;
        q_timestamp[q_wr_ptr] <= cycle_count;

        q_wr_ptr <= q_wr_ptr + 1;
        q_count <= q_count + 1;
      end

      // Dequeue (andled by FSM transition)
      if (state == S_IDLE && !q_empty) begin
        q_count <= q_count - 1;
      end
    end
  end

  // ---------------------------------------------------------
  // Main FSM
  // ---------------------------------------------------------
  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state              <= S_IDLE;
      q_rd_ptr           <= 0;
      tl_d_valid         <= 0;
      tl_d_corrupt       <= 0;
      tl_d_param         <= 0;
      stat_read_cnt      <= 0;
      stat_write_cnt     <= 0;
      stat_total_latency <= 0;
      mem_we             <= 0;
      act_error          <= 0;
    end else begin
      // Default Control
      mem_we     <= 0;
      tl_d_valid <= 0;
      tl_d_param <= 0;

      case (state)
        // --- S_IDLE: Dispatch Transaction ---
        S_IDLE: begin
          if (!q_empty) begin
            act_opcode <= q_opcode[q_rd_ptr];
            act_size <= q_size[q_rd_ptr];
            act_source <= q_source[q_rd_ptr];
            act_addr <= q_addr[q_rd_ptr];
            act_start_time <= q_timestamp[q_rd_ptr];

            // Error Detectiona
            if (q_addr[q_rd_ptr] >= MEM_SIZE_BYTES ||
                            alignment_error                    ||
                           (q_opcode[q_rd_ptr] > 2)) begin
              act_error <= 1'b1;
            end else begin
              act_error <= 1'b0;
            end

            beat_cnt <= calc_beats(q_size[q_rd_ptr]);

            // Dispatch Read or Write
            if (q_opcode[q_rd_ptr] == OP_GET) begin
              stat_read_cnt <= stat_read_cnt + 1;
              lat_cnt  <= (READ_LATENCY > 0) ? READ_LATENCY - 1 : 0;
              mem_addr <= q_addr[q_rd_ptr];
              q_rd_ptr <= q_rd_ptr + 1;
              state    <= S_LATENCY;
            end else begin
              stat_write_cnt <= stat_write_cnt + 1;
              // Commit Head Data Immediately
              mem_we    <= (act_error == 0);
              mem_addr  <= q_addr[q_rd_ptr];
              mem_wdata <= q_data[q_rd_ptr];
              mem_wmask <= q_mask[q_rd_ptr];

              act_addr  <= q_addr[q_rd_ptr] + 8;
              q_rd_ptr  <= q_rd_ptr + 1;

              // Check if Burst or Single
              if (calc_beats(q_size[q_rd_ptr]) > 1) begin
                beat_cnt <= calc_beats(q_size[q_rd_ptr]) - 1;
                state    <= S_DRAIN_WR;
              end else begin
                lat_cnt <= (WRITE_LATENCY > 0) ? WRITE_LATENCY - 1 : 0;
                state   <= S_LATENCY;
              end
            end
          end
        end

        // --- S_DRAIN_WR: Consume Burst Data ---
        S_DRAIN_WR: begin
          // A_READY is high in this state.
          if (tl_a_valid) begin
            mem_we    <= !act_error;
            mem_addr  <= act_addr;
            mem_wdata <= tl_a_data;
            mem_wmask <= tl_a_mask;

            act_addr <= act_addr + 8;
            beat_cnt <= beat_cnt - 1;

            if (beat_cnt == 1) begin
              lat_cnt <= (WRITE_LATENCY > 0) ? WRITE_LATENCY - 1 : 0;
              state   <= S_LATENCY;
            end
          end
        end

        // --- S_LATENCY: Simulate Hardware Delay ---
        S_LATENCY: begin
          if (lat_cnt > 0) begin
            lat_cnt <= lat_cnt - 1;
          end else begin
            stat_total_latency <= stat_total_latency + (cycle_count - act_start_time); // metric Update

            if (act_opcode == OP_GET) state <= S_RESP_RD;
            else state <= S_RESP_WR;
          end
        end

        // --- S_RESP_RD: Read Response Burst ---
        S_RESP_RD: begin
          tl_d_valid <= 1'b1;
          tl_d_opcode <= OP_ACKDATA;
          tl_d_size <= act_size;
          tl_d_source <= act_source;
          tl_d_data <= act_error ? 64'hDEADBEEFBAD0BAD0 : mem_rdata;
          tl_d_corrupt <= act_error;

          mem_addr <= act_addr + 8;  // Pipelined address increment

          if (tl_d_ready) begin
            act_addr <= act_addr + 8;
            beat_cnt <= beat_cnt - 1;
            if (beat_cnt == 1) state <= S_IDLE;
          end
        end

        // --- S_RESP_WR: Write Ack---
        S_RESP_WR: begin
          tl_d_valid <= 1'b1;
          tl_d_opcode <= OP_ACK;
          tl_d_size <= act_size;
          tl_d_source <= act_source;
          tl_d_corrupt <= act_error;

          if (tl_d_ready) state <= S_IDLE;
        end
        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
`endif
