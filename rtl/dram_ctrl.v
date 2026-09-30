// ---------------------------------------------------------------------------
// dram_ctrl.v -- single-bank, open-page DRAM controller
//
// One bank, 8 rows. Synchronous, active-low reset. The controller accepts one
// host request at a time, translates it into DRAM commands (ACT / RD / WR /
// PRE / REF), and enforces DDR4 timing constraints counted in clock cycles.
//
// Page policy: OPEN PAGE. After an access the row is left open, so a request
// to the same row skips ACT entirely (row hit). A request to a different row
// must PRE then ACT (row miss).
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module dram_ctrl #(
    // ---- geometry -------------------------------------------------------
    parameter integer ROW_W = 3,   // 8 rows
    parameter integer COL_W = 4,   // 16 columns
    parameter integer DW    = 32,  // data width

    // ---- timing, in clock cycles ---------------------------------------
    // Values are DDR4-2400 (tCK = 0.833 ns), the "17-17-17" speed bin. That
    // label literally means tCL-tRCD-tRP in cycles, which is where the first
    // three come from. The rest convert the JEDEC nanosecond spec:
    //
    //   tRCD  17   ~14.2 ns  RAS-to-CAS delay: ACT -> RD/WR. Time for the
    //                        row charge to develop on the sense amps.
    //   tRP   17   ~14.2 ns  Row precharge: PRE -> ACT. Restoring bitlines
    //                        to Vdd/2 before another row may be opened.
    //   tCL   17   ~14.2 ns  CAS latency: RD -> first data out.
    //   tRAS  39   ~32   ns  Row active time: ACT -> PRE. A row may not be
    //                        closed early; the capacitors must be restored.
    //   tRC   56   = tRAS + tRP. ACT -> ACT, the full row cycle.
    //   tWR   18   ~15   ns  Write recovery: end of write data -> PRE.
    //   tRFC 420  ~350   ns  Refresh cycle time for an 8 Gb device.
    //   tREFI 9360  7.8  us  Average refresh interval (64 ms / 8192 rows).
    parameter integer tRCD  = 17,
    parameter integer tRP   = 17,
    parameter integer tCL   = 17,
    parameter integer tRAS  = 39,
    parameter integer tRC   = 56,
    parameter integer tWR   = 18,
    parameter integer tRFC  = 420,
    parameter integer tREFI = 9360
) (
    input  wire                clk,
    input  wire                rst_n,

    // ---- host interface -------------------------------------------------
    input  wire                req_valid,
    input  wire                req_write,
    input  wire [ROW_W-1:0]    req_row,
    input  wire [COL_W-1:0]    req_col,
    input  wire [DW-1:0]       req_wdata,
    output wire                req_ready,
    output reg                 done,
    output reg  [DW-1:0]       rdata,
    output reg                 rdata_valid,

    // ---- DRAM command bus -----------------------------------------------
    output reg  [2:0]          cmd,
    output reg  [ROW_W-1:0]    row_addr,
    output reg  [COL_W-1:0]    col_addr,
    output reg  [DW-1:0]       wdata,
    input  wire [DW-1:0]       rdata_in
);

    // ---- command encoding ----------------------------------------------
    localparam [2:0] CMD_NOP = 3'd0,
                     CMD_ACT = 3'd1,
                     CMD_RD  = 3'd2,
                     CMD_WR  = 3'd3,
                     CMD_PRE = 3'd4,
                     CMD_REF = 3'd5;

    // ---- states ---------------------------------------------------------
    localparam [2:0] IDLE        = 3'd0,  // no row open
                     ACTIVATING  = 3'd1,  // ACT issued, waiting tRCD
                     ACTIVE      = 3'd2,  // a row is open, controller idle
                     READING     = 3'd3,  // RD issued, waiting tCL
                     WRITING     = 3'd4,  // WR issued, waiting write latency
                     PRECHARGING = 3'd5,  // PRE issued, waiting tRP
                     REFRESHING  = 3'd6;  // REF issued, waiting tRFC

    reg [2:0] state, next_state;

    // ---- counters -------------------------------------------------------
    localparam integer  CW   = 16;
    localparam [CW-1:0] CMAX = {CW{1'b1}};
    localparam [CW-1:0] CZERO = {CW{1'b0}};

    reg [CW-1:0] t_ctr;    // generic intra-state wait (tRCD/tCL/tRP/tRFC)
    reg [CW-1:0] ras_ctr;  // cycles since last ACT, for tRAS
    reg [CW-1:0] rc_ctr;   // cycles since last ACT, for tRC
    reg [CW-1:0] wr_ctr;   // write-recovery countdown, for tWR
    reg [CW-1:0] ref_ctr;  // cycles since last REF, for tREFI

    // ---- open-row tracking ----------------------------------------------
    reg             row_open;
    reg [ROW_W-1:0] open_row;

    // ---- latched request -------------------------------------------------
    reg             req_busy;
    reg             cur_write;
    reg [ROW_W-1:0] cur_row;
    reg [COL_W-1:0] cur_col;
    reg [DW-1:0]    cur_wdata;

    reg             ref_pending;

    // Refuse new work while a refresh is pending. This is what stops a stream
    // of row hits from starving the refresh: the host is throttled at the
    // door rather than the FSM waiting for an idle moment that never comes.
    assign req_ready = !req_busy && !ref_pending && rst_n;

    // ---- helper conditions -----------------------------------------------
    wire rc_ok  = (rc_ctr  >= tRC [CW-1:0]);  // safe to issue another ACT
    wire ras_ok = (ras_ctr >= tRAS[CW-1:0]);  // row has been open long enough
    wire wr_ok  = (wr_ctr  == CZERO);         // write recovery finished
    wire pre_ok = ras_ok && wr_ok;            // safe to issue PRE
    wire twait  = (t_ctr   != CZERO);         // intra-state wait still running

    wire row_hit  = req_busy && row_open && (cur_row == open_row);
    wire row_miss = req_busy && row_open && (cur_row != open_row);

    // =====================================================================
    // Next-state logic (combinational). Assigning next_state = state first
    // guarantees every path is covered, so this block cannot infer a latch.
    // =====================================================================
    always @(*) begin
        next_state = state;

        case (state)
            IDLE: begin
                // No row is open, so REF needs no precharge first.
                if (ref_pending)            next_state = REFRESHING;
                else if (req_busy && rc_ok) next_state = ACTIVATING;
            end

            ACTIVATING: if (!twait) next_state = ACTIVE;

            ACTIVE: begin
                // Refresh is checked before host work, so a pending refresh
                // always wins. tRAS/tWR still gate the PRE that must precede
                // it -- priority may not override a device timing minimum.
                if (ref_pending) begin
                    if (pre_ok) next_state = PRECHARGING;
                end
                else if (row_hit) begin
                    next_state = cur_write ? WRITING : READING;
                end
                else if (row_miss) begin
                    if (pre_ok) next_state = PRECHARGING;
                end
            end

            READING:     if (!twait) next_state = ACTIVE;
            WRITING:     if (!twait) next_state = ACTIVE;
            PRECHARGING: if (!twait) next_state = IDLE;
            REFRESHING:  if (!twait) next_state = IDLE;

            default: next_state = IDLE;
        endcase
    end

    // ---- how long each state waits once entered --------------------------
    reg [CW-1:0] entry_load;
    always @(*) begin
        case (next_state)
            ACTIVATING:  entry_load = tRCD[CW-1:0] - 16'd1;
            READING:     entry_load = tCL [CW-1:0] - 16'd1;
            WRITING:     entry_load = tCL [CW-1:0] - 16'd1;
            PRECHARGING: entry_load = tRP [CW-1:0] - 16'd1;
            REFRESHING:  entry_load = tRFC[CW-1:0] - 16'd1;
            default:     entry_load = CZERO;
        endcase
    end

    wire entering = (next_state != state);

    // =====================================================================
    // Sequential: state, counters, registered command bus.
    // Nonblocking (<=) throughout, so every register on the right-hand side
    // is read at its pre-edge value. Blocking assignments here would let the
    // order of statements change the hardware, and the command bus would
    // sample counters that had already updated this edge.
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            t_ctr       <= CZERO;
            // Start saturated so the first ACT is not delayed by a tRC/tRAS
            // window that never actually happened.
            ras_ctr     <= CMAX;
            rc_ctr      <= CMAX;
            wr_ctr      <= CZERO;
            ref_ctr     <= CZERO;
            ref_pending <= 1'b0;
            row_open    <= 1'b0;
            open_row    <= {ROW_W{1'b0}};
            req_busy    <= 1'b0;
            cur_write   <= 1'b0;
            cur_row     <= {ROW_W{1'b0}};
            cur_col     <= {COL_W{1'b0}};
            cur_wdata   <= {DW{1'b0}};
            cmd         <= CMD_NOP;
            row_addr    <= {ROW_W{1'b0}};
            col_addr    <= {COL_W{1'b0}};
            wdata       <= {DW{1'b0}};
            done        <= 1'b0;
            rdata       <= {DW{1'b0}};
            rdata_valid <= 1'b0;
        end else begin
            state       <= next_state;
            done        <= 1'b0;
            rdata_valid <= 1'b0;
            cmd         <= CMD_NOP;

            // ---- intra-state wait timer ------------------------------------
            if (entering)             t_ctr <= entry_load;
            else if (t_ctr != CZERO)  t_ctr <= t_ctr - 16'd1;

            // ---- saturating counters ---------------------------------------
            if (ras_ctr != CMAX)  ras_ctr <= ras_ctr + 16'd1;
            if (rc_ctr  != CMAX)  rc_ctr  <= rc_ctr  + 16'd1;
            if (wr_ctr  != CZERO) wr_ctr  <= wr_ctr  - 16'd1;

            // ---- refresh interval -------------------------------------------
            if (ref_ctr >= tREFI[CW-1:0]) ref_pending <= 1'b1;
            ref_ctr <= ref_ctr + 16'd1;

            // ---- capture a new host request ---------------------------------
            if (req_valid && req_ready) begin
                req_busy  <= 1'b1;
                cur_write <= req_write;
                cur_row   <= req_row;
                cur_col   <= req_col;
                cur_wdata <= req_wdata;
            end

            // ---- issue the command on the first cycle of each state ---------
            // These assignments come last, so where they overlap the counter
            // updates above (ras_ctr, rc_ctr) the command-time value wins.
            if (entering) begin
                case (next_state)
                    ACTIVATING: begin
                        cmd      <= CMD_ACT;
                        row_addr <= cur_row;
                        open_row <= cur_row;
                        row_open <= 1'b1;
                        ras_ctr  <= CZERO;
                        rc_ctr   <= CZERO;
                    end
                    READING: begin
                        cmd      <= CMD_RD;
                        col_addr <= cur_col;
                    end
                    WRITING: begin
                        cmd      <= CMD_WR;
                        col_addr <= cur_col;
                        wdata    <= cur_wdata;
                    end
                    PRECHARGING: begin
                        cmd      <= CMD_PRE;
                        row_open <= 1'b0;
                    end
                    REFRESHING: begin
                        cmd         <= CMD_REF;
                        ref_pending <= 1'b0;
                        ref_ctr     <= CZERO;
                        // A refresh internally activates and precharges every
                        // row, so it counts as a completed row cycle.
                        ras_ctr     <= CMAX;
                        rc_ctr      <= CMAX;
                    end
                    default: ;
                endcase
            end

            // ---- completion --------------------------------------------------
            if (state == READING && !twait) begin
                rdata       <= rdata_in;
                rdata_valid <= 1'b1;
                done        <= 1'b1;
                req_busy    <= 1'b0;
            end
            if (state == WRITING && !twait) begin
                wr_ctr   <= tWR[CW-1:0];   // write recovery starts now
                done     <= 1'b1;
                req_busy <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
