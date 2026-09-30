// ---------------------------------------------------------------------------
// dram_ctrl_mb.v -- multi-bank DDR4 controller with FR-FCFS scheduling
//
// Generalises the single-bank controller in two directions:
//
//   NBANKS  independent banks, each with its own row state and timing, so an
//           ACT on one bank overlaps data movement on another. Adds the two
//           inter-bank constraints that only exist once banks can race:
//           tRRD (ACT-to-ACT spacing) and tFAW (at most 4 ACTs per window).
//           Both are POWER limits, not settling times -- activating a row is
//           the most current-hungry operation a DRAM performs.
//
//   QDEPTH  entries of request queue with FR-FCFS arbitration:
//           First-Ready, First-Come-First-Served. Each cycle the scheduler
//           prefers a column command that can issue right now (a row hit on
//           an already-open bank) over a row command, and among equals takes
//           the oldest. This is what real memory controllers do, and it turns
//           the row hit rate from a property of the workload into a property
//           of the scheduler.
//
// Setting NBANKS=1, QDEPTH=1 reduces this to the in-order single-bank
// baseline, which is how the comparison in docs/README.md is kept honest:
// one RTL source, one address stream, only the parameters change.
//
// Simplification, documented deliberately: read data is captured from
// rdata_in when the RD command issues and carried through the completion
// pipeline, so it emerges at the host tCL later. The host-visible timing is
// correct; only the modelling of the DQ bus itself is shortcut.
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module dram_ctrl_mb #(
    parameter integer NBANKS = 4,
    parameter integer QDEPTH = 8,
    parameter integer ROW_W  = 3,
    parameter integer COL_W  = 4,
    parameter integer DW     = 32,
    parameter integer IDW    = 8,

    // ---- intra-bank timing (DDR4-2400, tCK = 0.833 ns) ------------------
    parameter integer tRCD  = 17,
    parameter integer tRP   = 17,
    parameter integer tCL   = 17,
    parameter integer tRAS  = 39,
    parameter integer tRC   = 56,
    parameter integer tWR   = 18,
    parameter integer tRFC  = 420,
    parameter integer tREFI = 9360,

    // ---- inter-bank timing ----------------------------------------------
    //   tRRD  6  ~5 ns   ACT -> ACT on a DIFFERENT bank. Spaces out the
    //                    current spikes caused by opening rows.
    //   tFAW 26 ~21 ns   Four-activate window: no more than 4 ACTs may start
    //                    in any rolling tFAW. A hard power ceiling on how
    //                    much bank parallelism can actually be used.
    //   tCCD  4          Column-to-column delay. One column command every 4
    //                    cycles is BL8 at double data rate -- the data bus is
    //                    simply busy. (Real DDR4 splits this into tCCD_S=4
    //                    across bank groups and tCCD_L=6 within one.)
    parameter integer tRRD = 6,
    parameter integer tFAW = 26,
    parameter integer tCCD = 4
) (
    input  wire               clk,
    input  wire               rst_n,

    // ---- host interface ---------------------------------------------------
    input  wire               req_valid,
    input  wire               req_write,
    input  wire [BA_W-1:0]    req_bank,
    input  wire [ROW_W-1:0]   req_row,
    input  wire [COL_W-1:0]   req_col,
    input  wire [DW-1:0]      req_wdata,
    input  wire [IDW-1:0]     req_id,
    output wire               req_ready,

    output reg                done,
    output reg  [IDW-1:0]     done_id,
    output reg  [DW-1:0]      rdata,
    output reg                rdata_valid,

    // ---- DRAM command bus -------------------------------------------------
    output reg  [2:0]         cmd,
    output reg  [BA_W-1:0]    cmd_bank,
    output reg  [ROW_W-1:0]   row_addr,
    output reg  [COL_W-1:0]   col_addr,
    output reg  [DW-1:0]      wdata,
    input  wire [DW-1:0]      rdata_in
);

    // clog2 that also yields 1 for NBANKS==1, so the port width is never zero.
    function integer clog2b;
        input integer v;
        integer i;
        begin
            clog2b = 1;
            for (i = 1; (1 << i) < v; i = i + 1) clog2b = i + 1;
        end
    endfunction
    localparam integer BA_W = clog2b(NBANKS);

    localparam [2:0] CMD_NOP = 3'd0, CMD_ACT = 3'd1, CMD_RD = 3'd2,
                     CMD_WR  = 3'd3, CMD_PRE = 3'd4, CMD_REF = 3'd5;

    localparam integer CW     = 16;
    localparam [CW-1:0] CZERO = {CW{1'b0}};
    localparam [CW-1:0] CMAX  = {CW{1'b1}};
    localparam integer MAXCL  = 64;   // completion pipeline depth bound

    integer i, b, k;

    // =====================================================================
    // Per-bank state
    // =====================================================================
    reg               bank_open [0:NBANKS-1];
    reg [ROW_W-1:0]   bank_row  [0:NBANKS-1];
    reg [CW-1:0]      rcd_ctr   [0:NBANKS-1];  // down: ACT -> RD/WR
    reg [CW-1:0]      rp_ctr    [0:NBANKS-1];  // down: PRE -> ACT
    reg [CW-1:0]      wr_ctr    [0:NBANKS-1];  // down: write recovery
    reg [CW-1:0]      ras_ctr   [0:NBANKS-1];  // up  : since ACT, for tRAS
    reg [CW-1:0]      rc_ctr    [0:NBANKS-1];  // up  : since ACT, for tRC

    // ---- shared resources --------------------------------------------------
    reg [CW-1:0] rrd_ctr;              // down: since any ACT
    reg [CW-1:0] ccd_ctr;              // down: since any column command
    reg [CW-1:0] faw_ctr [0:3];        // rolling four-activate window
    reg [1:0]    faw_idx;
    reg [CW-1:0] ref_ctr;              // up  : since last REF
    reg [CW-1:0] rfc_ctr;              // down: REF in progress
    reg          ref_pending;

    // =====================================================================
    // Request queue. Entry 0 is the oldest; removal compacts downward, so
    // "oldest" is just a lower index and the FCFS half of FR-FCFS is free.
    // =====================================================================
    reg                q_val   [0:QDEPTH-1];
    reg                q_write [0:QDEPTH-1];
    reg [BA_W-1:0]     q_bank  [0:QDEPTH-1];
    reg [ROW_W-1:0]    q_row   [0:QDEPTH-1];
    reg [COL_W-1:0]    q_col   [0:QDEPTH-1];
    reg [DW-1:0]       q_wdata [0:QDEPTH-1];
    reg [IDW-1:0]      q_id    [0:QDEPTH-1];
    reg [31:0]         qn;

    assign req_ready = (qn < QDEPTH[31:0]) && !ref_pending && rst_n;

    wire [BA_W-1:0] req_bank_i = (NBANKS == 1) ? {BA_W{1'b0}} : req_bank;

    // =====================================================================
    // Completion pipeline: a column command issued now retires tCL later.
    // Several may be in flight at once across banks -- that overlap is the
    // whole point of having banks.
    // =====================================================================
    reg               p_val  [0:MAXCL-1];
    reg               p_read [0:MAXCL-1];
    reg [IDW-1:0]     p_id   [0:MAXCL-1];
    reg [DW-1:0]      p_data [0:MAXCL-1];

    // =====================================================================
    // Scheduling decision (combinational)
    // =====================================================================
    reg        do_col, do_act, do_pre, do_ref;
    reg [31:0] sel_col, sel_act, sel_pre;
    reg [31:0] pre_bank;

    reg all_closed;

    // Address hazard: an entry may not be reordered ahead of an OLDER entry
    // targeting the same bank/row/column. Without this, FR-FCFS would happily
    // hoist a read past a pending write to the same address and return stale
    // data -- a reordering bug that a purely timing-focused checker would
    // never see, and the reason the scoreboard checks data as well.
    reg haz [0:QDEPTH-1];
    integer j;

    // per-bank permission wires, recomputed each cycle
    reg col_ok [0:NBANKS-1];
    reg act_ok [0:NBANKS-1];
    reg pre_ok [0:NBANKS-1];

    wire rrd_ok = (rrd_ctr == CZERO);
    wire faw_ok = (faw_ctr[faw_idx] == CZERO);
    wire ccd_ok = (ccd_ctr == CZERO);
    wire ref_busy = (rfc_ctr != CZERO);

    always @(*) begin
        for (b = 0; b < NBANKS; b = b + 1) begin
            col_ok[b] = bank_open[b] && (rcd_ctr[b] == CZERO) && ccd_ok && !ref_busy;
            act_ok[b] = !bank_open[b] && (rp_ctr[b] == CZERO) &&
                        (rc_ctr[b] >= tRC[CW-1:0]) && rrd_ok && faw_ok &&
                        !ref_pending && !ref_busy;
            pre_ok[b] = bank_open[b] && (ras_ctr[b] >= tRAS[CW-1:0]) &&
                        (wr_ctr[b] == CZERO) && !ref_busy;
        end

        all_closed = 1'b1;
        for (b = 0; b < NBANKS; b = b + 1)
            if (bank_open[b] || (rp_ctr[b] != CZERO)) all_closed = 1'b0;

        sel_col = 32'hFFFF_FFFF;
        sel_act = 32'hFFFF_FFFF;
        sel_pre = 32'hFFFF_FFFF;

        for (i = 0; i < QDEPTH; i = i + 1) begin
            haz[i] = 1'b0;
            for (j = 0; j < QDEPTH; j = j + 1)
                if ((j < i) && q_val[j] && q_val[i] &&
                    (q_bank[j] == q_bank[i]) && (q_row[j] == q_row[i]) &&
                    (q_col[j] == q_col[i]))
                    haz[i] = 1'b1;
        end

        // ---- FR-FCFS pass 1: a column command that is ready RIGHT NOW ----
        // Scanning downward and keeping the last write leaves sel_col holding
        // the LOWEST matching index, i.e. "first ready, then oldest".
        for (i = QDEPTH-1; i >= 0; i = i - 1)
            if (q_val[i] && !haz[i] && col_ok[q_bank[i]] && bank_open[q_bank[i]] &&
                (bank_row[q_bank[i]] == q_row[i]))
                sel_col = i[31:0];

        // ---- pass 2: open a row for the oldest request that needs one ----
        for (i = QDEPTH-1; i >= 0; i = i - 1)
            if (q_val[i] && act_ok[q_bank[i]])
                sel_act = i[31:0];

        // ---- pass 3: close a row that is in the way ----------------------
        // Only reached when no hit is ready, so a bank is never precharged
        // out from under queued requests that could still use it.
        for (i = QDEPTH-1; i >= 0; i = i - 1)
            if (q_val[i] && bank_open[q_bank[i]] && pre_ok[q_bank[i]] &&
                (bank_row[q_bank[i]] != q_row[i]))
                sel_pre = i[31:0];

        // ---- arbitrate: exactly one command per cycle ---------------------
        do_ref = 1'b0;
        do_col = 1'b0;
        do_act = 1'b0;
        do_pre = 1'b0;
        pre_bank = 32'd0;

        if (ref_pending && !ref_busy && all_closed) begin
            do_ref = 1'b1;
        end
        else if (ref_pending) begin
            // Drain, do not stall: already-open banks may still serve row
            // hits, but no new row may be opened. The queue is finite and
            // req_ready is low, so this terminates -- that is the argument
            // that a request stream cannot starve the refresh.
            if (sel_col != 32'hFFFF_FFFF)      do_col = 1'b1;
            else if (sel_pre != 32'hFFFF_FFFF) begin
                do_pre = 1'b1; pre_bank = q_bank[sel_pre];
            end
            else begin
                // Close idle open banks that no queued request wants.
                for (b = NBANKS-1; b >= 0; b = b - 1)
                    if (pre_ok[b]) begin do_pre = 1'b1; pre_bank = b[31:0]; end
            end
        end
        else if (!ref_busy) begin
            if (sel_col != 32'hFFFF_FFFF)      do_col = 1'b1;
            else if (sel_act != 32'hFFFF_FFFF) do_act = 1'b1;
            else if (sel_pre != 32'hFFFF_FFFF) begin
                do_pre = 1'b1; pre_bank = q_bank[sel_pre];
            end
        end
    end

    // =====================================================================
    // Sequential
    // =====================================================================
    reg [31:0] rm;   // index of the queue entry being removed this cycle

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (b = 0; b < NBANKS; b = b + 1) begin
                bank_open[b] <= 1'b0;
                bank_row[b]  <= {ROW_W{1'b0}};
                rcd_ctr[b]   <= CZERO;
                rp_ctr[b]    <= CZERO;
                wr_ctr[b]    <= CZERO;
                ras_ctr[b]   <= CMAX;
                rc_ctr[b]    <= CMAX;
            end
            for (k = 0; k < 4; k = k + 1) faw_ctr[k] <= CZERO;
            for (k = 0; k < QDEPTH; k = k + 1) q_val[k] <= 1'b0;
            for (k = 0; k < MAXCL; k = k + 1) begin
                p_val[k] <= 1'b0; p_read[k] <= 1'b0;
                p_id[k]  <= {IDW{1'b0}}; p_data[k] <= {DW{1'b0}};
            end
            faw_idx     <= 2'd0;
            rrd_ctr     <= CZERO;
            ccd_ctr     <= CZERO;
            ref_ctr     <= CZERO;
            rfc_ctr     <= CZERO;
            ref_pending <= 1'b0;
            qn          <= 32'd0;
            cmd         <= CMD_NOP;
            cmd_bank    <= {BA_W{1'b0}};
            row_addr    <= {ROW_W{1'b0}};
            col_addr    <= {COL_W{1'b0}};
            wdata       <= {DW{1'b0}};
            done        <= 1'b0;
            done_id     <= {IDW{1'b0}};
            rdata       <= {DW{1'b0}};
            rdata_valid <= 1'b0;
        end else begin
            cmd         <= CMD_NOP;
            done        <= 1'b0;
            rdata_valid <= 1'b0;

            // ---- counters ---------------------------------------------------
            for (b = 0; b < NBANKS; b = b + 1) begin
                if (rcd_ctr[b] != CZERO) rcd_ctr[b] <= rcd_ctr[b] - 16'd1;
                if (rp_ctr[b]  != CZERO) rp_ctr[b]  <= rp_ctr[b]  - 16'd1;
                if (wr_ctr[b]  != CZERO) wr_ctr[b]  <= wr_ctr[b]  - 16'd1;
                if (ras_ctr[b] != CMAX)  ras_ctr[b] <= ras_ctr[b] + 16'd1;
                if (rc_ctr[b]  != CMAX)  rc_ctr[b]  <= rc_ctr[b]  + 16'd1;
            end
            if (rrd_ctr != CZERO) rrd_ctr <= rrd_ctr - 16'd1;
            if (ccd_ctr != CZERO) ccd_ctr <= ccd_ctr - 16'd1;
            if (rfc_ctr != CZERO) rfc_ctr <= rfc_ctr - 16'd1;
            for (k = 0; k < 4; k = k + 1)
                if (faw_ctr[k] != CZERO) faw_ctr[k] <= faw_ctr[k] - 16'd1;

            if (ref_ctr >= tREFI[CW-1:0]) ref_pending <= 1'b1;
            ref_ctr <= ref_ctr + 16'd1;

            // ---- completion pipeline ---------------------------------------
            for (k = MAXCL-1; k > 0; k = k - 1) begin
                p_val[k]  <= p_val[k-1];
                p_read[k] <= p_read[k-1];
                p_id[k]   <= p_id[k-1];
                p_data[k] <= p_data[k-1];
            end
            p_val[0] <= 1'b0;

            // The command bus is registered, so when a RD is *issued* the
            // address is not on the bus yet and rdata_in is still stale. The
            // read data is therefore captured one cycle later, when cmd==RD
            // is actually visible to the memory model. This override runs
            // after the shift above, so it wins for that entry.
            if (p_val[0] && p_read[0]) p_data[1] <= rdata_in;

            if (p_val[tCL-1]) begin
                done        <= 1'b1;
                done_id     <= p_id[tCL-1];
                rdata       <= p_data[tCL-1];
                rdata_valid <= p_read[tCL-1];
            end

            // ---- issue one command ------------------------------------------
            rm = 32'hFFFF_FFFF;

            if (do_ref) begin
                cmd         <= CMD_REF;
                rfc_ctr     <= tRFC[CW-1:0];
                ref_pending <= 1'b0;
                ref_ctr     <= CZERO;
                for (b = 0; b < NBANKS; b = b + 1) begin
                    ras_ctr[b] <= CMAX;
                    rc_ctr[b]  <= CMAX;
                end
            end
            else if (do_col) begin
                cmd      <= q_write[sel_col] ? CMD_WR : CMD_RD;
                cmd_bank <= q_bank[sel_col];
                col_addr <= q_col[sel_col];
                wdata    <= q_wdata[sel_col];
                ccd_ctr  <= tCCD[CW-1:0];
                if (q_write[sel_col]) wr_ctr[q_bank[sel_col]] <= tCL[CW-1:0] + tWR[CW-1:0];
                p_val[0]  <= 1'b1;
                p_read[0] <= !q_write[sel_col];
                p_id[0]   <= q_id[sel_col];
                p_data[0] <= {DW{1'b0}};   // filled in next cycle, see above
                rm = sel_col;
            end
            else if (do_act) begin
                cmd      <= CMD_ACT;
                cmd_bank <= q_bank[sel_act];
                row_addr <= q_row[sel_act];
                bank_open[q_bank[sel_act]] <= 1'b1;
                bank_row [q_bank[sel_act]] <= q_row[sel_act];
                rcd_ctr  [q_bank[sel_act]] <= tRCD[CW-1:0];
                ras_ctr  [q_bank[sel_act]] <= CZERO;
                rc_ctr   [q_bank[sel_act]] <= CZERO;
                rrd_ctr  <= tRRD[CW-1:0];
                faw_ctr[faw_idx] <= tFAW[CW-1:0];
                faw_idx  <= faw_idx + 2'd1;
            end
            else if (do_pre) begin
                cmd      <= CMD_PRE;
                cmd_bank <= pre_bank[BA_W-1:0];
                bank_open[pre_bank] <= 1'b0;
                rp_ctr   [pre_bank] <= tRP[CW-1:0];
            end

            // ---- queue maintenance ------------------------------------------
            // Compact over the removed entry, then append any new request.
            if (rm != 32'hFFFF_FFFF) begin
                for (k = 0; k < QDEPTH-1; k = k + 1) begin
                    if (k[31:0] >= rm) begin
                        q_val[k]   <= q_val[k+1];
                        q_write[k] <= q_write[k+1];
                        q_bank[k]  <= q_bank[k+1];
                        q_row[k]   <= q_row[k+1];
                        q_col[k]   <= q_col[k+1];
                        q_wdata[k] <= q_wdata[k+1];
                        q_id[k]    <= q_id[k+1];
                    end
                end
                q_val[QDEPTH-1] <= 1'b0;
            end

            if (req_valid && req_ready) begin
                // Land at qn, or qn-1 when an entry left this cycle.
                k = (rm != 32'hFFFF_FFFF) ? (qn - 32'd1) : qn;
                q_val[k]   <= 1'b1;
                q_write[k] <= req_write;
                q_bank[k]  <= req_bank_i;
                q_row[k]   <= req_row;
                q_col[k]   <= req_col;
                q_wdata[k] <= req_wdata;
                q_id[k]    <= req_id;
            end

            case ({(req_valid && req_ready), (rm != 32'hFFFF_FFFF)})
                2'b10:   qn <= qn + 32'd1;
                2'b01:   qn <= qn - 32'd1;
                default: qn <= qn;
            endcase
        end
    end

endmodule

`default_nettype wire
