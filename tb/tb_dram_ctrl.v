// ---------------------------------------------------------------------------
// tb_dram_ctrl.v -- self-checking testbench for the single-bank DRAM controller
//
// Three independent things live here:
//
//   1. A DRAM memory model that behaves like a real bank: it latches the row
//      at ACT and indexes with the column at RD/WR. It never looks at the
//      controller's internal open_row, so a row-tracking bug shows up as
//      corrupted data rather than being masked.
//
//   2. A timing checker that timestamps every command off the command bus and
//      re-derives tRCD / tRP / tRAS / tRC / tWR / tRFC and the refresh
//      deadline from scratch. It shares no counter with the DUT, so it can
//      disagree with it -- which is the entire point.
//
//   3. Directed tests, then a randomized stream, with statistics.
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module tb_dram_ctrl;

    // ---- must match the DUT ---------------------------------------------
    localparam integer ROW_W = 3;
    localparam integer COL_W = 4;
    localparam integer DW    = 32;

    localparam integer tRCD  = 17;
    localparam integer tRP   = 17;
    localparam integer tCL   = 17;
    localparam integer tRAS  = 39;
    localparam integer tRC   = 56;
    localparam integer tWR   = 18;
    localparam integer tRFC  = 420;
    localparam integer tREFI = 9360;

    // How late a refresh may be and still be acceptable. A refresh that
    // becomes pending mid-access must wait out the access, then tRAS/tWR,
    // then tRP before REF can issue -- roughly 70 cycles worst case.
    localparam integer REF_SLACK = 200;

    localparam [2:0] CMD_NOP = 3'd0,
                     CMD_ACT = 3'd1,
                     CMD_RD  = 3'd2,
                     CMD_WR  = 3'd3,
                     CMD_PRE = 3'd4,
                     CMD_REF = 3'd5;

    // ---- DUT connections --------------------------------------------------
    reg                clk = 1'b0;
    reg                rst_n = 1'b0;

    reg                req_valid = 1'b0;
    reg                req_write = 1'b0;
    reg  [ROW_W-1:0]   req_row   = {ROW_W{1'b0}};
    reg  [COL_W-1:0]   req_col   = {COL_W{1'b0}};
    reg  [DW-1:0]      req_wdata = {DW{1'b0}};
    wire               req_ready;
    wire               done;
    wire [DW-1:0]      rdata;
    wire               rdata_valid;

    wire [2:0]         cmd;
    wire [ROW_W-1:0]   row_addr;
    wire [COL_W-1:0]   col_addr;
    wire [DW-1:0]      wdata;
    reg  [DW-1:0]      rdata_in = {DW{1'b0}};

`ifdef BROKEN
    dram_ctrl_broken #(
`else
    dram_ctrl #(
`endif
        .ROW_W(ROW_W), .COL_W(COL_W), .DW(DW),
        .tRCD(tRCD), .tRP(tRP), .tCL(tCL), .tRAS(tRAS),
        .tRC(tRC), .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_write(req_write),
        .req_row(req_row), .req_col(req_col), .req_wdata(req_wdata),
        .req_ready(req_ready), .done(done),
        .rdata(rdata), .rdata_valid(rdata_valid),
        .cmd(cmd), .row_addr(row_addr), .col_addr(col_addr),
        .wdata(wdata), .rdata_in(rdata_in)
    );

    always #5 clk = ~clk;          // 100 MHz simulation clock

    // ---- free-running cycle counter used by every checker -----------------
    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    // =====================================================================
    // 1. DRAM memory model
    // =====================================================================
    reg [DW-1:0]    mem  [0:(1<<ROW_W)-1][0:(1<<COL_W)-1];
    reg [ROW_W-1:0] bank_row = {ROW_W{1'b0}};   // row latched by the bank at ACT
    reg [DW-1:0]    rd_captured = {DW{1'b0}};

    always @(posedge clk) begin
        // A real bank latches the row address when ACT is issued and holds it
        // until precharged. On a row hit no ACT arrives, so this simply keeps
        // the previous value -- exactly the behaviour we want to verify.
        if (cmd == CMD_ACT) bank_row <= row_addr;
        if (cmd == CMD_WR)  mem[bank_row][col_addr] <= wdata;
        if (cmd == CMD_RD)  rd_captured <= mem[bank_row][col_addr];
    end

    // Data appears on the bus tCL after RD. Only one request is ever in
    // flight, so holding the captured value is sufficient.
    always @(*) rdata_in = rd_captured;

    // =====================================================================
    // 2. Independent timing checker
    // =====================================================================
    integer errors      = 0;
    integer checks      = 0;

    integer last_act    = -100000;
    integer last_pre    = -100000;
    integer last_ref    = -100000;
    integer last_rd     = -100000;
    integer last_wr_end = -100000;   // cycle the write data burst finished

    integer n_act = 0, n_rd = 0, n_wr = 0, n_pre = 0, n_ref = 0;
    integer worst_ref_gap = 0;

    task check(input cond, input [1023:0] what, input integer got, input integer need);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] cycle %0d: %0s -- measured %0d, minimum %0d",
                         cyc, what, got, need);
            end
        end
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            case (cmd)
                CMD_ACT: begin
                    n_act = n_act + 1;
                    if (last_pre > -100000)
                        check(cyc - last_pre >= tRP,  "tRP  (PRE -> ACT)", cyc - last_pre, tRP);
                    if (last_act > -100000)
                        check(cyc - last_act >= tRC,  "tRC  (ACT -> ACT)", cyc - last_act, tRC);
                    if (last_ref > -100000)
                        check(cyc - last_ref >= tRFC, "tRFC (REF -> ACT)", cyc - last_ref, tRFC);
                    last_act = cyc;
                end

                CMD_RD: begin
                    n_rd = n_rd + 1;
                    check(cyc - last_act >= tRCD, "tRCD (ACT -> RD)", cyc - last_act, tRCD);
                    last_rd = cyc;
                end

                CMD_WR: begin
                    n_wr = n_wr + 1;
                    check(cyc - last_act >= tRCD, "tRCD (ACT -> WR)", cyc - last_act, tRCD);
                    last_wr_end = cyc + tCL;   // data burst completes tCL later
                end

                CMD_PRE: begin
                    n_pre = n_pre + 1;
                    check(cyc - last_act >= tRAS, "tRAS (ACT -> PRE)", cyc - last_act, tRAS);
                    if (last_wr_end > -100000)
                        check(cyc - last_wr_end >= tWR, "tWR  (write end -> PRE)",
                              cyc - last_wr_end, tWR);
                    last_pre = cyc;
                end

                CMD_REF: begin
                    n_ref = n_ref + 1;
                    // REF requires an idle, fully precharged bank.
                    check(dut.row_open == 1'b0, "REF issued with a row still open", 0, 1);
                    if (last_pre > -100000)
                        check(cyc - last_pre >= tRP, "tRP  (PRE -> REF)", cyc - last_pre, tRP);
                    if (last_ref > -100000) begin
                        check(cyc - last_ref >= tRFC, "tRFC (REF -> REF)", cyc - last_ref, tRFC);
                        if (cyc - last_ref > worst_ref_gap) worst_ref_gap = cyc - last_ref;
                    end
                    last_ref = cyc;
                end

                default: ;
            endcase

            // Refresh deadline: the gap between refreshes may exceed tREFI
            // only by the time needed to wind down an in-flight access.
            if (last_ref > -100000 && (cyc - last_ref) > (tREFI + REF_SLACK)) begin
                errors = errors + 1;
                $display("  [FAIL] cycle %0d: MISSED REFRESH DEADLINE -- %0d cycles since last REF (limit %0d)",
                         cyc, cyc - last_ref, tREFI + REF_SLACK);
                last_ref = cyc;   // report once per overrun, then resync
            end
        end
    end

    // =====================================================================
    // 3. Host driver, scoreboard and statistics
    // =====================================================================
    reg [DW-1:0] shadow  [0:(1<<ROW_W)-1][0:(1<<COL_W)-1];
    reg          written [0:(1<<ROW_W)-1][0:(1<<COL_W)-1];

    integer stat_en       = 0;
    integer stat_cycles   = 0;
    integer stat_refcyc   = 0;
    integer stat_reqs     = 0;
    integer stat_hits     = 0;
    integer stat_latsum   = 0;

    always @(posedge clk) begin
        if (stat_en) begin
            stat_cycles <= stat_cycles + 1;
            if (dut.state == 3'd6) stat_refcyc <= stat_refcyc + 1;  // REFRESHING
        end
    end

    integer accept_cyc = 0;
    integer last_lat   = 0;
    integer acts_at_accept = 0;
    integer was_hit    = 0;

    task do_req(input w, input [ROW_W-1:0] r, input [COL_W-1:0] c, input [DW-1:0] d);
        begin
            @(negedge clk);
            req_valid = 1'b1;
            req_write = w;
            req_row   = r;
            req_col   = c;
            req_wdata = d;

            while (req_ready !== 1'b1) @(negedge clk);
            accept_cyc     = cyc;
            acts_at_accept = n_act;

            @(negedge clk);
            req_valid = 1'b0;

            while (done !== 1'b1) @(negedge clk);
            last_lat = cyc - accept_cyc;
            was_hit  = (n_act == acts_at_accept) ? 1 : 0;

            if (stat_en) begin
                stat_reqs   = stat_reqs + 1;
                stat_latsum = stat_latsum + last_lat;
                if (was_hit) stat_hits = stat_hits + 1;
            end

            if (w) begin
                shadow[r][c]  = d;
                written[r][c] = 1'b1;
            end else if (written[r][c]) begin
                checks = checks + 1;
                if (rdata !== shadow[r][c]) begin
                    errors = errors + 1;
                    $display("  [FAIL] cycle %0d: DATA MISMATCH row %0d col %0d -- got %08h, expected %08h",
                             cyc, r, c, rdata, shadow[r][c]);
                end
            end
        end
    endtask

    // ---- test sequence ----------------------------------------------------
    integer i, r, c, w;
    integer seed = 32'hC0FFEE;
    integer pass_mark;

    initial begin
        $dumpfile("sim/dram_ctrl.vcd");
        $dumpvars(0, tb_dram_ctrl);

        for (r = 0; r < (1<<ROW_W); r = r + 1)
            for (c = 0; c < (1<<COL_W); c = c + 1)
                written[r][c] = 1'b0;

        $display("");
        $display("=====================================================");
        $display(" DRAM controller -- self-checking testbench");
        $display(" tRCD=%0d tRP=%0d tCL=%0d tRAS=%0d tRC=%0d tWR=%0d tRFC=%0d tREFI=%0d",
                 tRCD, tRP, tCL, tRAS, tRC, tWR, tRFC, tREFI);
        $display("=====================================================");

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        // ---- Test 1: single write then read back -------------------------
        $display("");
        $display("-- Test 1: write then read back");
        pass_mark = errors;
        do_req(1'b1, 3'd3, 4'd5, 32'hDEAD_BEEF);
        do_req(1'b0, 3'd3, 4'd5, 32'h0);
        $display("   read returned %08h, latency %0d cycles, row %0s",
                 rdata, last_lat, was_hit ? "HIT" : "MISS");
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- Test 2: row hit skips ACT -----------------------------------
        $display("");
        $display("-- Test 2: row hit (same row, different column)");
        pass_mark = errors;
        acts_at_accept = n_act;
        do_req(1'b1, 3'd3, 4'd9, 32'h1234_5678);
        if (!was_hit) begin
            errors = errors + 1;
            $display("  [FAIL] expected a row hit, but an ACT was issued");
        end
        $display("   latency %0d cycles, row %0s (no ACT expected)",
                 last_lat, was_hit ? "HIT" : "MISS");
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- Test 3: row miss forces PRE then ACT ------------------------
        $display("");
        $display("-- Test 3: row miss (different row)");
        pass_mark = errors;
        i = n_pre;
        do_req(1'b1, 3'd6, 4'd2, 32'hA5A5_A5A5);
        if (was_hit) begin
            errors = errors + 1;
            $display("  [FAIL] expected a row miss, but no ACT was issued");
        end
        if (n_pre <= i) begin
            errors = errors + 1;
            $display("  [FAIL] row miss did not issue a PRE");
        end
        $display("   latency %0d cycles, row %0s, PRE issued: %0s",
                 last_lat, was_hit ? "HIT" : "MISS", (n_pre > i) ? "yes" : "no");
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- Test 4: back-to-back requests --------------------------------
        $display("");
        $display("-- Test 4: 8 back-to-back requests to the open row");
        pass_mark = errors;
        for (i = 0; i < 8; i = i + 1)
            do_req(1'b1, 3'd6, i[COL_W-1:0], 32'h1000_0000 + i);
        for (i = 0; i < 8; i = i + 1)
            do_req(1'b0, 3'd6, i[COL_W-1:0], 32'h0);
        $display("   16 requests completed, ACT count now %0d", n_act);
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- Test 5: refresh arriving mid-access --------------------------
        // Push the DUT's refresh interval counter close to expiry, then start
        // an access so the refresh becomes pending while the bank is busy.
        $display("");
        $display("-- Test 5: refresh arriving mid-access");
        pass_mark = errors;
        i = n_ref;
        @(negedge clk);
        dut.ref_ctr = tREFI[15:0] - 16'd3;
        do_req(1'b0, 3'd6, 4'd4, 32'h0);      // in flight when refresh fires
        do_req(1'b1, 3'd1, 4'd7, 32'hFEED_FACE);
        do_req(1'b0, 3'd1, 4'd7, 32'h0);
        if (n_ref <= i) begin
            errors = errors + 1;
            $display("  [FAIL] refresh never issued after tREFI expired");
        end
        $display("   REF commands issued: %0d", n_ref - i);
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- Test 6: randomized stream -------------------------------------
        $display("");
        $display("-- Test 6: 500 randomized requests");
        pass_mark = errors;
        stat_en = 1;
        for (i = 0; i < 500; i = i + 1) begin
            r = {$random(seed)} % (1<<ROW_W);
            c = {$random(seed)} % (1<<COL_W);
            w = {$random(seed)} % 2;
            do_req(w[0], r[ROW_W-1:0], c[COL_W-1:0], 32'h5000_0000 + i);
        end
        stat_en = 0;
        $display("   %0s", (errors == pass_mark) ? "PASS" : "FAIL");

        // ---- results --------------------------------------------------------
        $display("");
        $display("=====================================================");
        $display(" RESULTS (randomized stream of %0d requests)", stat_reqs);
        $display("=====================================================");
        $display("  row hit rate            : %0d / %0d  = %0d.%0d %%",
                 stat_hits, stat_reqs,
                 (stat_hits*100)/stat_reqs,
                 ((stat_hits*1000)/stat_reqs) % 10);
        $display("  average latency         : %0d.%0d cycles",
                 stat_latsum/stat_reqs,
                 ((stat_latsum*10)/stat_reqs) % 10);
        $display("  cycles simulated        : %0d", stat_cycles);
        $display("  cycles refreshing       : %0d", stat_refcyc);
        $display("  time spent refreshing   : %0d.%0d %%",
                 (stat_refcyc*100)/stat_cycles,
                 ((stat_refcyc*1000)/stat_cycles) % 10);
        $display("  worst REF-to-REF gap    : %0d cycles (tREFI = %0d)",
                 worst_ref_gap, tREFI);
        $display("");
        $display("  commands: ACT=%0d RD=%0d WR=%0d PRE=%0d REF=%0d",
                 n_act, n_rd, n_wr, n_pre, n_ref);
        $display("");
        $display("=====================================================");
        $display("  timing + data checks run : %0d", checks);
        $display("  failures                 : %0d", errors);
        if (errors == 0)
            $display("  OVERALL: PASS");
        else
            $display("  OVERALL: FAIL");
        $display("=====================================================");
        $display("");

        $finish;
    end

    // Safety net so a hung handshake cannot run forever.
    initial begin
        #20_000_000;
        $display("  [FAIL] TIMEOUT -- simulation did not complete");
        $finish;
    end

endmodule

`default_nettype wire
