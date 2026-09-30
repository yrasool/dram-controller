// ---------------------------------------------------------------------------
// tb_dram_ctrl_mb.v -- self-checking testbench for the multi-bank controller
//
// Same philosophy as the single-bank testbench: the checker timestamps every
// command off the bus and re-derives each rule itself. It now tracks state
// PER BANK, and adds the three constraints that only exist once banks can
// race each other: tRRD, tFAW and tCCD.
//
// The driver keeps many requests outstanding, because a controller with banks
// and a queue cannot be exercised by a one-at-a-time handshake -- an in-order
// driver would hide exactly the parallelism we built.
//
// Compile-time configuration (see Makefile):
//   -DNB=<banks>  -DQD=<queue depth>
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

`ifndef NB
  `define NB 4
`endif
`ifndef QD
  `define QD 8
`endif
`ifndef NREQ
  `define NREQ 500
`endif

module tb_dram_ctrl_mb;

    localparam integer NBANKS = `NB;
    localparam integer QDEPTH = `QD;
    localparam integer NREQ   = `NREQ;

    localparam integer ROW_W = 3;
    localparam integer COL_W = 4;
    localparam integer DW    = 32;
    localparam integer IDW   = 16;

    localparam integer tRCD  = 17, tRP  = 17, tCL   = 17, tRAS = 39;
    localparam integer tRC   = 56, tWR  = 18, tRFC  = 420, tREFI = 9360;
    localparam integer tRRD  = 6,  tFAW = 26, tCCD  = 4;

    localparam integer REF_SLACK = 400;

    localparam [2:0] CMD_NOP = 3'd0, CMD_ACT = 3'd1, CMD_RD = 3'd2,
                     CMD_WR  = 3'd3, CMD_PRE = 3'd4, CMD_REF = 3'd5;

    function integer clog2b;
        input integer v;
        integer i;
        begin
            clog2b = 1;
            for (i = 1; (1 << i) < v; i = i + 1) clog2b = i + 1;
        end
    endfunction
    localparam integer BA_W = clog2b(NBANKS);

    // ---- DUT connections ---------------------------------------------------
    reg                clk = 1'b0;
    reg                rst_n = 1'b0;
    reg                req_valid = 1'b0;
    reg                req_write = 1'b0;
    reg  [BA_W-1:0]    req_bank  = {BA_W{1'b0}};
    reg  [ROW_W-1:0]   req_row   = {ROW_W{1'b0}};
    reg  [COL_W-1:0]   req_col   = {COL_W{1'b0}};
    reg  [DW-1:0]      req_wdata = {DW{1'b0}};
    reg  [IDW-1:0]     req_id    = {IDW{1'b0}};
    wire               req_ready;
    wire               done;
    wire [IDW-1:0]     done_id;
    wire [DW-1:0]      rdata;
    wire               rdata_valid;
    wire [2:0]         cmd;
    wire [BA_W-1:0]    cmd_bank;
    wire [ROW_W-1:0]   row_addr;
    wire [COL_W-1:0]   col_addr;
    wire [DW-1:0]      wdata;
    wire [DW-1:0]      rdata_in;

    dram_ctrl_mb #(
        .NBANKS(NBANKS), .QDEPTH(QDEPTH), .ROW_W(ROW_W), .COL_W(COL_W),
        .DW(DW), .IDW(IDW),
        .tRCD(tRCD), .tRP(tRP), .tCL(tCL), .tRAS(tRAS), .tRC(tRC),
        .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI),
        .tRRD(tRRD), .tFAW(tFAW), .tCCD(tCCD)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_write(req_write), .req_bank(req_bank),
        .req_row(req_row), .req_col(req_col), .req_wdata(req_wdata),
        .req_id(req_id), .req_ready(req_ready),
        .done(done), .done_id(done_id), .rdata(rdata), .rdata_valid(rdata_valid),
        .cmd(cmd), .cmd_bank(cmd_bank), .row_addr(row_addr),
        .col_addr(col_addr), .wdata(wdata), .rdata_in(rdata_in)
    );

    always #5 clk = ~clk;

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    integer errors = 0;
    integer checks = 0;

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

    // =====================================================================
    // 1. Multi-bank memory model
    // =====================================================================
    reg [DW-1:0]    mem [0:NBANKS-1][0:(1<<ROW_W)-1][0:(1<<COL_W)-1];
    reg [ROW_W-1:0] tb_row [0:NBANKS-1];    // row each bank latched at ACT
    reg             tb_open [0:NBANKS-1];

    integer bi, ri, ci;

    always @(posedge clk) begin
        if (cmd == CMD_ACT) begin
            tb_row[cmd_bank]  <= row_addr;
            tb_open[cmd_bank] <= 1'b1;
        end
        if (cmd == CMD_PRE) tb_open[cmd_bank] <= 1'b0;
        if (cmd == CMD_REF)
            for (bi = 0; bi < NBANKS; bi = bi + 1) tb_open[bi] <= 1'b0;
        if (cmd == CMD_WR) mem[cmd_bank][tb_row[cmd_bank]][col_addr] <= wdata;
    end

    // Combinational read path: valid during the cycle the RD is on the bus.
    assign rdata_in = mem[cmd_bank][tb_row[cmd_bank]][col_addr];

    // =====================================================================
    // 2. Independent timing checker
    // =====================================================================
    integer last_act_b  [0:NBANKS-1];
    integer last_pre_b  [0:NBANKS-1];
    integer last_wrend_b[0:NBANKS-1];

    integer last_act_any = -100000;
    integer last_col     = -100000;
    integer last_ref     = -100000;

    integer faw_hist [0:3];       // cycles of the last four ACTs
    integer faw_p = 0;

    integer n_act = 0, n_rd = 0, n_wr = 0, n_pre = 0, n_ref = 0;
    integer worst_ref_gap = 0;
    integer max_faw_seen = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            case (cmd)
                CMD_ACT: begin
                    n_act = n_act + 1;
                    if (last_pre_b[cmd_bank] > -100000)
                        check(cyc - last_pre_b[cmd_bank] >= tRP, "tRP  (PRE -> ACT, same bank)",
                              cyc - last_pre_b[cmd_bank], tRP);
                    if (last_act_b[cmd_bank] > -100000)
                        check(cyc - last_act_b[cmd_bank] >= tRC, "tRC  (ACT -> ACT, same bank)",
                              cyc - last_act_b[cmd_bank], tRC);
                    if (last_ref > -100000)
                        check(cyc - last_ref >= tRFC, "tRFC (REF -> ACT)", cyc - last_ref, tRFC);
                    // inter-bank
                    if (last_act_any > -100000)
                        check(cyc - last_act_any >= tRRD, "tRRD (ACT -> ACT, any bank)",
                              cyc - last_act_any, tRRD);
                    // tFAW: this ACT plus the previous three must not all fall
                    // inside one rolling window.
                    if (faw_hist[faw_p] > -100000)
                        check(cyc - faw_hist[faw_p] >= tFAW, "tFAW (5th ACT inside window)",
                              cyc - faw_hist[faw_p], tFAW);
                    if (faw_hist[faw_p] > -100000 && (cyc - faw_hist[faw_p]) > max_faw_seen)
                        max_faw_seen = cyc - faw_hist[faw_p];

                    faw_hist[faw_p] = cyc;
                    faw_p = (faw_p + 1) % 4;
                    last_act_b[cmd_bank] = cyc;
                    last_act_any = cyc;
                end

                CMD_RD, CMD_WR: begin
                    if (cmd == CMD_RD) n_rd = n_rd + 1; else n_wr = n_wr + 1;
                    check(cyc - last_act_b[cmd_bank] >= tRCD, "tRCD (ACT -> RD/WR)",
                          cyc - last_act_b[cmd_bank], tRCD);
                    check(tb_open[cmd_bank] === 1'b1, "column command to a closed bank", 0, 1);
                    if (last_col > -100000)
                        check(cyc - last_col >= tCCD, "tCCD (column -> column)",
                              cyc - last_col, tCCD);
                    last_col = cyc;
                    if (cmd == CMD_WR) last_wrend_b[cmd_bank] = cyc + tCL;
                end

                CMD_PRE: begin
                    n_pre = n_pre + 1;
                    check(cyc - last_act_b[cmd_bank] >= tRAS, "tRAS (ACT -> PRE)",
                          cyc - last_act_b[cmd_bank], tRAS);
                    if (last_wrend_b[cmd_bank] > -100000)
                        check(cyc - last_wrend_b[cmd_bank] >= tWR, "tWR  (write end -> PRE)",
                              cyc - last_wrend_b[cmd_bank], tWR);
                    last_pre_b[cmd_bank] = cyc;
                end

                CMD_REF: begin
                    n_ref = n_ref + 1;
                    for (bi = 0; bi < NBANKS; bi = bi + 1)
                        check(tb_open[bi] === 1'b0, "REF issued with a row still open", 0, 1);
                    if (last_ref > -100000) begin
                        check(cyc - last_ref >= tRFC, "tRFC (REF -> REF)", cyc - last_ref, tRFC);
                        if (cyc - last_ref > worst_ref_gap) worst_ref_gap = cyc - last_ref;
                    end
                    last_ref = cyc;
                end

                default: ;
            endcase

            if (last_ref > -100000 && (cyc - last_ref) > (tREFI + REF_SLACK)) begin
                errors = errors + 1;
                $display("  [FAIL] cycle %0d: MISSED REFRESH DEADLINE -- %0d cycles since REF (limit %0d)",
                         cyc, cyc - last_ref, tREFI + REF_SLACK);
                last_ref = cyc;
            end
        end
    end

    // =====================================================================
    // 3. Driver + scoreboard
    // =====================================================================
    localparam integer MAXO = 4096;
    reg            o_val   [0:MAXO-1];
    reg            o_read  [0:MAXO-1];
    reg [DW-1:0]   o_exp   [0:MAXO-1];
    integer        o_start [0:MAXO-1];

    reg [DW-1:0] shadow  [0:NBANKS-1][0:(1<<ROW_W)-1][0:(1<<COL_W)-1];
    reg          written [0:NBANKS-1][0:(1<<ROW_W)-1][0:(1<<COL_W)-1];

    integer issued = 0, retired = 0;
    integer latsum = 0, worst_lat = 0;
    integer start_cyc = 0, end_cyc = 0;

    // Completion monitor: retirement is out of order, so it is matched by id.
    always @(posedge clk) begin
        if (rst_n && done) begin
            if (!o_val[done_id]) begin
                errors = errors + 1;
                $display("  [FAIL] cycle %0d: completion for unknown id %0d", cyc, done_id);
            end else begin
                checks  = checks + 1;
                latsum  = latsum + (cyc - o_start[done_id]);
                if ((cyc - o_start[done_id]) > worst_lat) worst_lat = cyc - o_start[done_id];
                if (o_read[done_id] && (rdata !== o_exp[done_id])) begin
                    errors = errors + 1;
                    $display("  [FAIL] cycle %0d: DATA MISMATCH id %0d -- got %08h, expected %08h",
                             cyc, done_id, rdata, o_exp[done_id]);
                end
                o_val[done_id] = 1'b0;
                retired = retired + 1;
            end
        end
    end

    // Pre-generated address stream, identical across every configuration so
    // the comparison isolates the controller, not the workload.
    reg [3:0]  s_bank [0:NREQ-1];
    reg [2:0]  s_row  [0:NREQ-1];
    reg [3:0]  s_col  [0:NREQ-1];
    reg        s_wr   [0:NREQ-1];

    integer seed = 32'hC0FFEE;
    integer n, hits_expected;

    integer stat_refcyc = 0;
    integer stat_on = 0;
    always @(posedge clk) if (stat_on && dut.rfc_ctr != 0) stat_refcyc <= stat_refcyc + 1;

    initial begin
`ifdef VCD
        $dumpfile("sim/dram_ctrl_mb.vcd");
        $dumpvars(0, tb_dram_ctrl_mb);
`endif
        for (bi = 0; bi < NBANKS; bi = bi + 1) begin
            last_act_b[bi]   = -100000;
            last_pre_b[bi]   = -100000;
            last_wrend_b[bi] = -100000;
            tb_open[bi]      = 1'b0;
            tb_row[bi]       = 0;
            for (ri = 0; ri < (1<<ROW_W); ri = ri + 1)
                for (ci = 0; ci < (1<<COL_W); ci = ci + 1) written[bi][ri][ci] = 1'b0;
        end
        for (n = 0; n < 4; n = n + 1) faw_hist[n] = -100000;
        for (n = 0; n < MAXO; n = n + 1) o_val[n] = 1'b0;

        // identical stream for every config
        for (n = 0; n < NREQ; n = n + 1) begin
            s_bank[n] = {$random(seed)} % NBANKS;
            s_row [n] = {$random(seed)} % (1<<ROW_W);
            s_col [n] = {$random(seed)} % (1<<COL_W);
            s_wr  [n] = ({$random(seed)} % 2);
        end

        $display("");
        $display("=====================================================");
        $display(" MULTI-BANK DRAM controller -- NBANKS=%0d  QDEPTH=%0d", NBANKS, QDEPTH);
        $display(" tRRD=%0d tFAW=%0d tCCD=%0d  (inter-bank constraints)", tRRD, tFAW, tCCD);
        $display("=====================================================");

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        stat_on   = 1;
        start_cyc = cyc;

        // Push requests as fast as the controller will take them.
        for (n = 0; n < NREQ; n = n + 1) begin
            @(negedge clk);
            while (req_ready !== 1'b1) @(negedge clk);
            req_valid = 1'b1;
            req_write = s_wr[n];
            req_bank  = s_bank[n][BA_W-1:0];
            req_row   = s_row[n];
            req_col   = s_col[n];
            req_wdata = 32'h5000_0000 + n;
            req_id    = n[IDW-1:0];

            o_val[n]   = 1'b1;
            o_read[n]  = !s_wr[n];
            o_start[n] = cyc;
            if (s_wr[n]) begin
                shadow [s_bank[n][BA_W-1:0]][s_row[n]][s_col[n]] = 32'h5000_0000 + n;
                written[s_bank[n][BA_W-1:0]][s_row[n]][s_col[n]] = 1'b1;
                o_exp[n] = 32'h5000_0000 + n;
            end else begin
                o_exp[n] = written[s_bank[n][BA_W-1:0]][s_row[n]][s_col[n]]
                           ? shadow[s_bank[n][BA_W-1:0]][s_row[n]][s_col[n]]
                           : 32'hXXXX_XXXX;
            end
            issued = issued + 1;
            @(negedge clk);
            req_valid = 1'b0;
        end

        // drain
        while (retired < NREQ) @(negedge clk);
        end_cyc = cyc;
        stat_on = 0;

        $display("");
        $display("  requests               : %0d", NREQ);
        $display("  total cycles           : %0d", end_cyc - start_cyc);
        $display("  throughput             : %0d.%0d cycles/request",
                 (end_cyc - start_cyc)/NREQ, (((end_cyc - start_cyc)*10)/NREQ) % 10);
        $display("  average latency        : %0d.%0d cycles",
                 latsum/NREQ, ((latsum*10)/NREQ) % 10);
        $display("  worst latency          : %0d cycles", worst_lat);
        $display("  row hit rate           : %0d / %0d = %0d.%0d %%",
                 NREQ - n_act, NREQ,
                 ((NREQ - n_act)*100)/NREQ, (((NREQ - n_act)*1000)/NREQ) % 10);
        $display("  time refreshing        : %0d.%0d %%",
                 (stat_refcyc*100)/(end_cyc-start_cyc),
                 ((stat_refcyc*1000)/(end_cyc-start_cyc)) % 10);
        $display("  commands               : ACT=%0d RD=%0d WR=%0d PRE=%0d REF=%0d",
                 n_act, n_rd, n_wr, n_pre, n_ref);
        $display("  worst REF-to-REF gap   : %0d (tREFI=%0d)", worst_ref_gap, tREFI);
        $display("");
        $display("  checks run             : %0d", checks);
        $display("  failures               : %0d", errors);
        $display("  OVERALL: %0s", (errors == 0) ? "PASS" : "FAIL");
        $display("=====================================================");
        $display("");
        $display("CSV,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                 NBANKS, QDEPTH, end_cyc - start_cyc, latsum/NREQ, worst_lat,
                 NREQ - n_act, errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("  [FAIL] TIMEOUT -- issued %0d retired %0d", issued, retired);
        $finish;
    end

endmodule

`default_nettype wire
