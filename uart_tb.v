`timescale 1ns/1ps
module uart_tb;

    localparam CLK_PERIOD    = 20;
    localparam CLKS_PER_BIT  = 5208;
    localparam CLKS_PER_TICK = 325;

    reg        clk, rst_n;
    reg        tx_valid;
    reg  [7:0] tx_data;
    wire       tx_line, tx_busy;
    wire [7:0] rx_data;
    wire       rx_valid, rx_busy;
    wire       frame_error, overrun_error;
    reg        rx_ready;

    uart #(.CLKS_PER_BIT(CLKS_PER_BIT),.CLKS_PER_TICK(CLKS_PER_TICK)) dut (
        .clk(clk),.rst_n(rst_n),
        .tx_valid(tx_valid),.tx_data(tx_data),.tx(tx_line),.tx_busy(tx_busy),
        .rx(tx_line),.rx_data(rx_data),.rx_valid(rx_valid),.rx_ready(rx_ready),
        .frame_error(frame_error),.overrun_error(overrun_error),.rx_busy(rx_busy)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer errors = 0;

    // Wait up to 14 bit-periods for rx_valid
    task wait_rx;
        output found;
        integer i;
        begin
            found = 0;
            for (i = 0; i < CLKS_PER_BIT*14 && !found; i = i+1) begin
                @(posedge clk);
                if (rx_valid) found = 1;
            end
        end
    endtask

    task send_byte;
        input [7:0] data;
        begin
            @(posedge clk); #1;
            tx_data = data; tx_valid = 1;
            @(posedge clk); #1; tx_valid = 0;
        end
    endtask

    task check_rx;
        input [7:0] expected;
        input do_ack;
        reg found;
        begin
            wait_rx(found);
            if (!found) begin
                $display("  FAIL: rx_valid never asserted (expected 0x%02h)", expected);
                errors = errors + 1;
            end else if (rx_data !== expected) begin
                $display("  FAIL: expected 0x%02h got 0x%02h", expected, rx_data);
                errors = errors + 1;
            end else begin
                $display("  PASS: received 0x%02h", rx_data);
            end
            if (do_ack) begin
                rx_ready = 1; @(posedge clk); #1; rx_ready = 0;
            end
            repeat (CLKS_PER_BIT*2) @(posedge clk);
        end
    endtask

    integer i;
    initial begin
        rst_n=0; tx_valid=0; tx_data=0; rx_ready=0;
        repeat(20) @(posedge clk); rst_n=1;
        repeat(5)  @(posedge clk);

        // ── Test 1: Basic correctness ─────────────────────────
        $display("\n[TEST 1] Basic byte transfers");
        send_byte(8'h55); check_rx(8'h55, 1);
        send_byte(8'hA3); check_rx(8'hA3, 1);
        send_byte(8'hFF); check_rx(8'hFF, 1);
        send_byte(8'h00); check_rx(8'h00, 1);

        // ── Test 2: Overrun detection ─────────────────────────
        $display("\n[TEST 2] Overrun detection (no rx_ready between frames)");
        send_byte(8'hAA);
        // Do NOT ack first byte
        check_rx(8'hAA, 0);   // receive but don't ack
        send_byte(8'h55);     // send second before ack
        // Wait for overrun_error
        begin : chk_overrun
            reg found_overrun;
            found_overrun = 0;
            for (i = 0; i < CLKS_PER_BIT*14 && !found_overrun; i = i+1) begin
                @(posedge clk);
                if (overrun_error) begin
                    found_overrun = 1;
                    $display("  PASS: overrun_error asserted as expected");
                end
            end
            if (!found_overrun) begin
                $display("  FAIL: overrun_error was not asserted");
                errors = errors + 1;
            end
        end
        rx_ready = 1; @(posedge clk); #1; rx_ready = 0;
        repeat (CLKS_PER_BIT*3) @(posedge clk);

        // ── Summary ───────────────────────────────────────────
        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);
        $finish;
    end

endmodule
