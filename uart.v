
module uart_tx #(
    parameter integer CLKS_PER_BIT = 5208   // 50 MHz / 9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_valid,   // 1-cycle pulse: load tx_data and start TX
    input  wire [7:0] tx_data,
    // Serial output
    output reg        tx,         // idle = 1
    output reg        tx_busy     // high while a frame is in flight
);

    // ── Counter width ──────────────────────────────────────────────────────
    localparam integer CNT_W = $clog2(CLKS_PER_BIT);

    // ── State encoding ─────────────────────────────────────────────────────
    localparam [1:0] TX_IDLE  = 2'b00,
                     TX_START = 2'b01,
                     TX_DATA  = 2'b10,
                     TX_STOP  = 2'b11;

    reg [1:0]       state;
    reg [CNT_W-1:0] clk_cnt;
    reg [2:0]       bit_idx;   // 0-7, which data bit is being sent
    reg [7:0]       tx_shift;  // parallel-to-serial shift register

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= TX_IDLE;
            clk_cnt  <= {CNT_W{1'b0}};
            bit_idx  <= 3'd0;
            tx       <= 1'b1;   // idle high
            tx_busy  <= 1'b0;
            tx_shift <= 8'd0;
        end else begin
            case (state)

                TX_IDLE: begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                    clk_cnt <= {CNT_W{1'b0}};
                    bit_idx <= 3'd0;
                    if (tx_valid) begin
                        tx_shift <= tx_data;  // latch data at start of frame
                        tx_busy  <= 1'b1;
                        state    <= TX_START;
                    end
                end

                TX_START: begin
                    tx <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        state   <= TX_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                TX_DATA: begin
                    // LSB first (UART convention)
                    tx <= tx_shift[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= TX_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                TX_STOP: begin
                    // Stop bit (logic 1).  After this the line returns idle.
                    tx <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        tx_busy <= 1'b0;
                        state   <= TX_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= TX_IDLE;
                end
            endcase
        end
    end

endmodule





module uart_rx #(
    // 50_000_000 / (9600 × 16) = 325.52 → truncated to 325
    parameter integer CLKS_PER_TICK = 325,
    parameter integer OVERSAMPLE    = 16    // must be power-of-two
)(
    input  wire       clk,
    input  wire       rst_n,
    // Serial input (asynchronous – double-flopped inside this module)
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid,     // 1-cycle pulse: rx_data contains new byte
    input  wire       rx_ready,     // 1-cycle pulse from host: byte consumed
    // Status outputs (all 1-cycle pulses unless noted)
    output reg        frame_error,
    output reg        overrun_error,
    output reg        rx_busy
);

    // ── Derived parameters ──────────────────────────────────────────────────
    localparam integer TICK_CNT_W = $clog2(CLKS_PER_TICK);
    localparam integer TICK_MAX   = OVERSAMPLE - 1;          // 15

    // Majority-vote sample points (centre at tick 8 = exactly 50% of bit)
    localparam integer SAMP_EARLY = 7;
    localparam integer SAMP_MID   = 8;
    localparam integer SAMP_LATE  = 9;

    // ── 2-FF synchroniser ──────────────────────────────────────────────────
    reg rx_s0, rx_s1, rx_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s0   <= 1'b1;   // idle state of UART line is high
            rx_s1   <= 1'b1;
            rx_prev <= 1'b1;
        end else begin
            rx_s0   <= rx;
            rx_s1   <= rx_s0;
            rx_prev <= rx_s1;  // one cycle behind rx_s1
        end
    end


    wire start_edge = rx_prev & ~rx_s1;

    // ── FSM state ────────────────────────────────────────────────────────────
    // Declared here (before the tick generator) so the tick generator's
    localparam [1:0] RX_IDLE  = 2'b00,
                     RX_START = 2'b01,
                     RX_DATA  = 2'b10,
                     RX_STOP  = 2'b11;

    reg [1:0] state;


    reg [TICK_CNT_W-1:0] clk_cnt;
    reg                  tick;     // 1-cycle pulse every CLKS_PER_TICK clocks

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= {TICK_CNT_W{1'b0}};
            tick    <= 1'b0;
        end else begin
            tick <= 1'b0;  // default: deasserted
            if (sync_reset) begin
                // Phase-align tick counter to falling edge of start bit.
                clk_cnt <= {TICK_CNT_W{1'b0}};
            end else if (clk_cnt == CLKS_PER_TICK - 1) begin
                clk_cnt <= {TICK_CNT_W{1'b0}};
                tick    <= 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end
    end

    // ── FSM data path ────────────────────────────────────────────────────────
    reg [3:0] tick_cnt;   // 0..TICK_MAX within each bit period
    reg [2:0] bit_idx;    // 0..7 data bit index
    reg [7:0] rx_shift;   // serial-to-parallel shift register

    // Majority-vote sample registers
    reg samp_early, samp_mid, samp_late;


    wire majority = (samp_early & samp_mid)
                  | (samp_early & samp_late)
                  | (samp_mid  & samp_late);

    // Overrun tracking: set when rx_valid asserted, cleared on rx_ready
    reg byte_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= RX_IDLE;
            tick_cnt     <= 4'd0;
            bit_idx      <= 3'd0;
            rx_shift     <= 8'd0;
            rx_data      <= 8'd0;
            rx_valid     <= 1'b0;
            frame_error  <= 1'b0;
            overrun_error<= 1'b0;
            rx_busy      <= 1'b0;
            byte_pending <= 1'b0;
            samp_early   <= 1'b0;
            samp_mid     <= 1'b0;
            samp_late    <= 1'b0;
        end else begin
            // ── Default pulse signals (deassert each cycle) ──────────────
            rx_valid      <= 1'b0;
            frame_error   <= 1'b0;
            overrun_error <= 1'b0;

            // ── Overrun handshake ────────────────────────────────────────
            // byte_pending tracks whether the host has consumed rx_data.
            if (rx_valid)
                byte_pending <= 1'b1;
            if (rx_ready)
                byte_pending <= 1'b0;

            case (state)

                // ──────────────────────────────────────────────────────────
                RX_IDLE: begin
                    rx_busy  <= 1'b0;
                    tick_cnt <= 4'd0;
                    bit_idx  <= 3'd0;
                    // Wait for falling edge (start of start bit).
                    // zeroed clk_cnt this same cycle.
                    if (start_edge) begin
                        rx_busy <= 1'b1;
                        state   <= RX_START;
                    end
                end


                RX_START: begin
                    if (tick) begin
                        tick_cnt <= tick_cnt + 1'b1;

                        if (tick_cnt == 4'd7) samp_early <= rx_s1;
                        if (tick_cnt == 4'd8) samp_mid   <= rx_s1;
                        if (tick_cnt == 4'd9) samp_late  <= rx_s1;

                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            if (!majority) begin
                                // Start bit confirmed (line was low at centre)
                                state <= RX_DATA;
                            end else begin
                                // Glitch: line returned high — abort
                                rx_busy <= 1'b0;
                                state   <= RX_IDLE;
                            end
                        end
                    end
                end


                RX_DATA: begin
                    if (tick) begin
                        tick_cnt <= tick_cnt + 1'b1;

                        if (tick_cnt == 4'd7) samp_early <= rx_s1;
                        if (tick_cnt == 4'd8) samp_mid   <= rx_s1;
                        if (tick_cnt == 4'd9) samp_late  <= rx_s1;

                        if (tick_cnt == 4'd15) begin
                            // majority is stable here: all three samples
                            // were registered at least 6 ticks ago.
                            tick_cnt             <= 4'd0;
                            rx_shift[bit_idx]    <= majority;
                            if (bit_idx == 3'd7) begin
                                bit_idx <= 3'd0;
                                state   <= RX_STOP;
                            end else begin
                                bit_idx <= bit_idx + 1'b1;
                            end
                        end
                    end
                end


                RX_STOP: begin
                    if (tick) begin
                        tick_cnt <= tick_cnt + 1'b1;

                        if (tick_cnt == 4'd7) samp_early <= rx_s1;
                        if (tick_cnt == 4'd8) samp_mid   <= rx_s1;
                        if (tick_cnt == 4'd9) samp_late  <= rx_s1;

                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            rx_busy  <= 1'b0;
                            state    <= RX_IDLE;

                            if (majority) begin
                                // Valid stop bit — output the byte
                                rx_data  <= rx_shift;
                                rx_valid <= 1'b1;
                                // Flag overrun if previous byte was not read
                                if (byte_pending)
                                    overrun_error <= 1'b1;
                            end else begin
                                // Stop bit missing → frame error; discard byte
                                frame_error <= 1'b1;
                            end
                        end
                    end
                end

                default: begin
                    state   <= RX_IDLE;
                    rx_busy <= 1'b0;
                end
            endcase
        end
    end

endmodule


// ============================================================================
// Top-level UART wrapper
// ============================================================================
module uart #(
    parameter integer CLKS_PER_BIT  = 5208,   // TX bit clock
    parameter integer CLKS_PER_TICK = 325      // RX oversampling tick
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    output wire       tx,
    output wire       tx_busy,
    input  wire       rx,
    output wire [7:0] rx_data,
    output wire       rx_valid,
    input  wire       rx_ready,
    // RX error / status
    output wire       frame_error,
    output wire       overrun_error,
    output wire       rx_busy
);

    uart_tx #(
        .CLKS_PER_BIT (CLKS_PER_BIT)
    ) u_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_valid (tx_valid),
        .tx_data  (tx_data),
        .tx       (tx),
        .tx_busy  (tx_busy)
    );

    uart_rx #(
        .CLKS_PER_TICK (CLKS_PER_TICK)
    ) u_rx (
        .clk          (clk),
        .rst_n        (rst_n),
        .rx           (rx),
        .rx_data      (rx_data),
        .rx_valid     (rx_valid),
        .rx_ready     (rx_ready),
        .frame_error  (frame_error),
        .overrun_error(overrun_error),
        .rx_busy      (rx_busy)
    );

endmodule
