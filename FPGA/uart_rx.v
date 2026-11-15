// ═══════════════════════════════════════════════════════════
//  uart_rx.v — UART Receiver (8N1)
//  קולט בייטים מה-ESP32
// ═══════════════════════════════════════════════════════════
module uart_rx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,        // קו סריאלי נכנס
    output reg  [7:0] data,      // הבייט שהתקבל
    output reg        valid      // פולס של מחזור אחד כשהבייט מוכן
);

    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;   // 434 @ 50MHz
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift;

    // ── סנכרון כפול — הגנה מפני מטא-יציבות ──
    reg rx_meta, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shift   <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;                    // ברירת מחדל — פולס יחיד

            case (state)

            // ── המתנה לביט התחלה (ירידה ל-0) ──
            S_IDLE: begin
                clk_cnt <= 16'd0;
                bit_idx <= 3'd0;
                if (rx_sync == 1'b0)
                    state <= S_START;
            end

            // ── אימות ביט ההתחלה באמצע שלו ──
            S_START: begin
                if (clk_cnt == HALF_BIT - 1) begin
                    if (rx_sync == 1'b0) begin
                        clk_cnt <= 16'd0;     // תקין → דוגמים מכאן
                        state   <= S_DATA;
                    end else begin
                        state   <= S_IDLE;    // רעש → ביטול
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ── דגימת 8 ביטי מידע (LSB ראשון) ──
            S_DATA: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    shift   <= {rx_sync, shift[7:1]};
                    if (bit_idx == 3'd7) begin
                        bit_idx <= 3'd0;
                        state   <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 3'd1;
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ── ביט עצירה ──
            S_STOP: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    data    <= shift;
                    valid   <= 1'b1;
                    state   <= S_IDLE;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            default: state <= S_IDLE;

            endcase
        end
    end

endmodule