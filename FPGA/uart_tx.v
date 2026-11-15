// ═══════════════════════════════════════════════════════════
//   uart_tx.v — UART Transmitter (8N1)
//   משדר בייטים ל-Matek F405 (חבילות MSP)
// ═══════════════════════════════════════════════════════════
module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,      // הבייט לשליחה
    input  wire       send,      // פולס של מחזור אחד = התחל שידור
    output reg        tx,        // קו סריאלי יוצא
    output reg        busy       // 1 כל עוד משדר — אל תשלח send חדש
);

    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;   // 434 @ 50MHz

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shift   <= 8'd0;
            tx      <= 1'b1;              // קו UART במנוחה = HIGH
            busy    <= 1'b0;
        end else begin
            case (state)

            // ── מנוחה — ממתין לפקודת שליחה ──
            S_IDLE: begin
                tx      <= 1'b1;
                busy    <= 1'b0;
                clk_cnt <= 16'd0;
                bit_idx <= 3'd0;
                if (send) begin
                    shift <= data;
                    busy  <= 1'b1;
                    tx    <= 1'b0;        // ביט התחלה
                    state <= S_START;
                end
            end

            // ── ביט התחלה ──
            S_START: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    tx      <= shift[0];  // ביט המידע הראשון (LSB)
                    bit_idx <= 3'd0;
                    state   <= S_DATA;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ── 8 ביטי מידע ──
            S_DATA: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    if (bit_idx == 3'd7) begin
                        tx    <= 1'b1;                  // ביט עצירה
                        state <= S_STOP;
                    end else begin
                        shift   <= {1'b0, shift[7:1]};
                        tx      <= shift[1];            // הביט הבא
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
                    busy    <= 1'b0;
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