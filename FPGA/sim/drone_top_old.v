// ═══════════════════════════════════════════════════════════
//   drone_top_old.v - נקודת הייחוס של tb_regress בלבד. לא נכנס ל-Quartus.
//   זהו drone_top.v כפי שהיה לפני ה-Mission FSM (גרסה v6), ורק שם המודול
//   שונה ל-drone_top_old, כדי שאפשר יהיה לסמלץ אותו לצד הגרסה החדשה.
// ═══════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════
//   drone_top.v - המודול הראשי
//
//   ה-FPGA בונה את חבילות ה-RC ל-Matek ומחליט מתי להפסיק לשלוח אותן.
//   אין כאן בקרת גובה, אין נחיתה ואין הורדת מצערת: כל אלה באחריות INAV.
// ═══════════════════════════════════════════════════════════
module drone_top_old #(
    parameter CLK_FREQ       = 50_000_000,
    parameter BAUD_RATE      = 115_200,
    parameter MSP_RATE_HZ    = 50,
    parameter LINK_LOSS_MS   = 1000,
    parameter ESP_DEAD_MS    = 1500,
    parameter LED_ACTIVE_LOW = 1
)(
    input  wire        clk,        // 50MHz          - PIN_17
    input  wire        esp_rx,     // ◀── ESP32 GPIO17 - PIN_43
    output wire        esp_tx,     // ──▶ ESP32 GPIO18 - PIN_42
    output wire        fc_tx,      // ──▶ Matek RX2    - PIN_41
    input  wire        fc_rx,      // ◀── Matek TX2    - PIN_40 (שמור)
    output wire [2:0]  led         // PIN_3 / PIN_7 / PIN_9
);

    // ═══ Power-On Reset - משחרר אחרי ~1.3ms ═══
    reg [15:0] por_cnt = 16'd0;
    reg        rst_n   = 1'b0;

    always @(posedge clk) begin
        if (por_cnt != 16'hFFFF) begin
            por_cnt <= por_cnt + 16'd1;
            rst_n   <= 1'b0;
        end else begin
            rst_n   <= 1'b1;
        end
    end

    // ═══ חיווטים פנימיים ═══
    wire [7:0] rx_data;
    wire       rx_valid;

    wire [7:0]        thr_raw;        // 0..200, חצאי אחוז
    wire signed [7:0] yaw_raw, pch_raw, rll_raw;
    wire [2:0]        mode;           // מבוקש: 0 ANGLE · 1 ALTHOLD · 2 POSHOLD · 3 RTH · 4 WP
    wire [2:0]        mode_out;       // אחרי שער המצפן ב-flight_fsm → CH7
    wire [1:0]        mag_health;     // 0 לא מאומת · 1 תקין · 2 קפוא
    wire              ctrl_valid;
    wire              cmd_arm, cmd_disarm, cmd_check;
    wire              cmd_abort, cmd_ping;

    wire [8:0]        lidar_fwd,     lidar_dwn;
    wire [2:0]        lidar_fwd_st,  lidar_dwn_st;    // 0=FAULT 1=VALID 2=NO_RETURN 3=UNCERTAIN 4=TOO_CLOSE
    wire [7:0]        lidar_fwd_seq, lidar_dwn_seq;   // מוני דגימות מה-ESP
    wire              sens_valid;

    wire       armed, send_now, failsafe, rc_hold;
    wire [2:0] state_dbg;
    wire [7:0] esp_tx_data;
    wire       esp_tx_send, esp_uart_busy;

    wire       msp_pkt_busy;
    wire [7:0] fc_tx_data;
    wire       fc_tx_send, fc_uart_busy;

    // ═══ שער בטיחות ═══
    //   מנוטרל ⇒ מצערת מינימלית והמקלות במרכז, בכל מקרה.
    wire [7:0] thr_safe = armed ? thr_raw : 8'd0;

    wire signed [7:0] yaw_safe = armed ? yaw_raw : 8'sd0;
    wire signed [7:0] pch_safe = armed ? pch_raw : 8'sd0;
    wire signed [7:0] rll_safe = armed ? rll_raw : 8'sd0;

    // ═══ 1 - קליטה מה-ESP32 ═══
    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_rx_esp (
        .clk(clk), .rst_n(rst_n), .rx(esp_rx),
        .data(rx_data), .valid(rx_valid)
    );

    // ═══ 2 - פענוח ═══
    //   cmd_land מפוענח אך לא מחובר: הנחיתה היא באחריות INAV.
    cmd_parser u_parser (
        .clk           (clk),
        .rst_n         (rst_n),
        .rx_data       (rx_data),
        .rx_valid      (rx_valid),
        .thr           (thr_raw),
        .yaw           (yaw_raw),
        .pch           (pch_raw),
        .rll           (rll_raw),
        .mode          (mode),
        .ctrl_valid    (ctrl_valid),
        .lidar_fwd     (lidar_fwd),
        .lidar_dwn     (lidar_dwn),
        .lidar_fwd_st  (lidar_fwd_st),
        .lidar_dwn_st  (lidar_dwn_st),
        .lidar_fwd_seq (lidar_fwd_seq),
        .lidar_dwn_seq (lidar_dwn_seq),
        .mag_health    (mag_health),
        .sens_valid    (sens_valid),
        .cmd_arm       (cmd_arm),
        .cmd_disarm    (cmd_disarm),
        .cmd_check     (cmd_check),
        .cmd_abort     (cmd_abort),
        .cmd_land      (),
        .cmd_ping      (cmd_ping)
    );

    // ═══ 3 - מכונת מצבי הבטיחות ═══
    flight_fsm #(
        .CLK_FREQ(CLK_FREQ), .MSP_RATE_HZ(MSP_RATE_HZ),
        .LINK_LOSS_MS(LINK_LOSS_MS), .ESP_DEAD_MS(ESP_DEAD_MS)
    ) u_fsm (
        .clk           (clk),
        .rst_n         (rst_n),
        .cmd_arm       (cmd_arm),
        .cmd_disarm    (cmd_disarm),
        .cmd_check     (cmd_check),
        .cmd_abort     (cmd_abort),
        .cmd_ping      (cmd_ping),
        .ctrl_valid    (ctrl_valid),
        .sens_valid    (sens_valid),
        .lidar_fwd_seq (lidar_fwd_seq),
        .lidar_dwn_seq (lidar_dwn_seq),
        .mode_req      (mode),
        .mag_health    (mag_health),
        .mode_out      (mode_out),
        .armed         (armed),
        .send_now      (send_now),
        .msp_busy      (msp_pkt_busy),
        .esp_tx_data   (esp_tx_data),
        .esp_tx_send   (esp_tx_send),
        .esp_tx_busy   (esp_uart_busy),
        .rc_hold       (rc_hold),
        .failsafe      (failsafe),
        .state_dbg     (state_dbg)
    );

    // ═══ 4 - בניית חבילות MSP ═══
    msp_builder u_msp (
        .clk(clk), .rst_n(rst_n),
        .thr(thr_safe), .yaw(yaw_safe), .pch(pch_safe), .rll(rll_safe),
        .mode(mode_out), .armed(armed),
        .send_now(send_now), .busy(msp_pkt_busy),
        .tx_data(fc_tx_data), .tx_send(fc_tx_send), .tx_busy(fc_uart_busy)
    );

    // ═══ 5 - שידור ל-Matek ═══
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx_fc (
        .clk(clk), .rst_n(rst_n),
        .data(fc_tx_data), .send(fc_tx_send),
        .tx(fc_tx), .busy(fc_uart_busy)
    );

    // ═══ 6 - שידור חזרה ל-ESP32 ═══
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx_esp (
        .clk(clk), .rst_n(rst_n),
        .data(esp_tx_data), .send(esp_tx_send),
        .tx(esp_tx), .busy(esp_uart_busy)
    );

    // ═══ שריון PIN_40 עד שנממש קליטת MSP מה-Matek ═══
    reg fc_rx_reserved /* synthesis preserve */;
    always @(posedge clk) fc_rx_reserved <= fc_rx;

    // ═══ שמירה על מדידות הלידאר עד שלוגיקת המכשולים תתווסף (שלב הבא) ═══
    reg [8:0] lidar_fwd_keep /* synthesis preserve */;
    reg [8:0] lidar_dwn_keep /* synthesis preserve */;
    reg [2:0] lidar_fwd_st_keep /* synthesis preserve */;
    reg [2:0] lidar_dwn_st_keep /* synthesis preserve */;

    always @(posedge clk) begin
        lidar_fwd_keep    <= lidar_fwd;
        lidar_dwn_keep    <= lidar_dwn;
        lidar_fwd_st_keep <= lidar_fwd_st;
        lidar_dwn_st_keep <= lidar_dwn_st;
    end

    // ═══ נוריות - Active Low ═══
    //    led[0] אבחון MSP (מפסיקה להבהב כשהזרם נעצר) · led[1] אובדן קשר / ABORT · led[2] פעימת לב 1Hz
    reg [24:0] hb_cnt    = 25'd0;
    reg        heartbeat = 1'b0;

    always @(posedge clk) begin
        if (hb_cnt >= (CLK_FREQ / 2) - 1) begin
            hb_cnt    <= 25'd0;
            heartbeat <= ~heartbeat;
        end else begin
            hb_cnt <= hb_cnt + 25'd1;
        end
    end

    reg  msp_busy_d = 1'b0;
    always @(posedge clk) msp_busy_d <= msp_pkt_busy;
    wire msp_done = msp_busy_d & ~msp_pkt_busy;

    reg [5:0] msp_div = 6'd0;
    reg       msp_hb  = 1'b0;

    always @(posedge clk) begin
        if (msp_done) begin
            if (msp_div >= 6'd11) begin
                msp_div <= 6'd0;
                msp_hb  <= ~msp_hb;
            end else begin
                msp_div <= msp_div + 6'd1;
            end
        end
    end

    wire [2:0] led_raw = { heartbeat, (failsafe | rc_hold), msp_hb };
    assign led = LED_ACTIVE_LOW ? ~led_raw : led_raw;

endmodule
