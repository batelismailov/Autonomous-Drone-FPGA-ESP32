// ═══════════════════════════════════════════════════════════
//   msp_builder.v - בניית חבילת MSP_SET_RAW_RC ל-Matek F405
//
//   מבנה החבילה (22 בתים):
//     '$' 'M' '<' <size=16> <cmd=200> <16 בתים מידע> <XOR>
//
//   סדר ערוצים, כל ערוץ uint16 little-endian ב-µs:
//     CH1 Roll · CH2 Pitch · CH3 Throttle · CH4 Yaw
//     CH5 AUX1  ARM            1000 מנוטרל · 2000 מחומש       (INAV: 1700-2100)
//     CH6 AUX2  ANGLE          1500 תמיד                      (INAV: 900-2100)
//     CH7 AUX3  בורר ניווט     ANGLE 1000 · ALTHOLD 1250 · POSHOLD 1500 · RTH 1750 · WP 2000
//                              (INAV: ALTHOLD 1150-1350 · POSHOLD 1400-1600 ·
//                                     RTH 1650-1850 · WP 1900-2100)
//     CH8 AUX4  SURFACE        1000 כבוי · 2000 מבוקש (F1022A; INAV: aux SURFACE 1700-2100)
//
//   ⚠️ CH7 עובר ישירות מערך לערך, בלי slew ובלי מעבר הדרגתי. הטווחים ב-INAV
//      צמודים זה לזה: מעבר הדרגתי מ-ALTHOLD (1250) ל-WP (2000) היה עובר דרך
//      POSHOLD ו-RTH ומפעיל אותם בדרך. הערך נגזר ישירות מאוגר המצב, והחבילה
//      נועלת אותו פעם אחת - לכן אין חבילה שנושאת ערך ביניים.
//
//   ⚠️ CH7 הוא בקשה. INAV מחליט אם המצב מופעל (למשל POSHOLD בלי GPS לא
//      יופעל), והאפליקציה פועלת לפי המצב שהוא מדווח בפועל.
// ═══════════════════════════════════════════════════════════
module msp_builder (
    input  wire              clk,
    input  wire              rst_n,

    // ── ערכי שליטה (מ-cmd_parser, דרך שער הבטיחות ב-drone_top) ──
    input  wire [7:0]        thr,        // 0..200, חצאי אחוז (131 = 65.5%)
    input  wire signed [7:0] yaw,        // -100..100
    input  wire signed [7:0] pch,
    input  wire signed [7:0] rll,
    input  wire [2:0]        mode,       // 0=ANGLE 1=ALTHOLD 2=POSHOLD 3=RTH 4=WP
    input  wire              armed,      // מ-flight_fsm
    input  wire              surf,       // F1022A: CH8 = SURFACE (מ-drone_top, אחרי השער)

    // ── הפעלה ──
    input  wire              send_now,   // פולס: בנה ושדר חבילה
    output reg               busy,       // 1 כל עוד החבילה בשידור

    // ── אל uart_tx של ערוץ ה-FC ──
    output reg  [7:0]        tx_data,
    output reg               tx_send,
    input  wire              tx_busy
);

    localparam MSP_SET_RAW_RC = 8'd200;
    localparam PAYLOAD_SIZE   = 8'd16;      // 8 ערוצים × 2 בתים
    localparam LAST_BYTE      = 5'd21;      // אינדקס הבית האחרון (checksum)

    // ═══════════════════════════════════════════════
    //   המרת ערכי שליטה ל-PWM  (1000..2000 µs)
    // ═══════════════════════════════════════════════

    // דו-כיווני: -100..100 → 0..200 → 1000..2000
    wire [8:0] yaw_u = $signed({yaw[7], yaw}) + 9'sd100;
    wire [8:0] pch_u = $signed({pch[7], pch}) + 9'sd100;
    wire [8:0] rll_u = $signed({rll[7], rll}) + 9'sd100;

    wire [15:0] pwm_yaw = 16'd1000 + ({7'd0, yaw_u} * 16'd5);
    wire [15:0] pwm_pch = 16'd1000 + ({7'd0, pch_u} * 16'd5);
    wire [15:0] pwm_rll = 16'd1000 + ({7'd0, rll_u} * 16'd5);

    // חד-כיווני, בחצאי אחוז: 0..200 → 1000..2000, צעד של 5µs
    wire [15:0] pwm_thr = 16'd1000 + ({8'd0, thr} * 16'd5);

    // בורר הניווט על CH7. ערך לא חוקי ⇒ ANGLE, המצב הבטוח
    reg [15:0] pwm_nav;
    always @* begin
        case (mode)
            3'd1:    pwm_nav = 16'd1250;    // NAV ALTHOLD
            3'd2:    pwm_nav = 16'd1500;    // NAV POSHOLD
            3'd3:    pwm_nav = 16'd1750;    // NAV RTH
            3'd4:    pwm_nav = 16'd2000;    // NAV WP
            default: pwm_nav = 16'd1000;    // ANGLE - אף מצב ניווט
        endcase
    end

    // ═══════════════════════════════════════════════
    //   מאגר הערוצים - ננעל בתחילת כל חבילה
    // ═══════════════════════════════════════════════
    reg [15:0] ch [0:7];

    // ═══════════════════════════════════════════════
    //   בחירת הבית הנוכחי לשידור
    // ═══════════════════════════════════════════════
    reg  [4:0] byte_idx;
    reg  [7:0] csum;

    wire [3:0] p        = byte_idx - 5'd5;   // מיקום בתוך המידע 0..15
    wire [2:0] ch_sel   = p[3:1];            // איזה ערוץ
    wire       hi_byte  = p[0];              // בית עליון או תחתון

    reg [7:0] cur_byte;
    always @* begin
        case (byte_idx)
            5'd0    : cur_byte = 8'h24;            // '$'
            5'd1    : cur_byte = 8'h4D;            // 'M'
            5'd2    : cur_byte = 8'h3C;            // '<'
            5'd3    : cur_byte = PAYLOAD_SIZE;
            5'd4    : cur_byte = MSP_SET_RAW_RC;
            LAST_BYTE: cur_byte = csum;
            default : cur_byte = hi_byte ? ch[ch_sel][15:8]
                                         : ch[ch_sel][7:0];    // little-endian
        endcase
    end

    // בתים 3..20 נכנסים לחישוב ה-checksum
    wire in_csum = (byte_idx >= 5'd3) && (byte_idx <= 5'd20);

    // ═══════════════════════════════════════════════
    //   מכונת מצבים - שידור בית אחר בית
    // ═══════════════════════════════════════════════
    localparam S_IDLE = 2'd0,   // ממתין לפקודה
               S_LOAD = 2'd1,   // טוען בית ל-uart_tx
               S_WAIT = 2'd2,   // ממתין שה-uart יתחיל
               S_HOLD = 2'd3;   // ממתין שה-uart יסיים

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            byte_idx <= 5'd0;
            csum     <= 8'd0;
            tx_data  <= 8'd0;
            tx_send  <= 1'b0;
            busy     <= 1'b0;
            ch[0]    <= 16'd1500;   // Roll     - מרכז
            ch[1]    <= 16'd1500;   // Pitch    - מרכז
            ch[2]    <= 16'd1000;   // Throttle - מינימום!
            ch[3]    <= 16'd1500;   // Yaw      - מרכז
            ch[4]    <= 16'd1000;   // CH5 AUX1 - DISARM
            ch[5]    <= 16'd1500;   // CH6 AUX2 - ANGLE
            ch[6]    <= 16'd1000;   // CH7 AUX3 - אף מצב ניווט
            ch[7]    <= 16'd1000;   // CH8 AUX4 - שמור
        end else begin
            case (state)

            // ── ממתין לפקודת שידור ──
            S_IDLE: begin
                tx_send <= 1'b0;
                busy    <= 1'b0;
                if (send_now) begin
                    // נעילת כל הערכים - לא ישתנו באמצע החבילה
                    ch[0] <= pwm_rll;                        // CH1 Roll
                    ch[1] <= pwm_pch;                        // CH2 Pitch
                    ch[2] <= pwm_thr;                        // CH3 Throttle
                    ch[3] <= pwm_yaw;                        // CH4 Yaw
                    ch[4] <= armed ? 16'd2000 : 16'd1000;    // CH5 AUX1 = ARM
                    ch[5] <= 16'd1500;                       // CH6 AUX2 = ANGLE, תמיד
                    ch[6] <= pwm_nav;                        // CH7 AUX3 = בורר ניווט
                    ch[7] <= surf ? 16'd2000 : 16'd1000;     // CH8 AUX4 = SURFACE (F1022A)

                    byte_idx <= 5'd0;
                    csum     <= 8'd0;
                    busy     <= 1'b1;
                    state    <= S_LOAD;
                end
            end

            // ── טעינת בית ל-uart_tx ──
            S_LOAD: begin
                if (!tx_busy) begin
                    tx_data <= cur_byte;
                    tx_send <= 1'b1;
                    if (in_csum) csum <= csum ^ cur_byte;
                    state   <= S_WAIT;
                end
            end

            // ── פולס send באורך מחזור אחד ──
            S_WAIT: begin
                tx_send <= 1'b0;
                if (tx_busy) state <= S_HOLD;
            end

            // ── המתנה לסיום השידור ──
            S_HOLD: begin
                if (!tx_busy) begin
                    if (byte_idx == LAST_BYTE) begin
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        byte_idx <= byte_idx + 5'd1;
                        state    <= S_LOAD;
                    end
                end
            end

            default: state <= S_IDLE;

            endcase
        end
    end

endmodule
