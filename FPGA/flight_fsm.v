// ═══════════════════════════════════════════════════════════
//   flight_fsm.v - מכונת מצבי הבטיחות של ה-FPGA
//
//   DISARMED → READY → ARMED → { LINK_LOSS | ABORT }
//
//   ═══ חלוקת האחריות ═══
//   INAV אחראי על ייצוב, אומדן גובה, ALTHOLD, LAND, RTH, Failsafe וניתוק בנחיתה.
//   ה-FPGA הוא שכבת הבטיחות והבוררות בלבד: הוא בונה את חבילות ה-RC,
//   ומחליט מתי להפסיק לשלוח אותן. הוא לא מוריד מצערת, לא מנחית,
//   ולא מנתק מנועים לפי חיישן המרחק.
//
//   ═══ אובדן קשר ═══
//   אין פקודות שליטה LINK_LOSS_MS בזמן חימוש ⇒ ST_LINK_LOSS:
//   זרם ה-RC נעצר, INAV מזהה RX loss ומפעיל את ה-Failsafe שלו.
//   החבילה האחרונה שיצאה היא עדיין מחומשת, כדי ש-INAV יראה אובדן קליטה
//   ולא פקודת ניתוק. הזרם חוזר רק אחרי CMD:DISARM.
//   ⚠️ כשה-ESP עצמו מת (ESP_DEAD_MS) ההתנהגות **זהה** - ST_LINK_LOSS, מחומשים,
//      הזרם עצור - ומשתנה רק סיבת האבחון (END_ESP). זה **אינו** ABORT:
//      ניתוק מנועים הוא פעולה בלתי הפיכה, והוא שמור אך ורק ל-CMD:ABORT
//      מפורש של הטייס. כשל של רכיב תקשורת אינו מצדיק אותו.
//
//   ⚠️ ב-INAV מוגדר failsafe_procedure = LAND: עצירת הזרם באוויר מפעילה
//      נחיתת חירום של INAV (לא כיבוי מנועים). בדיקת אובדן קשר - קודם על
//      השולחן בלי פרופלורים, ורק אחר כך בטיסה בגובה נמוך.
//
//   חיישני המרחק כאן משמשים לאבחון בלבד (טריות דגימות לפי FSEQ / DSEQ).
//   לוגיקת המכשולים תתווסף בשלב הבא ותשתמש ב-FWD.
//
//   ═══ שער מצבי הניווט לפי בריאות המצפן ═══
//   ה-FPGA הוא הכותב היחיד של CH7, ולכן החסימה כאן ולא רק באפליקציה.
//   mode_req - מה שהתבקש (MODE בשורת השליטה) · mode_out - מה שיוצא ל-CH7.
//     ANGLE / ALTHOLD - תמיד עוברים.
//     POSHOLD / RTH / WP - כניסה חדשה רק כשהמצפן מאומת (MGH=1) והדיווח טרי.
//     מצב ניווט שכבר פעיל נשאר פעיל גם כשהמצפן "לא מאומת" - UNVERIFIED לבדו
//     לעולם אינו מפעיל מעבר באוויר.
//     MGH=2 (קפוא, עם הוכחת תנועה ב-ESP) בזמן POSHOLD / RTH / WP ⇒ ALTHOLD:
//     INAV ממשיך להחזיק גובה, והניווט האופקי - שיכול לברוח - נעצר.
//     אין כאן DISARM ואין שינוי במצערת.
// ═══════════════════════════════════════════════════════════
module flight_fsm #(
    parameter CLK_FREQ      = 50_000_000,
    parameter MSP_RATE_HZ   = 50,
    parameter LINK_LOSS_MS  = 1000,     // אין פקודות שליטה ⇒ עוצרים את זרם ה-RC
    parameter ESP_DEAD_MS   = 1500,     // שום שורה מה-ESP ⇒ ESP מת
    parameter SENS_STALE_MS = 300       // אין דגימת חיישן חדשה ⇒ דגל טריות כבוי
)(
    input  wire       clk,
    input  wire       rst_n,

    // ── מ-cmd_parser ──
    input  wire       cmd_arm,
    input  wire       cmd_disarm,
    input  wire       cmd_check,
    input  wire       cmd_abort,
    input  wire       cmd_ping,
    input  wire       ctrl_valid,

    input  wire       sens_valid,
    input  wire [7:0] lidar_fwd_seq,
    input  wire [7:0] lidar_dwn_seq,

    // ── שער מצבי הניווט ──
    input  wire [2:0] mode_req,       // מ-cmd_parser: 0 ANGLE 1 ALTHOLD 2 POSHOLD 3 RTH 4 WP
    input  wire [1:0] mag_health,     // מ-cmd_parser: 0 לא מאומת · 1 תקין · 2 קפוא
    output reg  [2:0] mode_out,       // ל-msp_builder (CH7)

    // ── ל-msp_builder / drone_top ──
    output reg        armed,
    output reg        send_now,
    input  wire       msp_busy,

    // ── תשובה ל-ESP32 ──
    output reg  [7:0] esp_tx_data,
    output reg        esp_tx_send,
    input  wire       esp_tx_busy,

    // ── חיווי ──
    output wire       rc_hold,        // 1 = זרם ה-RC עצור
    output reg        failsafe,
    output wire [2:0] state_dbg,
    output wire       dwn_fresh_o     // דגימת DOWN חדשה ב-SENS_STALE_MS האחרונים (ל-drone_top)
);

    localparam integer MSP_PERIOD = CLK_FREQ / MSP_RATE_HZ;
    localparam integer MS_DIV     = (CLK_FREQ >= 1000) ? (CLK_FREQ / 1000) : 1;

    localparam ST_DISARMED  = 3'd0,
               ST_READY     = 3'd1,
               ST_ARMED     = 3'd2,
               ST_LINK_LOSS = 3'd3,     // הזרם נעצר, INAV מטפל
               ST_RESERVED  = 3'd4,     // היה LANDING - הנחיתה עברה ל-INAV
               ST_ABORT     = 3'd5;

    // סיבת העצירה האחרונה - נשלחת בתשובת STATUS
    localparam [1:0] END_NONE = 2'd0,
                     END_LINK = 2'd1,   // אין פקודות שליטה
                     END_ESP  = 2'd2;   // ה-ESP מת

    reg [2:0] st;
    reg       msp_hold;                 // 1 = לא שולחים MSP ⇒ INAV נכנס ל-Failsafe שלו
    reg [1:0] last_end;

    assign state_dbg = st;
    assign rc_hold   = msp_hold;

    wire in_air = (st == ST_ARMED) || (st == ST_LINK_LOSS);

    // ═══════════════════════════════════════════════
    //   בסיס זמן: פולס כל 1ms
    // ═══════════════════════════════════════════════
    reg [31:0] ms_cnt;
    reg        ms_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt  <= 32'd0;
            ms_tick <= 1'b0;
        end else begin
            ms_tick <= 1'b0;
            if (ms_cnt >= MS_DIV - 1) begin
                ms_cnt  <= 32'd0;
                ms_tick <= 1'b1;
            end else begin
                ms_cnt <= ms_cnt + 32'd1;
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   דגימות חיישן חדשות - לפי FSEQ / DSEQ.
    //   שורה שחוזרת על אותה דגימה לא נחשבת דגימה חדשה.
    // ═══════════════════════════════════════════════
    reg [7:0] fwd_seq_last, dwn_seq_last;

    wire fwd_new = sens_valid && (lidar_fwd_seq != fwd_seq_last);
    wire dwn_new = sens_valid && (lidar_dwn_seq != dwn_seq_last);

    // ═══════════════════════════════════════════════
    //   גיל כל מקור מידע (ms, רווי ב-65535)
    // ═══════════════════════════════════════════════
    reg [15:0] ctrl_age, fwd_age, dwn_age, esp_age;

    wire any_cmd  = cmd_arm | cmd_disarm | cmd_check | cmd_abort | cmd_ping;
    wire esp_line = ctrl_valid | sens_valid | any_cmd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_age     <= 16'hFFFF;
            fwd_age      <= 16'hFFFF;
            dwn_age      <= 16'hFFFF;
            esp_age      <= 16'hFFFF;
            fwd_seq_last <= 8'd0;
            dwn_seq_last <= 8'd0;
        end else begin
            if (sens_valid) begin
                fwd_seq_last <= lidar_fwd_seq;
                dwn_seq_last <= lidar_dwn_seq;
            end

            if (ctrl_valid)                          ctrl_age <= 16'd0;
            else if (ms_tick && ctrl_age != 16'hFFFF) ctrl_age <= ctrl_age + 16'd1;

            if (fwd_new)                             fwd_age  <= 16'd0;
            else if (ms_tick && fwd_age  != 16'hFFFF) fwd_age  <= fwd_age  + 16'd1;

            if (dwn_new)                             dwn_age  <= 16'd0;
            else if (ms_tick && dwn_age  != 16'hFFFF) dwn_age  <= dwn_age  + 16'd1;

            if (esp_line)                            esp_age  <= 16'd0;
            else if (ms_tick && esp_age  != 16'hFFFF) esp_age  <= esp_age  + 16'd1;
        end
    end

    wire link_lost  = (ctrl_age >= LINK_LOSS_MS);
    wire esp_dead   = (esp_age  >= ESP_DEAD_MS);
    wire ctrl_fresh = (ctrl_age <  LINK_LOSS_MS);
    wire fwd_fresh  = (fwd_age  <  SENS_STALE_MS);
    wire dwn_fresh  = (dwn_age  <  SENS_STALE_MS);
    assign dwn_fresh_o = dwn_fresh;

    // ═══════════════════════════════════════════════
    //   שער מצבי הניווט לפי בריאות המצפן
    //   MGH מגיע בשורת החיישנים (כל 50ms). בלי שורה טרייה - המצפן לא נחשב מאומת.
    // ═══════════════════════════════════════════════
    localparam [2:0] M_ANGLE   = 3'd0,
                     M_ALTHOLD = 3'd1,
                     M_POSHOLD = 3'd2,
                     M_RTH     = 3'd3,
                     M_WP      = 3'd4;
    localparam [1:0] MAG_OK     = 2'd1,
                     MAG_FROZEN = 2'd2;

    reg [15:0] mag_age;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                   mag_age <= 16'hFFFF;
        else if (sens_valid)                          mag_age <= 16'd0;
        else if (ms_tick && mag_age != 16'hFFFF)      mag_age <= mag_age + 16'd1;
    end

    wire mag_ok     = (mag_age < SENS_STALE_MS) && (mag_health == MAG_OK);
    wire mag_frozen = (mag_health == MAG_FROZEN);
    wire req_nav    = (mode_req == M_POSHOLD) || (mode_req == M_RTH) || (mode_req == M_WP);
    wire out_nav    = (mode_out == M_POSHOLD) || (mode_out == M_RTH) || (mode_out == M_WP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  mode_out <= M_ANGLE;
        else if (!req_nav)           mode_out <= mode_req;       // ANGLE / ALTHOLD - תמיד
        else if (mag_frozen) begin
            if (out_nav)             mode_out <= M_ALTHOLD;      // failover מבוקר
            // אחרת: לא נכנסים לניווט, נשארים במצב הנוכחי
        end
        else if (mode_out == mode_req) mode_out <= mode_out;     // כבר במצב הזה - נשאר
        else if (mag_ok)             mode_out <= mode_req;       // כניסה חדשה - רק עם מצפן מאומת
        // אחרת: חסום - נשארים במצב הנוכחי
    end

    // ═══════════════════════════════════════════════
    //   מכונת המצבים
    // ═══════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= ST_DISARMED;
            armed    <= 1'b0;
            failsafe <= 1'b0;
            msp_hold <= 1'b0;
            last_end <= END_NONE;
        end else begin

            // ═══ ABORT - כיבוי מנועים מיידי, עדיפות עליונה ═══
            if (cmd_abort) begin
                st       <= ST_ABORT;
                armed    <= 1'b0;
                failsafe <= 1'b1;
                // ⚠️ שחרור עצירת הזרם. ABORT הוא בקשה מפורשת של הטייס לכבות מנועים,
                //    ולכן INAV חייב לקבל חבילה עם AUX1=1000. בלי השורה הזו, ABORT
                //    שנלחץ ממצב אובדן קשר היה משאיר את הזרם עצור ואת המנועים חיים
                //    תחת ה-Failsafe של INAV, במקום לנתק אותם מיד.
                msp_hold <= 1'b0;
            end else begin

            case (st)

            // ─── מנוטרל - המנועים כבויים, זרם ה-RC פעיל ───
            ST_DISARMED: begin
                armed    <= 1'b0;
                failsafe <= 1'b0;
                msp_hold <= 1'b0;
                if (cmd_check) st <= ST_READY;
            end

            ST_READY: begin
                armed    <= 1'b0;
                failsafe <= 1'b0;
                msp_hold <= 1'b0;
                if (cmd_arm) begin
                    last_end <= END_NONE;
                    st       <= ST_ARMED;
                end
                else if (cmd_disarm) st <= ST_DISARMED;
            end

            ST_ARMED: begin
                armed    <= 1'b1;
                failsafe <= 1'b0;

                if (esp_dead) begin
                    // ה-ESP מת: אין פקודות ואין חיישנים. עוצרים את זרם ה-RC
                    // והחבילה האחרונה נשארת מחומשת, כדי ש-INAV יזהה אובדן קליטה
                    // ויפעיל את ה-Failsafe שלו.
                    // ⚠️ זה **לא** ABORT. ABORT שמור אך ורק לפקודת חירום מפורשת
                    //    של הטייס. כאן רק סיבת האבחון שונה מאובדן שליטה רגיל.
                    failsafe <= 1'b1;
                    msp_hold <= 1'b1;
                    last_end <= END_ESP;
                    st       <= ST_LINK_LOSS;
                end
                else if (link_lost) begin
                    // אין פקודות שליטה: עוצרים את הזרם, נשארים מחומשים בחבילה האחרונה
                    failsafe <= 1'b1;
                    msp_hold <= 1'b1;
                    last_end <= END_LINK;
                    st       <= ST_LINK_LOSS;
                end
                else if (cmd_disarm) st <= ST_READY;
            end

            // ─── אובדן קשר: הזרם עצור, INAV מנהל את האירוע ───
            //   אין חזרה אוטומטית לשליטה. היציאה: CMD:DISARM (או ABORT).
            ST_LINK_LOSS: begin
                armed    <= 1'b1;        // החבילה האחרונה שיצאה נשארה מחומשת
                failsafe <= 1'b1;
                msp_hold <= 1'b1;
                // אם גם שורות החיישנים פסקו - ה-ESP עצמו מת. ההתנהגות זהה,
                // הזרם כבר עצור, ומשתנה רק סיבת האבחון שמדווחת לאפליקציה.
                // אין מעבר ל-ABORT: ניתוק מנועים מתבצע רק בפקודה מפורשת.
                if (esp_dead) last_end <= END_ESP;

                if (cmd_disarm) begin
                    armed    <= 1'b0;
                    failsafe <= 1'b0;
                    msp_hold <= 1'b0;
                    st       <= ST_DISARMED;
                end
            end

            ST_ABORT: begin
                armed    <= 1'b0;
                failsafe <= 1'b1;
                if (cmd_disarm) begin
                    msp_hold <= 1'b0;
                    st       <= ST_DISARMED;
                end
            end

            default: st <= ST_DISARMED;   // כולל ST_RESERVED

            endcase
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   מחולל קצב - send_now כל 20ms, אלא אם הזרם עצור
    //
    //   hold_now חוסם כבר במחזור שבו מזוהה אובדן הקשר, עוד לפני ש-msp_hold
    //   ננעל. כך החבילה האחרונה שיוצאת היא עדיין מחומשת, ולא יכולה לצאת
    //   חבילה עם AUX1=1000 שהייתה מנתקת את INAV לפני זיהוי ה-RX loss.
    // ═══════════════════════════════════════════════
    wire hold_now = msp_hold || ((esp_dead || link_lost) && in_air);

    reg [31:0] msp_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msp_cnt  <= 32'd0;
            send_now <= 1'b0;
        end else begin
            send_now <= 1'b0;
            if (msp_cnt >= MSP_PERIOD - 1) begin
                msp_cnt <= 32'd0;
                if (!msp_busy && !hold_now) send_now <= 1'b1;
            end else begin
                msp_cnt <= msp_cnt + 32'd1;
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   משיב "STATUS:OK:<st>:<F><R>\n" על CMD:CHECK או CMD:PING
    //     <st>  מצב 0..5
    //     <F>   ספרת hex: bit3 זרם RC עצור · bit2 דגימת FWD טרייה
    //                     bit1 דגימת DWN טרייה · bit0 פקודות שליטה טריות
    //     <R>   עצירה אחרונה: '-' אין · 'L' אובדן פקודות · 'E' ESP מת
    //     :F1022B  גרסת ה-FPGA - תווים קבועים בסוף התשובה (ה-ESP מעביר לאפליקציה).
    //              ה-ESP קורא את 14 התווים הראשונים כמו קודם, כך שהתוספת תואמת לאחור.
    //   ⚠️ לעדכן את הגרסה (6 התווים) בכל שינוי ב-FPGA.
    // ═══════════════════════════════════════════════
    localparam RESP_LEN = 5'd22;

    wire [3:0] dbg_flags = { msp_hold, fwd_fresh, dwn_fresh, ctrl_fresh };
    wire [7:0] flag_ch   = (dbg_flags < 4'd10) ? (8'h30 + {4'd0, dbg_flags})
                                               : (8'h37 + {4'd0, dbg_flags});
    reg  [7:0] end_ch;

    always @* begin
        case (last_end)
            END_LINK: end_ch = "L";
            END_ESP : end_ch = "E";
            default : end_ch = "-";
        endcase
    end

    reg [4:0] resp_idx;
    reg [7:0] resp_byte;

    always @* begin
        case (resp_idx)
            5'd0   : resp_byte = "S";
            5'd1   : resp_byte = "T";
            5'd2   : resp_byte = "A";
            5'd3   : resp_byte = "T";
            5'd4   : resp_byte = "U";
            5'd5   : resp_byte = "S";
            5'd6   : resp_byte = ":";
            5'd7   : resp_byte = "O";
            5'd8   : resp_byte = "K";
            5'd9   : resp_byte = ":";
            5'd10  : resp_byte = 8'h30 + {5'd0, st};
            5'd11  : resp_byte = ":";
            5'd12  : resp_byte = flag_ch;
            5'd13  : resp_byte = end_ch;
            // ── גרסת ה-FPGA ──
            5'd14  : resp_byte = ":";
            5'd15  : resp_byte = "F";
            5'd16  : resp_byte = "1";
            5'd17  : resp_byte = "0";
            5'd18  : resp_byte = "2";
            5'd19  : resp_byte = "2";
            5'd20  : resp_byte = "B";
            5'd21  : resp_byte = 8'h0A;
            default: resp_byte = 8'h00;
        endcase
    end

    localparam R_IDLE = 2'd0, R_LOAD = 2'd1, R_WAIT = 2'd2, R_HOLD = 2'd3;
    reg [1:0] r_st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_st        <= R_IDLE;
            resp_idx    <= 5'd0;
            esp_tx_data <= 8'd0;
            esp_tx_send <= 1'b0;
        end else begin
            case (r_st)
            R_IDLE: begin
                esp_tx_send <= 1'b0;
                if (cmd_check || cmd_ping) begin
                    resp_idx <= 5'd0;
                    r_st     <= R_LOAD;
                end
            end
            R_LOAD: if (!esp_tx_busy) begin
                    esp_tx_data <= resp_byte;
                    esp_tx_send <= 1'b1;
                    r_st        <= R_WAIT;
                end
            R_WAIT: begin
                    esp_tx_send <= 1'b0;
                    if (esp_tx_busy) r_st <= R_HOLD;
                end
            R_HOLD: if (!esp_tx_busy) begin
                    if (resp_idx == RESP_LEN - 1) r_st <= R_IDLE;
                    else begin resp_idx <= resp_idx + 5'd1; r_st <= R_LOAD; end
                end
            default: r_st <= R_IDLE;
            endcase
        end
    end

endmodule
