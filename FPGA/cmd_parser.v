// ═══════════════════════════════════════════════════════════
//   cmd_parser.v - פענוח פקודות ומדידות מה-ESP32
//
//   שליטה:  "THR:50 YAW:-20 PCH:0 RLL:0 MODE:ANGLE DSC:0 PHA:3 TH2:100\r\n"
//   פקודה:  "CMD:ARM" / "CMD:DISARM" / "CMD:CHECK"
//           "CMD:ABORT" / "CMD:PING"
//   חיישן:  "FWD:70 FOK:1 DWN:0 DOK:4 FSEQ:17 DSEQ:203 PHS:3 FWA:1 MGH:1 SRF:0\r\n"
//
//   THR / TH2:    המצערת. thr נשמר ביחידות של **חצי אחוז** (0..200, 131 = 65.5%).
//                 THR:n - אחוזים שלמים, נשמר כ-2n (שורה ישנה נותנת תוצאה זהה).
//                 TH2:n - חצאי אחוז, 0..200 (תקרה 200). ה-ESP מוסיף אותו בסוף
//                 השורה **אחרי** אכיפת רצפת המצערת, ולכן הוא גובר על THR.
//                 ⚠️ thr מתעדכן פעם אחת, בסוף השורה (LF) - לא באמצע. אחרת בין
//                    THR לבין TH2 של אותה שורה היה ערך ביניים (65.0 במקום 65.5),
//                    וחבילת RC שנבנית באותו רגע הייתה נושאת אותו.
//                 ⚠️ THH של האפליקציה אינו מפוענח כאן בכוונה: ה-FPGA מקבל רק
//                    את הערך שה-ESP הפיק אחרי הרצפה.
//   MODE:         מצב הטיסה המבוקש, ANGLE / ALTHOLD / POSHOLD / RTH / WP
//                 0=ANGLE 1=ALTHOLD 2=POSHOLD 3=RTH 4=WP - msp_builder ממיר ל-CH7.
//                 ערך לא מוכר - כולל MAN / AUTO / RTL של הגרסה הקודמת - אינו
//                 משנה את המצב: נשאר המצב האחרון שהתקבל.
//                 ⚠️ זו בקשה בלבד. INAV מחליט אם המצב אכן מופעל, והאפליקציה
//                    פועלת לפי המצב שהוא מדווח בפועל ולא לפי מה שנבחר.
//   FOK / DOK:    0=SENSOR_FAULT 1=VALID 2=NO_RETURN 3=UNCERTAIN 4=TOO_CLOSE
//                 ערך לא מוכר (5 ומעלה) נקלט כ-SENSOR_FAULT
//   FSEQ / DSEQ:  מונה הדגימות של כל חיישן ב-ESP (0..255). משתנה רק כשהגיעה
//                 דגימה חדשה - השורה עצמה נשלחת כל 50ms גם בלי דגימה חדשה
//   SRF:          (F1022A) בקשת SURFACE מה-ESP, בשורת החיישנים (20Hz): 1 = לבקש, אחר = 0.
//                 ה-ESP מחשב אותה (בקשת האפליקציה + DOWN תקין). ה-FPGA מוסיף שער משלו
//                 (drone_top: POSHOLD בלבד, בלי משימה, זרם RC פעיל). אחרי איפוס: 0.
//   THS:          (F1022B) מרכז המקל של האפליקציה ב-SURFACE, חצאי אחוז 0..200, בשורת החיישנים.
//                 ה-FPGA מחזיק אותו ב-CH3 ביציאה מ-SURFACE (drone_top). ‎-1 / מעל 200 ⇒ 255 = אין.
//   MGH:          בריאות המצפן מה-ESP: 0=לא מאומת 1=תקין 2=קפוא.
//                 ערך לא מוכר נקלט כ-0 (לא מאומת). אחרי איפוס: 0.
//                 flight_fsm חוסם לפיו כניסה ל-POSHOLD / RTH / WP.
//
//   ⚠️ שורת חיישן מפעילה sens_valid בלבד ולא ctrl_valid, כדי שנתוני חיישן
//      לא יאפסו את מונה אובדן הקשר.
//
//   ⚠️ שורת השליטה נושאת גם שדות שנועדו ל-ESP בלבד, למשל DSC (כוונת ירידה)
//      ו-PHA (אישור שלב הטיסה מהאפליקציה). מפתח שאינו ברשימה נופל ל-default
//      ואינו נשמר בשום מוצא - אבל הוא גם **אינו** מבטל את line_is_ctrl, ולכן
//      השורה נשארת שורת שליטה תקפה. אפשר להוסיף שדות כאלה בלי לגעת כאן.
//
//   הערה: ALT / VSP כבר לא נשלחים מה-ESP. אומדן הגובה, בקרת הגובה והנחיתה
//   הם באחריות INAV בלבד, ולכן ה-FPGA לא מקבל ולא משתמש בהם.
//   CMD:LAND עדיין מפוענח לתאימות, אבל לא מחובר לשום לוגיקה.
// ═══════════════════════════════════════════════════════════
module cmd_parser (
    input  wire               clk,
    input  wire               rst_n,

    input  wire [7:0]         rx_data,
    input  wire               rx_valid,

    // ── ערכי שליטה ──
    output reg  [7:0]         thr,           // 0..200, חצאי אחוז
    output reg  signed [7:0]  yaw,           // -100..100
    output reg  signed [7:0]  pch,
    output reg  signed [7:0]  rll,
    output reg  [2:0]         mode,          // 0=ANGLE 1=ALTHOLD 2=POSHOLD 3=RTH 4=WP
    output reg                srf,           // F1022A: בקשת SURFACE (SRF:1)
    output reg  [7:0]         ths,           // F1022B: מרכז המקל ב-SURFACE (THS), 0..200; 255 = אין
    output reg                ctrl_valid,

    // ── מדידות לידאר ──
    output reg  [8:0]         lidar_fwd,     // cm מקצה הפרופלור
    output reg  [8:0]         lidar_dwn,     // cm מתחתית הרחפן
    output reg  [2:0]         lidar_fwd_st,  // 0..4, ראו למעלה
    output reg  [2:0]         lidar_dwn_st,
    output reg  [7:0]         lidar_fwd_seq, // מונה דגימות FWD
    output reg  [7:0]         lidar_dwn_seq, // מונה דגימות DWN
    output reg  [1:0]         mag_health,    // 0=לא מאומת 1=תקין 2=קפוא
    output reg                sens_valid,

    // ── פקודות (פולס מחזור אחד) ──
    output reg                cmd_arm,
    output reg                cmd_disarm,
    output reg                cmd_check,
    output reg                cmd_abort,
    output reg                cmd_land,      // מפוענח, לא בשימוש
    output reg                cmd_ping
);

    localparam CH_COLON = 8'h3A,
               CH_SPACE = 8'h20,
               CH_LF    = 8'h0A,
               CH_CR    = 8'h0D,
               CH_MINUS = 8'h2D,
               CH_ZERO  = 8'h30,
               CH_NINE  = 8'h39;

    localparam K_THR  = 32'h0054_4852,   // "THR"
               K_TH2  = 32'h0054_4832,   // "TH2"
               K_YAW  = 32'h0059_4157,   // "YAW"
               K_PCH  = 32'h0050_4348,   // "PCH"
               K_RLL  = 32'h0052_4C4C,   // "RLL"
               K_MODE = 32'h4D4F_4445,   // "MODE"
               K_CMD  = 32'h0043_4D44,   // "CMD"
               K_FWD  = 32'h0046_5744,   // "FWD"
               K_DWN  = 32'h0044_574E,   // "DWN"
               K_FOK  = 32'h0046_4F4B,   // "FOK"
               K_DOK  = 32'h0044_4F4B,   // "DOK"
               K_FSEQ = 32'h4653_4551,   // "FSEQ"
               K_DSEQ = 32'h4453_4551,   // "DSEQ"
               K_MGH  = 32'h004D_4748,   // "MGH"
               K_SRF  = 32'h0053_5246,   // "SRF"
               K_THS  = 32'h0054_4853;   // "THS"

    // ערכי MODE - val_reg שומר את שמונת התווים האחרונים, והארוך מביניהם 7
    localparam V_ANGLE   = 64'h0000_0041_4E47_4C45,  // "ANGLE"
               V_ALTHOLD = 64'h0041_4C54_484F_4C44,  // "ALTHOLD"
               V_POSHOLD = 64'h0050_4F53_484F_4C44,  // "POSHOLD"
               V_RTH     = 64'h0000_0000_0052_5448,  // "RTH"
               V_WP      = 64'h0000_0000_0000_5750;  // "WP"

    // ערכי CMD
    localparam V_ARM    = 64'h0000_0000_0041_524D,   // "ARM"
               V_DISARM = 64'h0000_4449_5341_524D,   // "DISARM"
               V_CHECK  = 64'h0000_0043_4845_434B,   // "CHECK"
               V_ABORT  = 64'h0000_0041_424F_5254,   // "ABORT"
               V_LAND   = 64'h0000_0000_4C41_4E44,   // "LAND"
               V_PING   = 64'h0000_0000_5049_4E47;   // "PING"

    localparam [2:0] M_ANGLE   = 3'd0,
                     M_ALTHOLD = 3'd1,
                     M_POSHOLD = 3'd2,
                     M_RTH     = 3'd3,
                     M_WP      = 3'd4;

    localparam S_KEY = 1'b0,
               S_VAL = 1'b1;

    reg        state;
    reg [31:0] key_reg;
    reg [63:0] val_reg;
    reg [23:0] num;              // רוויה ב-9,999,999 - מספר ארוך לא "מתגלגל" לערך קטן
    reg        neg;
    reg        line_is_ctrl;
    reg        line_is_sens;
    // מצערת של השורה הנוכחית - ננעלת ל-thr רק בסוף השורה
    reg  [7:0] thr_line;       // מ-THR, כבר ×2
    reg  [7:0] th2_line;       // מ-TH2
    reg        line_has_thr;
    reg        line_has_th2;

    // ערכי שליטה מוגבלים ל-±100, מרחקים עד 400
    wire [7:0]        mag  = (num > 24'd100) ? 8'd100 : num[7:0];
    wire [7:0]        half = (num > 24'd200) ? 8'd200 : num[7:0];   // TH2, חצאי אחוז
    wire signed [7:0] sval = neg ? -$signed(mag) : $signed(mag);
    wire [8:0]        dist = (num > 24'd400) ? 9'd400 : num[8:0];
    wire [2:0]        st3  = (num > 24'd4)   ? 3'd0   : num[2:0];
    wire [7:0]        seq8 = num[7:0];
    wire [1:0]        mgh2 = (num > 24'd2)   ? 2'd0   : num[1:0];   // לא מוכר ⇒ לא מאומת

    wire [23:0] num_next = (num > 24'd999_999) ? num
                         : (num * 24'd10) + {16'd0, rx_data - CH_ZERO};

    wire pair_is_ctrl = (key_reg == K_THR)  || (key_reg == K_TH2) || (key_reg == K_YAW) ||
                        (key_reg == K_PCH)  || (key_reg == K_RLL) ||
                        (key_reg == K_MODE);

    wire pair_is_sens = (key_reg == K_FWD)  || (key_reg == K_DWN)  ||
                        (key_reg == K_FOK)  || (key_reg == K_DOK)  ||
                        (key_reg == K_FSEQ) || (key_reg == K_DSEQ) ||
                        (key_reg == K_MGH)  || (key_reg == K_SRF) || (key_reg == K_THS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_KEY;
            key_reg       <= 32'd0;
            val_reg       <= 64'd0;
            num           <= 24'd0;
            neg           <= 1'b0;
            line_is_ctrl  <= 1'b0;
            line_is_sens  <= 1'b0;
            thr_line      <= 8'd0;
            th2_line      <= 8'd0;
            line_has_thr  <= 1'b0;
            line_has_th2  <= 1'b0;
            thr           <= 8'd0;
            yaw           <= 8'sd0;
            pch           <= 8'sd0;
            rll           <= 8'sd0;
            mode          <= M_ANGLE;        // אחרי איפוס: ANGLE, המצב הבטוח
            srf           <= 1'b0;           // אחרי איפוס: בלי SURFACE
            ths           <= 8'hFF;          // אחרי איפוס: אין מרכז (אין החזקה ביציאה)
            lidar_fwd     <= 9'd0;
            lidar_dwn     <= 9'd0;
            lidar_fwd_st  <= 3'd0;
            lidar_dwn_st  <= 3'd0;
            lidar_fwd_seq <= 8'd0;
            lidar_dwn_seq <= 8'd0;
            mag_health    <= 2'd0;           // אחרי איפוס: לא מאומת
            ctrl_valid    <= 1'b0;
            sens_valid    <= 1'b0;
            cmd_arm       <= 1'b0;
            cmd_disarm    <= 1'b0;
            cmd_check     <= 1'b0;
            cmd_abort     <= 1'b0;
            cmd_land      <= 1'b0;
            cmd_ping      <= 1'b0;
        end else begin
            // פולסים - מחזור אחד בלבד
            ctrl_valid <= 1'b0;
            sens_valid <= 1'b0;
            cmd_arm    <= 1'b0;
            cmd_disarm <= 1'b0;
            cmd_check  <= 1'b0;
            cmd_abort  <= 1'b0;
            cmd_land   <= 1'b0;
            cmd_ping   <= 1'b0;

            if (rx_valid) begin

                // ═══ צבירת שם המפתח ═══
                if (state == S_KEY) begin
                    if (rx_data == CH_COLON) begin
                        state   <= S_VAL;
                        val_reg <= 64'd0;
                        num     <= 24'd0;
                        neg     <= 1'b0;
                    end
                    else if (rx_data == CH_LF) begin
                        if (line_is_ctrl) ctrl_valid <= 1'b1;
                        if (line_is_sens) sens_valid <= 1'b1;
                        // נעילת המצערת של השורה: TH2 גובר על THR
                        if (line_has_th2)      thr <= th2_line;
                        else if (line_has_thr) thr <= thr_line;
                        line_has_thr <= 1'b0;
                        line_has_th2 <= 1'b0;
                        line_is_ctrl <= 1'b0;
                        line_is_sens <= 1'b0;
                        key_reg      <= 32'd0;
                    end
                    else if (rx_data == CH_SPACE || rx_data == CH_CR) begin
                        key_reg <= 32'd0;
                    end
                    else begin
                        key_reg <= {key_reg[23:0], rx_data};
                    end
                end

                // ═══ צבירת הערך ═══
                else begin
                    if (rx_data == CH_SPACE || rx_data == CH_LF || rx_data == CH_CR) begin

                        case (key_reg)
                            // נשמרים לשורה ונעולים ל-thr רק ב-LF
                            K_THR : begin thr_line <= {mag[6:0], 1'b0}; line_has_thr <= 1'b1; end  // ×2
                            K_TH2 : begin th2_line <= half;             line_has_th2 <= 1'b1; end
                            K_YAW : yaw <= sval;
                            K_PCH : pch <= sval;
                            K_RLL : rll <= sval;

                            K_FWD : lidar_fwd     <= dist;
                            K_DWN : lidar_dwn     <= dist;
                            K_FOK : lidar_fwd_st  <= st3;
                            K_DOK : lidar_dwn_st  <= st3;
                            K_FSEQ: lidar_fwd_seq <= seq8;
                            K_DSEQ: lidar_dwn_seq <= seq8;
                            K_MGH : mag_health    <= mgh2;
                            K_SRF : srf           <= (num == 24'd1);
                            K_THS : ths           <= (!neg && num <= 24'd200) ? num[7:0] : 8'hFF;

                            // המצב מתעדכן פעם אחת, בסוף הערך השלם - כך אין רגע
                            // שבו מוצג ערך ביניים, והמעבר ב-CH7 הוא קפיצה ישירה
                            K_MODE: begin
                                case (val_reg)
                                    V_ANGLE  : mode <= M_ANGLE;
                                    V_ALTHOLD: mode <= M_ALTHOLD;
                                    V_POSHOLD: mode <= M_POSHOLD;
                                    V_RTH    : mode <= M_RTH;
                                    V_WP     : mode <= M_WP;
                                    default  : ;     // לא מוכר: המצב הקודם נשמר
                                endcase
                            end

                            K_CMD : begin
                                case (val_reg)
                                    V_ARM    : cmd_arm    <= 1'b1;
                                    V_DISARM : cmd_disarm <= 1'b1;
                                    V_CHECK  : cmd_check  <= 1'b1;
                                    V_ABORT  : cmd_abort  <= 1'b1;
                                    V_LAND   : cmd_land   <= 1'b1;
                                    V_PING   : cmd_ping   <= 1'b1;
                                    default  : ;
                                endcase
                            end

                            default: ;
                        endcase

                        if (rx_data == CH_LF) begin
                            if (line_is_ctrl || pair_is_ctrl) ctrl_valid <= 1'b1;
                            if (line_is_sens || pair_is_sens) sens_valid <= 1'b1;
                            // נעילת המצערת של השורה, כולל הזוג שמסתיים כאן: TH2 גובר על THR
                            if (key_reg == K_TH2)      thr <= half;
                            else if (line_has_th2)     thr <= th2_line;
                            else if (key_reg == K_THR) thr <= {mag[6:0], 1'b0};
                            else if (line_has_thr)     thr <= thr_line;
                            line_has_thr <= 1'b0;
                            line_has_th2 <= 1'b0;
                            line_is_ctrl <= 1'b0;
                            line_is_sens <= 1'b0;
                        end
                        else begin
                            if (pair_is_ctrl) line_is_ctrl <= 1'b1;
                            if (pair_is_sens) line_is_sens <= 1'b1;
                        end

                        key_reg <= 32'd0;
                        state   <= S_KEY;
                    end
                    else if (rx_data == CH_MINUS) begin
                        neg <= 1'b1;
                    end
                    else if (rx_data >= CH_ZERO && rx_data <= CH_NINE) begin
                        num     <= num_next;
                        val_reg <= {val_reg[55:0], rx_data};
                    end
                    else begin
                        val_reg <= {val_reg[55:0], rx_data};
                    end
                end

            end
        end
    end

endmodule
