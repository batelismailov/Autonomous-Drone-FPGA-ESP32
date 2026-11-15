// ═══════════════════════════════════════════════════════════
//   mission_rx.v - פענוח שורות המשימה מה-ESP32
//
//   נבנה תמיד (MISSION_ENABLE=1 ב-drone_top). המשימה עצמה רדומה עד START.
//
//   המודול מאזין לאותו זרם בתים כמו cmd_parser, ומפענח **רק** מפתחות חדשים.
//   cmd_parser לא שונה: מפתח שאינו מכיר נופל אצלו ל-default ואינו נוגע בשום
//   מוצא, ושורה שאין בה אף מפתח שלו אינה ctrl_valid ואינה sens_valid.
//
//   כל מפתח כאן הוא בדיוק 3 תווים, ונשמרים 3 התווים האחרונים של שם המפתח.
//   אף מפתח כאן אינו "SEQ" או "ODE", ולכן FSEQ / DSEQ / MODE לא מתבלבלים.
//   המפתח נשמר כ-5 הביטים הנמוכים של כל תו (15 ביט): חד-ערכי לאותיות גדולות.
//   הכינוי היחיד בזרם הוא THR ≡ TH2, ושניהם אינם מפתחות של המודול הזה.
//
//   שורת INAV מהירה (10Hz), כל ערך מספר עשרוני עם סימן:
//     "NPX:x NPY:y NPZ:z NVX:vx NVY:vy NVZ:vz NPQ:s
//      NMD:m NST:st NWP:wp NER:e NNQ:s NFM:f NAR:a NFQ:s
//      GFX:f GSA:n GEP:eph GSQ:s"
//     NP*   MSP2_INAV_FULL_LOCAL_POSE, cm, צירי NEU: X צפון · Y מזרח · Z למעלה
//           (חיובי = גבוה יותר - אומת בקוד INAV של ה-build).
//           ⚠️ X / Y נשלחים **מגולגלים** ל-int16 (ערך מודולו 65536). ה-FSM צריך
//              רק הפרשים של כמה מטרים, וחשבון 16 ביט מגולגל נותן הפרש נכון
//              כל עוד ההפרש האמיתי קטן מ-327m. Z מוגבל ל-±32767.
//     NV*   מהירות cm/s, מוגבלת כאן ל-±2047
//     NMD..NER  MSP_NAV_STATUS: mode · state · active WP (מ-1) · error
//     NFM / NAR MSP2_INAV_STATUS: ביטי FM_* של ה-ESP · ARMED מ-armingFlags
//     GFX..GEP  MSP_RAW_GPS / MSP_GPSSTATISTICS: Fix (2=3D) · לוויינים · EPH cm
//     *Q    מוני דגימות - עולים רק כשהגיעה תשובת MSP חדשה. ה-FSM מודד טריות
//           **רק** לפיהם, לא לפי הגעת השורה. נשמרים 4 הביטים הנמוכים: מספיק
//           כדי לזהות שינוי.
//
//   שורה איטית (2Hz): "MIS:c CAT:a CUC:u CRS:r CAS:v BTS:b SLQ:s"
//     MIS   תוצאת אימות המשימה ב-ESP (1 = תואמת למשימה הצפויה)
//     CAT / CUC / CRS / CAS   nav_mc_althold_throttle · nav_user_control_mode ·
//           nav_wp_mission_restart · nav_auto_speed. לא ידוע ⇒ 3 / 2047.
//     BTS   מצב הסוללה: 0 OK · 1 WARN · 2 CRIT · 3 NONE
//
//   שדות בשורה האיטית גם: "CTM:m MAL:a"
//     CTM   thr_mid של INAV באחוזים (הנייטרל של POSHOLD בהמראה מהקרקע). 0 = לא ידוע.
//     MAL   גובה ה-WP הראשון של המשימה, cm יחסית לבית. 0 = לא ידוע / לא תקין.
//     CAS   כאן: המהירות האפקטיבית המרבית של WP במשימה (ה-ESP מחשב אותה
//           מ-p1/p2, nav_auto_speed ו-nav_max_auto_speed), לא nav_auto_speed הגולמי.
//
//   שורת השליטה: "RPL:0/1" - הגבלת Roll/Pitch ל-60% ב-ANGLE/ALTHOLD (ברירת מחדל 0 - כבויה עד בקשה מפורשת).
//
//   שורת הגדרות העקיפה (מה-ESP, כל שנייה ובכל שינוי), 9 שדות בסדר קבוע:
//     "MSE:d,c,v,f,a,o,s,l,k"
//     d מרחק זיהוי · c מרחק FRONT פנוי · v מרווח אנכי · f מרווח אופקי ·
//     a תקרת גובה · o עומק המכשול (cm) · s שלב בדיקה · l דגלים (bit0 המראה) ·
//     k = Σ (9−i)·v_i (i=1..8) - Fletcher, רגיש לסדר ולהחלפה.
//     הערכים נכתבים ישירות לרגיסטרים שה-FSM משתמש בהם - אבל רק כש-set_allow
//     (המשימה לא פעילה וה-FPGA מנוטרל) היה 1 מתחילת השורה ועד סופה. בתחילת
//     שורה כזו set_valid יורד, ועולה רק אם הגיעו בדיוק 8 ערכים ו-k תואם.
//     כש-set_allow=0 השורה כולה מתעלמת: הערכים ו-set_valid לא משתנים.
//     אחרי הדלקה: ברירות המחדל למטה, ו-set_valid=0 עד הסט הראשון מה-ESP.
//     ה-FSM מחליט אם להחיל (רק כשאינו פעיל וה-FPGA מנוטרל) ומאמת טווחים.
//
//   שדות בסוף שורת החיישנים: "PHS:p FWA:a FRC:c DRC:c FLM:m DLM:m"
//     FLM / DLM  מצב המדידה שהופעל בפועל בחיישן: 0 SHORT · 1 LONG
//     PHS / FWA נשלחים כבר היום, ו-cmd_parser מתעלם מהם.
//     FRC / DRC - סיווג הדגימה של FWD / DWN (אותה דגימה של FSEQ / DSEQ):
//       0 FAULT · 1 VALID · 2 OUT_OF_RANGE · 3 WEAK_TARGET · 4 UNCERTAIN
//       5 TOO_CLOSE · 6 NO_SIGNAL. ערך לא מוכר ⇒ FAULT.
//
//   פקודות: "CMD:MSTART" · "CMD:MSTOP" · "CMD:MCROSS" · "CMD:MSTAT"
//     cmd_parser אינו מכיר את הערכים האלה, ולכן הם אינם any_cmd אצלו.
//
//   ⚠️ כמו ב-cmd_parser, אין כאן checksum. ההגנה היא ברמת ה-FSM: מעבר מצב
//      נשען על כמה דגימות רצופות, ומטרות גובה מאושרות בשתי דגימות.
// ═══════════════════════════════════════════════════════════
module mission_rx (
    input  wire               clk,
    input  wire               rst_n,

    input  wire [7:0]         rx_data,
    input  wire               rx_valid,

    // ── MSP2_INAV_FULL_LOCAL_POSE ──
    output reg  signed [15:0] pos_x,      // מגולגל
    output reg  signed [15:0] pos_y,      // מגולגל
    output reg  signed [15:0] pos_z,
    output reg  signed [11:0] vel_x,
    output reg  signed [11:0] vel_y,
    output reg  signed [11:0] vel_z,
    output reg  [3:0]         pose_seq,

    // ── MSP_NAV_STATUS ──
    output reg  [3:0]         nav_mode,
    output reg  [4:0]         nav_state,
    output reg  [5:0]         nav_wp,
    output reg  [3:0]         nav_err,
    output reg  [3:0]         nav_seq,

    // ── MSP2_INAV_STATUS ──
    output reg  [10:0]        fm_flags,
    output reg                inav_armed,
    output reg  [3:0]         flg_seq,

    // ── GPS ──
    output reg  [1:0]         gps_fix,
    output reg  [5:0]         gps_sats,
    output reg  [9:0]         gps_eph,    // cm, רווי ב-1023
    output reg  [3:0]         gps_seq,

    // ── השורה האיטית ──
    output reg  [3:0]         mis_code,
    output reg  [1:0]         cfg_atm,
    output reg  [1:0]         cfg_ucm,
    output reg  [1:0]         cfg_rst,
    output reg  [10:0]        cfg_aspd,   // cm/s, רווי ב-2047
    output reg  [1:0]         batt_st,
    output reg  [3:0]         slow_seq,
    output reg  [6:0]         thr_mid,    // % - 0 = לא ידוע
    output reg  [10:0]        mis_alt,    // cm - 0 = לא ידוע

    // ── שדות שורת החיישנים ──
    output reg  [1:0]         phase,
    output reg                fwd_active,
    output reg  [2:0]         fwd_cls,
    output reg  [2:0]         dwn_cls,
    output reg                fwd_long,
    output reg                dwn_long,
    output reg                rpl_en,     // הגבלת Roll/Pitch, ברירת מחדל 0

    // ── הגדרות העקיפה (סט אחד, אחרי בדיקת SCK) ──
    input  wire               set_allow,
    output reg  [9:0]         set_stop,
    output reg  [9:0]         set_clear,
    output reg  [9:0]         set_vclr,
    output reg  [9:0]         set_fclr,
    output reg  [9:0]         set_ceil,
    output reg  [9:0]         set_depth,
    output reg  [2:0]         set_stage,
    output reg  [1:0]         set_flags,
    output reg  [15:0]        set_ck,
    output reg                set_valid,

    // ── פקודות (פולס מחזור אחד) ──
    output reg                cmd_mstart,
    output reg                cmd_mstop,
    output reg                cmd_mcross,
    output reg                cmd_mstat
);

    localparam CH_COLON = 8'h3A,
               CH_SPACE = 8'h20,
               CH_LF    = 8'h0A,
               CH_CR    = 8'h0D,
               CH_MINUS = 8'h2D,
               CH_ZERO  = 8'h30,
               CH_NINE  = 8'h39;

    localparam [14:0] K_NPX = 15'h3A18, K_NPY = 15'h3A19, K_NPZ = 15'h3A1A,
                      K_NVX = 15'h3AD8, K_NVY = 15'h3AD9, K_NVZ = 15'h3ADA,
                      K_NPQ = 15'h3A11,
                      K_NMD = 15'h39A4, K_NST = 15'h3A74, K_NWP = 15'h3AF0,
                      K_NER = 15'h38B2, K_NNQ = 15'h39D1,
                      K_NFM = 15'h38CD, K_NAR = 15'h3832, K_NFQ = 15'h38D1,
                      K_GFX = 15'h1CD8, K_GSA = 15'h1E61, K_GEP = 15'h1CB0,
                      K_GSQ = 15'h1E71,
                      K_MIS = 15'h3533,
                      K_CAT = 15'h0C34, K_CUC = 15'h0EA3, K_CRS = 15'h0E53,
                      K_CAS = 15'h0C33, K_BTS = 15'h0A93, K_SLQ = 15'h4D91,
                      K_PHS = 15'h4113, K_FWA = 15'h1AE1,
                      K_FRC = 15'h1A43, K_DRC = 15'h1243,
                      K_FLM = 15'h198D, K_DLM = 15'h118D, K_RPL = 15'h4A0C,
                      K_CTM = 15'h0E8D, K_MAL = 15'h342C,
                      K_MSE = 15'h3665,
                      K_CMD = 15'h0DA4;

    // ערכי CMD - שישה התווים האחרונים
    localparam [47:0] V_MSTART = 48'h4D53_5441_5254,   // "MSTART"
                      V_MSTOP  = 48'h004D_5354_4F50,   // "MSTOP"
                      V_MCROSS = 48'h4D43_524F_5353,   // "MCROSS"
                      V_MSTAT  = 48'h004D_5354_4154;   // "MSTAT"

    localparam S_KEY = 1'b0,
               S_VAL = 1'b1;

    reg        state;
    reg [14:0] key_reg;   // 5 הביטים הנמוכים של כל אות: חד-ערכי לאותיות גדולות
    reg [47:0] val_reg;
    reg [16:0] num;          // רוויה ב-99,999 - מספר ארוך לא "מתגלגל" לערך קטן
    reg        neg;

    // ── שורת ההגדרות: מספר השדה וסכום Fletcher ──
    reg  [3:0]  f_idx;
    reg  [13:0] s_sum1;
    reg  [15:0] s_sum2;
    reg         s_line;       // שורת הגדרות שמותר להחיל (set_allow בתחילתה)
    reg         s_bad;        // set_allow ירד באמצע השורה
    wire [9:0]  v10   = (num > 17'd1023) ? 10'd1023 : num[9:0];
    wire [13:0] sum1n = s_sum1 + {4'd0, v10};
    wire [15:0] sum2n = s_sum2 + {2'd0, sum1n};

    wire [16:0] num_next = (num > 17'd9_999) ? 17'd99_999
                         : (num * 17'd10) + {13'd0, rx_data[3:0]};

    // ── המרות עם רוויה ──
    wire [15:0]        mag15 = (num > 17'd32_767) ? 16'd32_767 : num[15:0];
    wire signed [15:0] s16   = neg ? -$signed(mag15) : $signed(mag15);
    wire [11:0]        mag11 = (num > 17'd2_047) ? 12'd2_047 : num[11:0];
    wire signed [11:0] s12   = neg ? -$signed(mag11) : $signed(mag11);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_KEY;
            key_reg    <= 15'd0;
            val_reg    <= 48'd0;
            num        <= 17'd0;
            neg        <= 1'b0;
            pos_x      <= 16'sd0;  pos_y <= 16'sd0;  pos_z <= 16'sd0;
            vel_x      <= 12'sd0;  vel_y <= 12'sd0;  vel_z <= 12'sd0;
            pose_seq   <= 4'd0;
            nav_mode   <= 4'd0;
            nav_state  <= 5'd0;
            nav_wp     <= 6'd0;
            nav_err    <= 4'd0;
            nav_seq    <= 4'd0;
            fm_flags   <= 11'd0;
            inav_armed <= 1'b0;
            flg_seq    <= 4'd0;
            gps_fix    <= 2'd0;
            gps_sats   <= 6'd0;
            gps_eph    <= 10'd1023;
            gps_seq    <= 4'd0;
            mis_code   <= 4'd0;
            cfg_atm    <= 2'd3;          // לא ידוע
            cfg_ucm    <= 2'd3;
            cfg_rst    <= 2'd3;
            cfg_aspd   <= 11'd2047;
            batt_st    <= 2'd3;
            slow_seq   <= 4'd0;
            thr_mid    <= 7'd0;
            mis_alt    <= 11'd0;
            fwd_long   <= 1'b0;
            dwn_long   <= 1'b0;
            rpl_en     <= 1'b0;          // כבויה עד שהאפליקציה מבקשת RPL:1
            f_idx      <= 4'd0;  s_sum1 <= 14'd0;  s_sum2 <= 16'd0;
            s_line     <= 1'b0;  s_bad  <= 1'b0;
            set_stop   <= 10'd100; set_clear <= 10'd150; set_vclr <= 10'd70; set_fclr <= 10'd70;
            set_ceil   <= 10'd400; set_depth <= 10'd50;  set_stage <= 3'd2;  set_flags <= 2'd0;
            set_ck     <= 16'd0;
            set_valid  <= 1'b0;
            phase      <= 2'd0;
            fwd_active <= 1'b0;
            fwd_cls    <= 3'd0;          // FAULT
            dwn_cls    <= 3'd0;
            cmd_mstart <= 1'b0;
            cmd_mstop  <= 1'b0;
            cmd_mcross <= 1'b0;
            cmd_mstat  <= 1'b0;
        end else begin
            cmd_mstart <= 1'b0;
            cmd_mstop  <= 1'b0;
            cmd_mcross <= 1'b0;
            cmd_mstat  <= 1'b0;

            if (rx_valid) begin
                if (state == S_KEY) begin
                    if (rx_data == CH_COLON) begin
                        state   <= S_VAL;
                        val_reg <= 48'd0;
                        num     <= 17'd0;
                        neg     <= 1'b0;
                        f_idx   <= 4'd0;
                        s_sum1  <= 14'd0;
                        s_sum2  <= 16'd0;
                        s_bad   <= 1'b0;
                        s_line  <= (key_reg == K_MSE) && set_allow;
                        if ((key_reg == K_MSE) && set_allow) set_valid <= 1'b0;
                    end
                    else if (rx_data == CH_SPACE || rx_data == CH_CR || rx_data == CH_LF) begin
                        key_reg <= 15'd0;
                    end
                    else begin
                        key_reg <= {key_reg[9:0], rx_data[4:0]};
                    end
                end
                else begin
                    if (rx_data == CH_SPACE || rx_data == CH_LF || rx_data == CH_CR) begin
                        case (key_reg)
                            K_NPX: pos_x    <= s16;
                            K_NPY: pos_y    <= s16;
                            K_NPZ: pos_z    <= s16;
                            K_NVX: vel_x    <= s12;
                            K_NVY: vel_y    <= s12;
                            K_NVZ: vel_z    <= s12;
                            K_NPQ: pose_seq <= num[3:0];

                            K_NMD: nav_mode  <= (num > 17'd15) ? 4'd15 : num[3:0];
                            K_NST: nav_state <= (num > 17'd31) ? 5'd31 : num[4:0];
                            K_NWP: nav_wp    <= (num > 17'd63) ? 6'd63 : num[5:0];
                            K_NER: nav_err   <= (num > 17'd15) ? 4'd15 : num[3:0];
                            K_NNQ: nav_seq   <= num[3:0];

                            K_NFM: fm_flags   <= (num > 17'd2047) ? 11'd0 : num[10:0];
                            K_NAR: inav_armed <= (num == 17'd1);
                            K_NFQ: flg_seq    <= num[3:0];

                            K_GFX: gps_fix  <= (num > 17'd2)    ? 2'd0     : num[1:0];
                            K_GSA: gps_sats <= (num > 17'd63)   ? 6'd63    : num[5:0];
                            K_GEP: gps_eph  <= (num > 17'd1023) ? 10'd1023 : num[9:0];
                            K_GSQ: gps_seq  <= num[3:0];

                            K_MIS: mis_code <= (num > 17'd15)   ? 4'd15    : num[3:0];
                            K_CAT: cfg_atm  <= (num > 17'd2)    ? 2'd3     : num[1:0];
                            K_CUC: cfg_ucm  <= (num > 17'd2)    ? 2'd3     : num[1:0];
                            K_CRS: cfg_rst  <= (num > 17'd2)    ? 2'd3     : num[1:0];
                            K_CAS: cfg_aspd <= (num > 17'd2047) ? 11'd2047 : num[10:0];
                            K_BTS: batt_st  <= (num > 17'd3)    ? 2'd3     : num[1:0];
                            K_SLQ: slow_seq <= num[3:0];

                            K_PHS: phase      <= (num > 17'd3) ? 2'd0 : num[1:0];
                            K_FWA: fwd_active <= (num == 17'd1);
                            K_FRC: fwd_cls    <= (num > 17'd6) ? 3'd0 : num[2:0];
                            K_DRC: dwn_cls    <= (num > 17'd6) ? 3'd0 : num[2:0];
                            K_FLM: fwd_long   <= (num == 17'd1);
                            K_DLM: dwn_long   <= (num == 17'd1);
                            K_RPL: rpl_en     <= (num != 17'd0);
                            K_CTM: thr_mid    <= (num > 17'd100) ? 7'd0  : num[6:0];
                            K_MAL: mis_alt    <= (num > 17'd2047) ? 11'd0 : num[10:0];

                            // סוף שורת ההגדרות: 8 ערכים, ואז ה-SCK
                            K_MSE: if (s_line && set_allow && !s_bad) begin
                                       if (f_idx == 4'd8 && {1'b0, s_sum2} == num) begin
                                           set_valid <= 1'b1;
                                           set_ck    <= s_sum2;
                                       end
                                   end

                            K_CMD: begin
                                case (val_reg)
                                    V_MSTART: cmd_mstart <= 1'b1;
                                    V_MSTOP : cmd_mstop  <= 1'b1;
                                    V_MCROSS: cmd_mcross <= 1'b1;
                                    V_MSTAT : cmd_mstat  <= 1'b1;
                                    default : ;
                                endcase
                            end

                            default: ;
                        endcase
                        key_reg <= 15'd0;
                        state   <= S_KEY;
                    end
                    // פסיק בשורת ההגדרות: הערך הבא
                    else if (rx_data == 8'h2C && key_reg == K_MSE) begin
                        if (!set_allow) s_bad <= 1'b1;
                        if (s_line && set_allow) case (f_idx)
                            4'd0: set_stop  <= v10;
                            4'd1: set_clear <= v10;
                            4'd2: set_vclr  <= v10;
                            4'd3: set_fclr  <= v10;
                            4'd4: set_ceil  <= v10;
                            4'd5: set_depth <= v10;
                            4'd6: set_stage <= (num > 17'd7) ? 3'd0 : num[2:0];
                            4'd7: set_flags <= (num > 17'd3) ? 2'd3 : num[1:0];
                            default: ;
                        endcase
                        if (f_idx != 4'd15) f_idx <= f_idx + 4'd1;
                        s_sum1 <= sum1n;
                        s_sum2 <= sum2n;
                        num    <= 17'd0;
                    end
                    else begin
                        val_reg <= {val_reg[39:0], rx_data};
                        if (rx_data == CH_MINUS)                              neg <= 1'b1;
                        else if (rx_data >= CH_ZERO && rx_data <= CH_NINE)    num <= num_next;
                    end
                end
            end
        end
    end

endmodule
