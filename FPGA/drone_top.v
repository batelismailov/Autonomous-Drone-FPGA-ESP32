// ═══════════════════════════════════════════════════════════
//   drone_top.v - המודול הראשי
//
//   ה-FPGA בונה את חבילות ה-RC ל-Matek ומחליט מתי להפסיק לשלוח אותן.
//   אין כאן בקרת גובה, אין נחיתה ואין הורדת מצערת: כל אלה באחריות INAV.
//
//   ═══ Mission FSM - כלול תמיד, רדום עד START ═══
//   mission_rx / mission_fsm / mission_tx נבנים תמיד (MISSION_ENABLE=1).
//   אחרי הדלקה ה-FSM ב-IDLE ו-m_active=0: בקשת האפליקציה עוברת כמו שהיא.
//   משימה מתחילה רק ב-CMD:MSTART שעבר את כל 16 בדיקות ה-SELF_CHECK.
//   הגדרות העקיפה ושלב הבדיקה נקבעים בזמן ריצה מהאפליקציה (דרך ה-ESP),
//   ומוחלים רק כשה-FPGA מנוטרל והמשימה לא פעילה - אין צורך בצריבה.
//
//   ═══ כפתור WP ═══
//   WP ידני אינו קיים: בקשת MODE:WP פירושה "משימה נבחרה". כשה-FSM אינו פעיל
//   היא יוצאת ל-CH7 כ-ANGLE על הקרקע (כדי ש-INAV יאפשר חימוש) וכ-POSHOLD
//   באוויר. WP אמיתי ב-CH7 - רק מה-Mission FSM.
//
//   ═══ הגבלת Roll/Pitch ═══
//   ב-ANGLE וב-ALTHOLD בלבד (לפי CH7 שיוצא בפועל, ובתנאי ש-INAV אינו מדווח
//   POSHOLD/RTH/WP): RC = 1500 + 0.75·(RC − 1500), כלומר 1125..1875.
//   YAW והמצערת לא משתנים. ביטול/הפעלה מהאפליקציה (RPL), במעבר הדרגתי.
// ═══════════════════════════════════════════════════════════
module drone_top #(
    parameter CLK_FREQ        = 50_000_000,
    parameter BAUD_RATE       = 115_200,
    parameter MSP_RATE_HZ     = 50,
    parameter LINK_LOSS_MS    = 1000,
    parameter ESP_DEAD_MS     = 1500,
    parameter LED_ACTIVE_LOW  = 1,
    parameter MISSION_ENABLE  = 1,     // 1 = Mission FSM כלול (רדום עד START)
    // עדות הגובה להמראה ידנית כש-WP נבחר (F1021E: 110/100 ⇒ 90/80, ראו למטה)
    parameter WP_HI_CM        = 90,    // שלב 2: דגימה תקינה אחת לפחות בגובה הזה
    parameter WP_HOLD_CM      = 80,    // שלב 3: כל דגימה תקינה לפחות בגובה הזה
    // F1022A: SURFACE ידני על CH8. 0 ⇒ CH8 קבוע 1000 ו-CH7 בלי שינוי, בדיוק כמו F1021E
    parameter SURF_ENABLE     = 1,
    parameter SRF_PULSE_FRAMES = 10,   // חבילות RC של ALTHOLD בכל החלפה (10 × 20ms = 200ms)
    parameter SRF_LEAD_FRAMES  = 5     // F1022B: חבילות RC עם מצערת ההחזקה **לפני** היציאה (5 × 20ms = 100ms)
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
    wire              srf_req;        // F1022A: בקשת SURFACE מה-ESP (SRF:1)
    wire [7:0]        srf_ths;        // F1022B: מרכז המקל ב-SURFACE מה-ESP (THS), 0..200; אחר = אין
    wire              m_active_w;     // משימה פעילה (0 כש-MISSION_ENABLE=0)
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
    wire       dwn_fresh;      // מ-flight_fsm: דגימת DOWN חדשה ב-300ms האחרונים
    wire [7:0] esp_tx_data;
    wire       esp_tx_send, esp_uart_busy;

    wire       msp_pkt_busy;
    wire [7:0] fc_tx_data;
    wire       fc_tx_send, fc_uart_busy;

    // ═══ אחרי בורר המשימה: מה שנכנס לשער המצפן ולשער הבטיחות ═══
    //   MISSION_ENABLE=0 ⇒ אלה בדיוק thr_raw / yaw_raw / pch_raw / rll_raw / mode
    wire [7:0]        thr_sel;
    wire signed [7:0] yaw_sel, pch_sel, rll_sel;
    wire [2:0]        mode_sel;
    wire              rp_limit;       // הגבלת Roll/Pitch פעילה (ANGLE / ALTHOLD ידני)

    // ═══ ערוץ התשובה ל-ESP: flight_fsm (STATUS) ועוד, עם MISSION_ENABLE=1, mission_tx ═══
    wire [7:0] fsm_tx_data;
    wire       fsm_tx_send;
    wire       fsm_esp_busy;

    // ═══ שער בטיחות ═══
    //   מנוטרל ⇒ מצערת מינימלית והמקלות במרכז, בכל מקרה.
    wire [7:0] thr_fin;           // F1022B: thr_sel, או מצערת ההחזקה אחרי יציאה מ-SURFACE (3.6)
    wire [7:0] thr_safe = armed ? thr_fin : 8'd0;

    wire signed [7:0] yaw_safe = armed ? yaw_sel : 8'sd0;
    wire signed [7:0] pch_safe = armed ? pch_sel : 8'sd0;
    wire signed [7:0] rll_safe = armed ? rll_sel : 8'sd0;

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
        .srf           (srf_req),
        .ths           (srf_ths),
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

    // ═══ 2.5 - ה-Mission FSM, בין הפענוח לשער המצפן ═══
    //   סדר הבוררות: אפליקציה → Mission → שער המצפן (flight_fsm) → שער הבטיחות.
    generate
        if (MISSION_ENABLE != 0) begin : g_mission

            wire signed [15:0] m_pos_x, m_pos_y, m_pos_z;
            wire signed [11:0] m_vel_x, m_vel_y, m_vel_z;
            wire [3:0]  m_pose_seq, m_nav_seq, m_flg_seq, m_gps_seq, m_slow_seq;
            wire [5:0]  m_nav_wp;
            wire [3:0]  m_nav_mode, m_nav_err, m_mis_code;
            wire [4:0]  m_nav_state;
            wire [10:0] m_fm_flags;
            wire        m_inav_armed, m_fwd_active;
            wire [1:0]  m_gps_fix, m_cfg_atm, m_cfg_ucm, m_cfg_rst, m_batt_st, m_phase;
            wire [5:0]  m_gps_sats;
            wire [9:0]  m_gps_eph;
            wire [10:0] m_cfg_aspd;
            wire [2:0]  m_fwd_cls, m_dwn_cls;
            wire        m_cmd_start, m_cmd_stop, m_cmd_cross, m_cmd_stat;
            wire [6:0]  m_thr_mid;
            wire [10:0] m_mis_alt;
            wire        m_fwd_long, m_dwn_long, m_rpl_en;
            wire [9:0]  m_s_stop, m_s_clear, m_s_vclr, m_s_fclr, m_s_ceil, m_s_depth;
            wire [2:0]  m_s_stage;
            wire [1:0]  m_s_flags;
            wire [15:0] m_s_ck;
            wire        m_s_valid, m_s_allow;

            mission_rx u_mrx (
                .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid),
                .pos_x(m_pos_x), .pos_y(m_pos_y), .pos_z(m_pos_z),
                .vel_x(m_vel_x), .vel_y(m_vel_y), .vel_z(m_vel_z), .pose_seq(m_pose_seq),
                .nav_mode(m_nav_mode), .nav_state(m_nav_state), .nav_wp(m_nav_wp),
                .nav_err(m_nav_err), .nav_seq(m_nav_seq),
                .fm_flags(m_fm_flags), .inav_armed(m_inav_armed), .flg_seq(m_flg_seq),
                .gps_fix(m_gps_fix), .gps_sats(m_gps_sats), .gps_eph(m_gps_eph), .gps_seq(m_gps_seq),
                .mis_code(m_mis_code),
                .cfg_atm(m_cfg_atm), .cfg_ucm(m_cfg_ucm), .cfg_rst(m_cfg_rst), .cfg_aspd(m_cfg_aspd),
                .batt_st(m_batt_st), .slow_seq(m_slow_seq),
                .thr_mid(m_thr_mid), .mis_alt(m_mis_alt),
                .phase(m_phase), .fwd_active(m_fwd_active), .fwd_cls(m_fwd_cls), .dwn_cls(m_dwn_cls),
                .fwd_long(m_fwd_long), .dwn_long(m_dwn_long), .rpl_en(m_rpl_en),
                .set_stop(m_s_stop), .set_clear(m_s_clear), .set_vclr(m_s_vclr), .set_fclr(m_s_fclr),
                .set_ceil(m_s_ceil), .set_depth(m_s_depth), .set_stage(m_s_stage), .set_flags(m_s_flags),
                .set_allow(m_s_allow), .set_ck(m_s_ck), .set_valid(m_s_valid),
                .cmd_mstart(m_cmd_start), .cmd_mstop(m_cmd_stop),
                .cmd_mcross(m_cmd_cross), .cmd_mstat(m_cmd_stat)
            );

            wire              m_active;
            wire [7:0]        m_thr;
            wire signed [7:0] m_yaw, m_pch, m_rll;
            wire [2:0]        m_mode;
            wire [4:0]        m_st;
            wire [7:0]        m_reason;
            wire [5:0]        m_saved_wp;
            wire [15:0]       m_checks;
            wire [9:0]        m_det_cm, m_stop_cm;
            wire [3:0]        m_stage;
            wire [9:0]        m_det_v, m_t_stop, m_trav;
            wire [7:0]        m_t_ack, m_rebound, m_neutral;
            wire [3:0]        m_flags;

            mission_fsm #(
                .CLK_FREQ              (CLK_FREQ)
            ) u_mfsm (
                .clk(clk), .rst_n(rst_n),
                .thr_app(thr_raw), .yaw_app(yaw_raw), .pch_app(pch_raw), .rll_app(rll_raw), .mode_app(mode),
                .lidar_fwd(lidar_fwd), .lidar_dwn(lidar_dwn),
                .lidar_fwd_seq(lidar_fwd_seq), .lidar_dwn_seq(lidar_dwn_seq),
                .mag_health(mag_health), .sens_valid(sens_valid),
                .armed(armed), .rc_hold(rc_hold),
                .pos_x(m_pos_x), .pos_y(m_pos_y), .pos_z(m_pos_z),
                .vel_x(m_vel_x), .vel_y(m_vel_y), .vel_z(m_vel_z), .pose_seq(m_pose_seq),
                .nav_mode(m_nav_mode), .nav_state(m_nav_state), .nav_wp(m_nav_wp),
                .nav_err(m_nav_err), .nav_seq(m_nav_seq),
                .fm_flags(m_fm_flags), .inav_armed(m_inav_armed), .flg_seq(m_flg_seq),
                .gps_fix(m_gps_fix), .gps_sats(m_gps_sats), .gps_eph(m_gps_eph), .gps_seq(m_gps_seq),
                .mis_code(m_mis_code),
                .cfg_atm(m_cfg_atm), .cfg_ucm(m_cfg_ucm), .cfg_rst(m_cfg_rst), .cfg_aspd(m_cfg_aspd),
                .batt_st(m_batt_st), .slow_seq(m_slow_seq),
                .thr_mid(m_thr_mid), .mis_alt(m_mis_alt),
                .phase(m_phase), .fwd_active(m_fwd_active), .fwd_cls(m_fwd_cls), .dwn_cls(m_dwn_cls),
                .fwd_long(m_fwd_long), .dwn_long(m_dwn_long),
                .cmd_mstart(m_cmd_start), .cmd_mstop(m_cmd_stop), .cmd_mcross(m_cmd_cross),
                .set_stop(m_s_stop), .set_clear(m_s_clear), .set_vclr(m_s_vclr), .set_fclr(m_s_fclr),
                .set_ceil(m_s_ceil), .set_depth(m_s_depth), .set_stage(m_s_stage), .set_flags(m_s_flags),
                .set_valid(m_s_valid), .set_allow(m_s_allow),
                .m_active(m_active), .m_thr(m_thr), .m_yaw(m_yaw), .m_pch(m_pch), .m_rll(m_rll),
                .m_mode(m_mode),
                .st_code(m_st), .reason(m_reason), .checks(m_checks),
                .det_cm(m_det_cm), .stop_cm(m_stop_cm), .saved_wp(m_saved_wp),
                .stage_code(m_stage),
                .det_v(m_det_v), .t_ack(m_t_ack), .t_stop(m_t_stop), .trav(m_trav), .rebound(m_rebound),
                .flags_out(m_flags), .m_neutral(m_neutral)
            );

            // ── המרבב: צירופי בלבד. m_active=0 ⇒ בקשת האפליקציה עוברת כמו שהיא ──
            assign thr_sel  = m_active ? m_thr : thr_raw;
            assign yaw_sel  = m_active ? m_yaw : yaw_raw;
            assign pch_sel  = m_active ? m_pch : pch_raw;
            assign rll_sel  = m_active ? m_rll : rll_raw;
            // ═══ WP מהאפליקציה = "משימה נבחרה", בלי משימה פעילה ═══
            //   על הקרקע ובטיפוס הנמוך - ANGLE (המראה ידנית). POSHOLD באוויר רק אם:
            //     · הטייס כבר היה באוויר ב-ALTHOLD / POSHOLD / RTH (כמו קודם - בלי שינוי), או
            //     · משימה רצה באוויר (אחרי SUSPENDED / COMPLETE נשארים ב-POSHOLD, כמו קודם), או
            //     · יש עדות גובה מה-DOWN **עכשיו** (wp_hi_ok, למטה).
            //   ⚠️ למה: מתחת ל-~1m הברומטר משובש מזרם הפרופלורים (נמדד: עד 1.5m מתחת
            //      לקרקע). POSHOLD שנכנס שם נועל גובה שגוי, יורד, ו-INAV מנתק (3/3 ניסיונות).
            //
            //   עדות הגובה - נבנית רק ממדידות מרחק תקינות, ורק בזמן ש-WP נבחר:
            //     1. 3 דגימות DOWN טריות ברצף, VALID ≥ 80cm (wp_v_n).
            //     2. דגימה תקינה ≥ WP_HI_CM (90) אחרי שלב 1 (wp_vhi) - הרחפן באמת עלה.
            //     3. 16 דגימות **אחרי** שלב 2 (~0.9s), כולן "עדיין גבוה": VALID ≥ WP_HOLD_CM (80) או
            //        אין-החזר (מעל הטווח - מותר רק אחרי שלב 2). כלומר ברגע המעבר הרחפן עדיין גבוה.
            //     · VALID < 80 ⇒ איפוס מלא (גם אחרי שלב 2: ירד - צריך שוב 3 × ≥ 80, ≥ 90 ו-16 דגימות).
            //     · TOO_CLOSE / FAULT ⇒ איפוס מלא. אין-החזר לפני שלב 2 ⇒ איפוס מלא.
            //       UNCERTAIN / WEAK - לא משנים.
            //     · אין דגימת DOWN חדשה 300ms (dwn_fresh), שלב ≠ 3, או WP לא נבחר ⇒ איפוס:
            //       בחירה חוזרת של WP מתחילה עדות מאפס.
            //     (F1021E: 90/80 ולא 110/100. נמדד ב-20.10 + 21.10: מ-60cm ומעלה שגיאת הברומטר
            //      מול DOWN היא ±20cm (מתחת ל-40cm עד ‎-1.5m). ב-110 המראה מהירה (1.6m/s) לא קיבלה
            //      אף מדידה תקינה ≥ 110 לפני היציאה מהטווח ⇒ נשארה ANGLE. הרצה על 29 המראות אמיתיות:
            //      110/100 ⇒ 20/29 · 90/80 ⇒ 25/29, ובכולן המעבר קרה מעל טווח ה-SHORT.)
            //   ה-POSHOLD שנבחר כך ננעל (לא חוזרים ל-ANGLE באוויר בגלל קריאה נמוכה רגעית),
            //   אבל הנעילה משתחררת כשהטייס עובר ל-ANGLE / מצב אחר: חזרה ל-WP צריכה עדות עדכנית.
            //   ה-Z של INAV לא נבדק כאן: שורת המיקום נשלחת רק כשמשימה מוכנה.
            reg       wp_pos;
            reg [1:0] wp_v_n;
            reg [4:0] wp_h_n;
            reg       wp_vhi;
            reg [7:0] wp_dseq_q;
            // דגימה חדשה רק בסוף שורת החיישנים (sens_valid), כמו ב-mission_fsm: כך DSEQ, DWN
            // ו-DRC כבר כולם מאותה שורה (DSEQ מתעדכן באמצע השורה, לפני DRC).
            wire      wp_dwn_evt  = sens_valid && (lidar_dwn_seq != wp_dseq_q);
            wire      wp_dwn_valid= (m_dwn_cls == 3'd1);
            wire      wp_dwn_nr   = (m_dwn_cls == 3'd2) || (m_dwn_cls == 3'd6);
            wire      wp_dwn_neu  = (m_dwn_cls == 3'd3) || (m_dwn_cls == 3'd4);
            wire      wp_hi_ok    = wp_vhi && (wp_h_n >= 5'd16);
            wire      wp_from_nav = (mode == 3'd1) || (mode == 3'd2) || (mode == 3'd3);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wp_pos <= 1'b0; wp_v_n <= 2'd0; wp_h_n <= 5'd0; wp_vhi <= 1'b0; wp_dseq_q <= 8'd0;
                end else begin
                    if (sens_valid) wp_dseq_q <= lidar_dwn_seq;
                    // ── עדות הגובה ──
                    if (m_phase != 2'd3 || !dwn_fresh || mode != 3'd4) begin
                        wp_v_n <= 2'd0; wp_h_n <= 5'd0; wp_vhi <= 1'b0;
                    end else if (wp_dwn_evt) begin
                        if (wp_dwn_valid && (lidar_dwn >= 9'd80)) begin   // שלב 1 (≥ WP_HOLD_CM גם הוא)
                            if (wp_v_n != 2'd3) wp_v_n <= wp_v_n + 2'd1;
                            else if (!wp_vhi) begin
                                if (lidar_dwn >= WP_HI_CM) wp_vhi <= 1'b1;     // שלב 2; הספירה מתחילה אחריו
                            end
                            else if (lidar_dwn >= WP_HOLD_CM) begin
                                if (wp_h_n != 5'd31) wp_h_n <= wp_h_n + 5'd1;  // עדיין גבוה
                            end
                            else begin
                                wp_vhi <= 1'b0; wp_h_n <= 5'd0;               // מתחת ל-WP_HOLD_CM
                            end
                        end
                        else if (wp_dwn_nr) begin
                            if (!wp_vhi) begin wp_v_n <= 2'd0; wp_h_n <= 5'd0; end
                            else if (wp_h_n != 5'd31) wp_h_n <= wp_h_n + 5'd1;
                        end
                        else if (!wp_dwn_neu) begin
                            wp_v_n <= 2'd0; wp_h_n <= 5'd0; wp_vhi <= 1'b0;
                        end
                    end
            // ── הנעילה ──
                    if (m_phase != 2'd3 || !(wp_from_nav || mode == 3'd4)) wp_pos <= 1'b0;
                    else if (wp_from_nav || m_active || (mode == 3'd4 && wp_hi_ok)) wp_pos <= 1'b1;
                end
            end
            assign m_active_w = m_active;
            assign mode_sel = m_active ? m_mode :
                              (mode == 3'd4) ? (wp_pos ? 3'd2 : 3'd0) : mode;

            // ── הגבלת Roll/Pitch: רק ב-ANGLE / ALTHOLD ידני ──
            assign rp_limit = m_rpl_en && !m_active && ((mode_out == 3'd0) || (mode_out == 3'd1)) &&
                              !m_fm_flags[5] && !m_fm_flags[6] && !m_fm_flags[7];

            // ── ערוץ התשובה: mission_tx רק על CMD:MSTAT, ו-flight_fsm ממתין לו ──
            wire [7:0] mtx_data;
            wire       mtx_send, mtx_active;

            mission_tx u_mtx (
                .clk(clk), .rst_n(rst_n),
                .req(m_cmd_stat),
                .st(m_st), .rsn(m_reason), .chk(m_checks),
                .det(m_det_cm), .stp(m_stop_cm), .wp(m_saved_wp),
                .stage(m_stage),
                .dv(m_det_v), .ta(m_t_ack), .tsp(m_t_stop), .trv(m_trav), .rbk(m_rebound),
                .sck(m_s_ck), .flg(m_flags), .ntr(m_neutral),
                .fsm_cmd(cmd_check | cmd_ping),
                .fsm_tx_data(fsm_tx_data), .fsm_tx_send(fsm_tx_send),
                .uart_busy(esp_uart_busy),
                .tx_data(mtx_data), .tx_send(mtx_send), .active(mtx_active)
            );

            assign esp_tx_data  = mtx_active ? mtx_data : fsm_tx_data;
            assign esp_tx_send  = mtx_active ? mtx_send : fsm_tx_send;
            assign fsm_esp_busy = esp_uart_busy | mtx_active;
        end
        else begin : g_no_mission
            // ── בלי Mission FSM: חוטים ישירים בלבד, בדיוק כמו בגרסה הקודמת ──
            assign thr_sel      = thr_raw;
            assign yaw_sel      = yaw_raw;
            assign pch_sel      = pch_raw;
            assign rll_sel      = rll_raw;
            assign mode_sel     = mode;
            assign m_active_w   = 1'b0;
            assign rp_limit     = 1'b0;
            assign esp_tx_data  = fsm_tx_data;
            assign esp_tx_send  = fsm_tx_send;
            assign fsm_esp_busy = esp_uart_busy;
        end
    endgenerate

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
        .mode_req      (mode_sel),
        .mag_health    (mag_health),
        .mode_out      (mode_out),
        .armed         (armed),
        .send_now      (send_now),
        .msp_busy      (msp_pkt_busy),
        .esp_tx_data   (fsm_tx_data),
        .esp_tx_send   (fsm_tx_send),
        .esp_tx_busy   (fsm_esp_busy),
        .rc_hold       (rc_hold),
        .failsafe      (failsafe),
        .state_dbg     (state_dbg),
        .dwn_fresh_o   (dwn_fresh)
    );

    // ═══ 3.5 - הגבלת Roll/Pitch ל-60% ב-ANGLE / ALTHOLD, במעבר הדרגתי ═══
    wire signed [7:0] pch_lim, rll_lim;
    rp_limiter #(.CLK_FREQ(CLK_FREQ)) u_rpl (
        .clk(clk), .rst_n(rst_n), .limit(rp_limit),
        .pch_in(pch_safe), .rll_in(rll_safe), .pch_out(pch_lim), .rll_out(rll_lim)
    );

    // ═══ 3.6 - SURFACE ידני על CH8 (F1022A, מסירת מצערת F1022B) ═══
    //   INAV 9.1 קורא את מתג SURFACE **רק בכניסה** ל-ALTHOLD / POSHOLD
    //   (navOnEnteringState_NAV_STATE_*_INITIALIZE). הזזת CH8 בתוך POSHOLD לא משנה כלום -
    //   לא בהדלקה ולא בכיבוי (אומת ב-SITL של INAV 9.1.0). לכן כל החלפה של CH8 מלווה
    //   ב"כניסה מחדש": SRF_PULSE_FRAMES חבילות RC עם CH7 = ALTHOLD, ואז חזרה ל-POSHOLD.
    //   ALTHOLD_INITIALIZE מאפס את בקר הגובה ומקבע את היעד לגובה הנוכחי (מעבר חלק).
    //
    //   השער (srf_allow):
    //     · בקשה מה-ESP (SRF:1). ה-ESP כבר בדק: בקשת האפליקציה + DOWN תקין ב-200ms האחרונים.
    //     · CH7 היוצא הוא POSHOLD (אחרי שער המצפן). ב-ANGLE / ALTHOLD / RTH / WP - כבוי.
    //     · אין משימה פעילה, וזרם ה-RC לא עצור.
    //     · לא נדרש חימוש: מנוטרל INAV לא מחליף מצבי ניווט, וכך אפשר לבדוק על הקרקע את
    //       כל השרשרת (CH8 + הפולס) בלי מדחפים ובלי חימוש.
    //   הפולס רק כש-CH7 הוא POSHOLD. אם הטייס עצמו עבר למצב אחר - מעבר המצב שלו הוא
    //   הכניסה מחדש, ואין פולס.
    //   ═══ F1022B: מסירת המצערת ביציאה (תיקון ל-E1022B) ═══
    //   ב-SURFACE המצערת היא גובה, ומקל מלא למטה = 0%. ביציאה INAV נכנס מחדש למצב ולוקח את
    //   rcCommand של אותו רגע כנייטרל (STICK). rcCommand עובר מסנן החלקה, ולכן המצערת חייבת
    //   להיות יציבה **לפני** הכניסה מחדש (ב-SITL: מצערת שהתחלפה באותה חבילה ⇒ טיפוס ב-1.3m/s).
    //   כאן, ב-FPGA, ולא ב-ESP - כי רק כאן ידוע בדיוק מה יוצא בכל חבילה, בלי תלות בשורות
    //   מהאפליקציה ובלי שורות שליטה שה-ESP מייצר (שהיו מאריכות את LINK_LOSS):
    //     1. יציאה כשהמצב הבא שומר גובה (POSHOLD נשאר, או ALTHOLD - גם failover של המצפן)
    //        ויש מרכז תקין מה-ESP (THS ≤ 200): SRF_LEAD_FRAMES חבילות עם CH3 = THS, ‏CH8 נשאר
    //        2000 ו-CH7 נשאר POSHOLD. ב-failover של המצפן זה מעכב את המעבר ל-ALTHOLD ב-100ms בלבד;
    //        ההחלטה של ההגנה עצמה לא השתנתה.
    //     2. אחר כך היציאה הרגילה: CH8 = 1000, ו-POSHOLD ⇒ פולס ALTHOLD / ALTHOLD ⇒ המעבר עצמו.
    //     3. CH3 = THS (החזקה) עד שהמצערת של האפליקציה מגיעה לאותו ערך (±1%). **אין שחרור לפי
    //        טיימר.** ההחזקה נגמרת גם במעבר ל-ANGLE / RTH / WP, בניתוק חימוש, בעצירת זרם ה-RC,
    //        במשימה, או בכניסה חוזרת ל-SURFACE. עד אז INAV פשוט מחזיק גובה.
    //   יציאה ל-ANGLE / RTH / WP, או בלי THS תקין (האפליקציה עוד לא ב-SURFACE): מיידית, כמו F1022A.
    wire       srf_on;
    wire [2:0] mode_msp;
    generate
        if (SURF_ENABLE != 0) begin : g_surf
            reg       srf_q;
            reg [3:0] srf_pulse;
            reg       srf_lead;            // 1 = בתוך ההקדמה (CH8 עדיין 2000)
            reg [2:0] srf_lead_n;
            reg       srf_hold;
            reg [7:0] srf_hold_v;
            wire      srf_mode_ok = (mode_out == 3'd2);
            wire      srf_keep    = (mode_out == 3'd2) || (mode_out == 3'd1);    // היעד שומר גובה
            wire      srf_allow   = srf_req && srf_mode_ok && !m_active_w && !rc_hold;
            wire      ths_ok      = (srf_ths <= 8'd200);
            wire [8:0] thr_diff   = (thr_raw >= srf_hold_v) ? ({1'b0, thr_raw} - {1'b0, srf_hold_v})
                                                            : ({1'b0, srf_hold_v} - {1'b0, thr_raw});
            wire      app_match   = (thr_diff <= 9'd2);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    srf_q      <= 1'b0;
                    srf_pulse  <= 4'd0;
                    srf_lead   <= 1'b0;
                    srf_lead_n <= 3'd0;
                    srf_hold   <= 1'b0;
                    srf_hold_v <= 8'd0;
                end else if (srf_allow) begin
                    srf_lead <= 1'b0;
                    srf_hold <= 1'b0;
                    if (!srf_q) begin                               // כניסה (כמו F1022A)
                        srf_q     <= 1'b1;
                        srf_pulse <= SRF_PULSE_FRAMES[3:0];
                    end else if (send_now && (srf_pulse != 4'd0)) begin
                        srf_pulse <= srf_pulse - 4'd1;
                    end
                end else if (srf_q && !srf_lead) begin
                    // בקשת יציאה
                    if (ths_ok && srf_keep && !rc_hold && !m_active_w) begin
                        srf_lead   <= 1'b1;                         // 1. הקדמה
                        srf_lead_n <= SRF_LEAD_FRAMES[2:0];
                        srf_hold   <= 1'b1;
                        srf_hold_v <= srf_ths;
                        srf_pulse  <= 4'd0;
                    end else begin
                        srf_q     <= 1'b0;                          // מיידית
                        srf_pulse <= srf_mode_ok ? SRF_PULSE_FRAMES[3:0] : 4'd0;
                        srf_hold  <= 1'b0;
                    end
                end else if (srf_lead) begin
                    if (!srf_keep || rc_hold || m_active_w) begin   // הטייס עבר ל-ANGLE וכו' / RC נעצר
                        srf_lead  <= 1'b0;
                        srf_q     <= 1'b0;
                        srf_pulse <= 4'd0;
                        srf_hold  <= 1'b0;
                    end else if (send_now) begin
                        if (srf_lead_n <= 3'd1) begin               // 2. היציאה עצמה
                            srf_lead  <= 1'b0;
                            srf_q     <= 1'b0;
                            srf_pulse <= srf_mode_ok ? SRF_PULSE_FRAMES[3:0] : 4'd0;
                        end
                        srf_lead_n <= srf_lead_n - 3'd1;
                    end
                end else begin
                    // כבוי: הפולס של היציאה + 3. ההחזקה
                    if (!srf_mode_ok)                               srf_pulse <= 4'd0;
                    else if (send_now && (srf_pulse != 4'd0))       srf_pulse <= srf_pulse - 4'd1;
                    if (srf_hold && (!srf_keep || !armed || rc_hold || m_active_w ||
                                     (app_match && srf_pulse == 4'd0)))
                        srf_hold <= 1'b0;
                end
            end
            assign srf_on   = srf_q;
            assign mode_msp = (srf_pulse != 4'd0) ? 3'd1 :
                              srf_lead            ? 3'd2 : mode_out;   // בהקדמה: עדיין POSHOLD
            assign thr_fin  = srf_hold ? srf_hold_v : thr_sel;
        end else begin : g_no_surf
            // CH8 קבוע 1000, ‏CH7 = mode_out ו-CH3 = thr_sel, בדיוק כמו F1021E
            assign srf_on   = 1'b0;
            assign mode_msp = mode_out;
            assign thr_fin  = thr_sel;
        end
    endgenerate

    // ═══ 4 - בניית חבילות MSP ═══
    msp_builder u_msp (
        .clk(clk), .rst_n(rst_n),
        .thr(thr_safe), .yaw(yaw_safe), .pch(pch_lim), .rll(rll_lim),
        .mode(mode_msp), .armed(armed), .surf(srf_on),
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

// ═══════════════════════════════════════════════════════════
//   rp_limiter - הגבלת Roll/Pitch ל-60% (RC 1200..1800)
//
//   out = v · g / 128.  g = 128 (טווח מלא) או 96 (0.75 בדיוק). g זז ב-2 כל 20ms,
//   כלומר 0.32s למעבר מלא - הפעלה, ביטול או מעבר מצב אינם קופצים בפקודה.
//   ±100 ⇒ ±75 ⇒ 1125 / 1875µs. YAW ומצערת לא עוברים כאן.
//   הכפל סדרתי ומשותף לשני הערוצים (≈20 מחזורי שעון, 0.4µs), והיציאות רשומות:
//   הפקודות מתעדכנות ב-20Hz, ולכן ההשהיה זניחה, והשרשרת אל msp_builder קצרה.
// ═══════════════════════════════════════════════════════════
module rp_limiter #(
    parameter CLK_FREQ = 50_000_000
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              limit,
    input  wire signed [7:0] pch_in,
    input  wire signed [7:0] rll_in,
    output reg  signed [7:0] pch_out,
    output reg  signed [7:0] rll_out
);
    localparam integer DIV20 = (CLK_FREQ >= 50) ? (CLK_FREQ / 50) : 1;
    reg [19:0] div_cnt;
    // g = 2·gh: הרגיסטר מחזיק חצי מהמקדם, כך שאין ביט קבוע (ומשם אין אזהרת latch)
    reg [6:0]  gh;
    wire [7:0] g = {gh, 1'b0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 20'd0;
            gh      <= 7'd64;
        end else if (div_cnt >= DIV20 - 1) begin
            div_cnt <= 20'd0;
            if (limit && gh > 7'd48)       gh <= gh - 7'd1;
            else if (!limit && gh < 7'd64) gh <= gh + 7'd1;
        end else begin
            div_cnt <= div_cnt + 20'd1;
        end
    end

    // ── כפל סדרתי: |v| · g, ביט אחרי ביט ──
    reg        ch;          // 0 = Pitch · 1 = Roll
    reg [3:0]  n;           // 0 = טעינה · 1..8 = צעדים · 9 = כתיבה
    reg        sgn;
    reg [14:0] acc, ash;
    reg [7:0]  gsh;
    wire signed [7:0] vin = ch ? rll_in : pch_in;
    wire [6:0] vabs = vin[7] ? (7'd0 - vin[6:0]) : vin[6:0];
    wire [6:0] res  = acc[13:7];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch <= 1'b0; n <= 4'd0; sgn <= 1'b0;
            acc <= 15'd0; ash <= 15'd0; gsh <= 8'd0;
            pch_out <= 8'sd0; rll_out <= 8'sd0;
        end else if (n == 4'd0) begin
            sgn <= vin[7];
            ash <= {8'd0, vabs};
            gsh <= g;
            acc <= 15'd0;
            n   <= 4'd1;
        end else if (n <= 4'd8) begin
            if (gsh[0]) acc <= acc + ash;
            ash <= {ash[13:0], 1'b0};
            gsh <= {1'b0, gsh[7:1]};
            n   <= n + 4'd1;
        end else begin
            if (ch) rll_out <= sgn ? -$signed({1'b0, res}) : $signed({1'b0, res});
            else    pch_out <= sgn ? -$signed({1'b0, res}) : $signed({1'b0, res});
            ch <= ~ch;
            n  <= 4'd0;
        end
    end
endmodule
