// ═══════════════════════════════════════════════════════════
//   mission_fsm.v - ה-Mission FSM של ההדגמה האוטונומית
//
//   ═══ רדום עד START מפורש ═══
//   המודול נבנה תמיד, אבל אחרי הדלקה הוא ב-IDLE ו-m_active=0: המרבב ב-drone_top
//   מעביר את בקשת האפליקציה כמו שהיא. אין חזרה אוטומטית למשימה אחרי איפוס,
//   אובדן תקשורת או ניתוק - START חדש נדרש תמיד, והוא מתקבל רק אם כל 16
//   בדיקות ה-SELF_CHECK עוברות באותו רגע.
//
//   ═══ חלוקת האחריות ═══
//   INAV סוגר את כל הלולאות: ייצוב, גובה, POSHOLD, WP ובלימה. ה-FSM רק מחליט
//   מה שלב המשימה ומבקש מ-INAV דרך CH7 ומקלות קבועים. אין כאן PID.
//   סדר הבוררות: אפליקציה → Mission → שער המצפן → שער הבטיחות (flight_fsm).
//   ABORT, אובדן קשר, ESP מת ושער המצפן גוברים תמיד.
//
//   ═══ רצף ההדגמה ═══
//   מהקרקע (רק אם המראה אוטומטית הותרה בהגדרות):
//     TO_POSHOLD → TO_CLIMB → TO_SETTLE → WP_ENTER → WP_ACTIVE
//     (POSHOLD על הקרקע במצערת נמוכה ⇒ INAV מכין המראה; מצערת מעל הנייטרל ⇒
//      טיפוס בקצב ש-INAV מבקר; עצירה בגובה ה-WP הראשון; רק אז תנועה אופקית)
//   באוויר (המראה ידנית, POSHOLD): HOLD_START → WP_ENTER → WP_ACTIVE
//   מכשול: OBST_STOP → CLIMB → CLEARANCE (+מרווח אנכי) → CLEAR_HOLD →
//          GROUND_REF → CROSS → REAR_EDGE → REAR_CLR (+מרווח אופקי) → REAR_STOP →
//          RET_ALT → ALT_CONFIRM → RESUME_WP → WP_ACTIVE
//   סיום: INAV מדווח FINISH ⇒ WP_DONE (POSHOLD) → DONE_HOLD - מחזיקים POSHOLD
//         עד שהטייס לוקח שליטה. אין LAND ואין RTH אוטומטיים.
//
//   ═══ כללים ═══
//   · טריות לפי מוני הדגימות בלבד. נתון ישן אינו תקין.
//   · בקשת מצב אינה התקדמות: כל מעבר מחכה ש-INAV ידווח שהמצב פעיל.
//   · LiDAR: רק דגימות חדשות נספרות. OUT_OF_RANGE אינו תקלה ואינו הוכחה:
//       FRONT - "פנוי" מוכח רק ב-VALID רחוק מ-CLEAR. אין-החזר (OOR / NO_SIGNAL, LONG ו-SHORT)
//               בטיפוס, אחרי שהמכשול נראה, הוא ראיה חלקית: מאפשר את המרווח האנכי, ובמעבר
//               ה-DOWN חייב לאשר את ראש המכשול (אחרת TOP_UNCONFIRMED).
//       DOWN  - אינו חוסם (בגובה של 2-4m ב-SHORT הוא צפוי). מעבר המכשול
//               מבוסס על עומק המכשול הידוע ועל תזוזת INAV, ו-DOWN רק מאריך.
//   · עצירה → תנועה אנכית → עצירה → תנועה אופקית.
//   · בכל מצב חדש: מחזור "entry" אחד בלי מעבר, שבו המונים מתאפסים.
//   · מצערת ו-Pitch יוצאים דרך מגביל קצב - אין קפיצה בפקודה.
//
//   ═══ מסירת שליטה ═══
//   הנייטרל של המצערת (m_neutral) מדווח לאפליקציה, והיא שולחת אותו בזמן
//   המשימה. מסירה לטייס באוויר מותנית בכך שהמצערת של האפליקציה כבר שווה
//   לנייטרל ("synced"): אחרת ה-FSM מחזיק POSHOLD (HOLD_SYNC) עד שהיא תואמת,
//   כדי שהמסירה לא תפיל את הרחפן.
//
//   ⚠️ זמנים ברזולוציה של 10ms. מרחקים ב-cm, מהירויות ב-cm/s.
// ═══════════════════════════════════════════════════════════
module mission_fsm #(
    parameter CLK_FREQ                  = 50_000_000,

    // ═══ אסטרטגיות ═══
    //   nav_mc_althold_throttle (CAT): 0 STICK / 1 MID_STICK מותרים. HOVER נדחה.
    //   nav_user_control_mode (CUC) חייב להיות CROSS_MODE: 1 CRUISE (ברירת מחדל) / 0 ATTI
    parameter CROSS_MODE                = 1,
    parameter CROSS_PITCH_ATTI          = 6,     // יחידות מקל (±100)
    //   F1021E: 8 ⇒ 80. נמדד בטיסה (21.10): +40µs (8) ב-POSHOLD/CRUISE ⇒ מהירות יעד ~0.8cm/s
    //   (deadband 2, rc_expo 55, pos_hold_deadband 10, nav_manual_speed 50). 80 = 400µs ⇒ ~32cm/s.
    //   INAV עצמו לא יעבור nav_manual_speed (50cm/s במקל מלא), ומגביל המהירות למטה נשאר.
    parameter CROSS_PITCH_CRUISE        = 80,
    parameter ABORT_POLICY              = 0,     // 0 SUSPENDED · 1 HOLD · 2 RTH
    parameter AUTO_SPEED_MAX            = 100,   // cm/s - מהירות WP אפקטיבית מרבית

    // ═══ מכשול ═══
    parameter OBST_DETECT_N             = 3,
    parameter OBST_WINDOW_MS            = 500,
    parameter FRONT_CLEAR_N             = 5,
    //   "אין החזר" בטיפוס (NO_SIGNAL / OUT_OF_RANGE, LONG ו-SHORT) - רק ראיה חלקית:
    //   מאפשר את המרווח האנכי, ובמעבר ה-DOWN חייב לראות את ראש המכשול עד
    //   מרחק העצירה + (מקצה הפרופלור עד ה-DOWN) + המרווח הזה. אחרת - עצירה.
    parameter TOP_CONFIRM_MARGIN_CM     = 30,
    parameter AVOID_MIN_FRONT_CM        = 30,
    parameter FWA_LOSS_MS               = 1000,

    // ═══ תנועה ═══
    parameter V_STOP_CMS                = 15,
    parameter VZ_SETTLE_CMS             = 15,
    parameter T_STOPPED_MS              = 500,
    parameter T_SETTLE_MS               = 1000,

    // ═══ טיפוס / המראה (חצאי אחוז מצערת) ═══
    parameter CLIMB_THR_OFFSET          = 30,
    parameter DESC_THR_OFFSET           = 40,
    parameter THR_SLEW_PER_10MS         = 2,
    parameter MIN_CLIMB_CM              = 10,
    parameter MAX_CLIMB_DELTA_CM        = 350,
    parameter CEIL_MARGIN_CM            = 30,    // עצירת הטיפוס לפני התקרה (נמדד: חריגה של 19cm ב-27cm/s)
    parameter TO_LIFT_CM                = 30,    // Z מעל זה = באוויר (מסירה דרך HOLD_SYNC)

    // ═══ DOWN ומעבר המכשול ═══
    parameter DWN_GMAX_SHORT_CM         = 115,
    parameter DWN_GMAX_LONG_CM          = 350,
    parameter GND_N                     = 4,
    parameter GND_TOL_CM                = 8,
    parameter OBST_MIN_H_CM             = 20,
    parameter TOP_N                     = 4,
    parameter REAR_N                    = 4,
    parameter REAR_TOL_CM               = 10,
    parameter MIN_GAP_CM                = 35,
    parameter MIN_GAP_DESC_CM           = 30,
    parameter CROSS_SPEED_MAX_CMS       = 40,
    parameter CROSS_SPEED_RESUME_CMS    = 25,
    parameter DRONE_LEN_CM              = 60,    // מקצה פרופלור קדמי עד קצה אחורי
    parameter DRONE_HALF_CM             = 30,    // מחיישן DOWN (מרכז) עד הקצה האחורי
    parameter POS_MARGIN_CM             = 30,    // מרווח לאי-ודאות מיקום יחסית
    parameter OBST_EXTRA_MAX_CM         = 150,   // DOWN רואה ראש מעבר לעומק המוצהר ⇒ ארוך מדי
    // ── כיוון המעבר: התקדמות נמדדת רק לאורך כיוון הטיסה ברגע הזיהוי ──
    parameter DIR_MIN_CMS               = 20,    // מהירות מינימלית בזיהוי כדי שהכיוון ייחשב ידוע
    parameter CROSS_LAT_MAX_CM          = 40,    // סטייה צדית מרבית מקו המעבר
    parameter CROSS_BACK_MAX_CM         = 30,    // נסיגה מרבית לאחור מנקודת העצירה
    parameter ALT_TOL_CM                = 20,

    // ═══ GPS ═══
    //   START והמשך המשימה: 12 לוויינים לפחות ו-EPH עד 1.5m.
    //   מעבר "עיוור" (DOWN לא רואה קרקע) - רק עם EPH עד 1m, והמרחק הנדרש גדל ב-EPH/2.
    parameter GPS_MIN_SATS              = 12,
    parameter GPS_MAX_EPH_CM            = 150,
    parameter CROSS_BLIND_EPH_CM        = 100,

    // ═══ זמנים (ms) ═══
    parameter T_ACK_MS                  = 1500,
    parameter T_STOP_MS                 = 5000,
    parameter T_CLIMB_MS                = 15000,
    parameter T_GREF_MS                 = 2000,
    parameter T_CROSS_MS                = 30000,
    parameter T_RETALT_MS               = 20000,
    parameter T_WP_STALL_MS             = 150000,
    parameter T_SYNC_MS                 = 5000,
    parameter MISSION_MAX_S             = 600,

    // ═══ טריות (ms, עד 5100) ═══
    parameter POSE_STALE_MS             = 400,
    parameter NAV_STALE_MS              = 700,
    parameter FLG_STALE_MS              = 700,
    parameter GPS_STALE_MS              = 1200,
    parameter SLOW_STALE_MS             = 3000,
    parameter SENS_STALE_MS             = 300,

    // ═══ השתלטות טייס ═══
    parameter TAKEOVER_STICK            = 10,
    parameter TAKEOVER_THR              = 8
)(
    input  wire               clk,
    input  wire               rst_n,

    // ── בקשת האפליקציה (cmd_parser) ──
    input  wire [7:0]         thr_app,
    input  wire signed [7:0]  yaw_app,
    input  wire signed [7:0]  pch_app,
    input  wire signed [7:0]  rll_app,
    input  wire [2:0]         mode_app,

    // ── חיישנים (cmd_parser) ──
    input  wire [8:0]         lidar_fwd,
    input  wire [8:0]         lidar_dwn,
    input  wire [7:0]         lidar_fwd_seq,
    input  wire [7:0]         lidar_dwn_seq,
    input  wire [1:0]         mag_health,
    input  wire               sens_valid,

    // ── שכבת הבטיחות (flight_fsm) ──
    input  wire               armed,
    input  wire               rc_hold,

    // ── mission_rx ──
    input  wire signed [15:0] pos_x,
    input  wire signed [15:0] pos_y,
    input  wire signed [15:0] pos_z,
    input  wire signed [11:0] vel_x,
    input  wire signed [11:0] vel_y,
    input  wire signed [11:0] vel_z,
    input  wire [3:0]         pose_seq,
    input  wire [3:0]         nav_mode,
    input  wire [4:0]         nav_state,
    input  wire [5:0]         nav_wp,
    input  wire [3:0]         nav_err,
    input  wire [3:0]         nav_seq,
    input  wire [10:0]        fm_flags,
    input  wire               inav_armed,
    input  wire [3:0]         flg_seq,
    input  wire [1:0]         gps_fix,
    input  wire [5:0]         gps_sats,
    input  wire [9:0]         gps_eph,
    input  wire [3:0]         gps_seq,
    input  wire [3:0]         mis_code,
    input  wire [1:0]         cfg_atm,
    input  wire [1:0]         cfg_ucm,
    input  wire [1:0]         cfg_rst,
    input  wire [10:0]        cfg_aspd,
    input  wire [1:0]         batt_st,
    input  wire [3:0]         slow_seq,
    input  wire [6:0]         thr_mid,
    input  wire [10:0]        mis_alt,
    input  wire [1:0]         phase,
    input  wire               fwd_active,
    input  wire [2:0]         fwd_cls,
    input  wire [2:0]         dwn_cls,
    input  wire               fwd_long,
    input  wire               dwn_long,
    input  wire               cmd_mstart,
    input  wire               cmd_mstop,
    input  wire               cmd_mcross,     // לא בשימוש: CROSS TEST הוחלף בשלב בדיקה 4
    // ── סט ההגדרות מ-mission_rx ──
    input  wire [9:0]         set_stop,
    input  wire [9:0]         set_clear,
    input  wire [9:0]         set_vclr,
    input  wire [9:0]         set_fclr,
    input  wire [9:0]         set_ceil,
    input  wire [9:0]         set_depth,
    input  wire [2:0]         set_stage,
    input  wire [1:0]         set_flags,
    input  wire               set_valid,     // הסט התקבל שלם (mission_rx)
    output wire               set_allow,     // מותר להחיל סט חדש: המשימה לא פעילה וה-FPGA מנוטרל

    // ── אל המרבב ב-drone_top ──
    output wire               m_active,
    output reg  [7:0]         m_thr,
    output wire signed [7:0]  m_yaw,
    output reg  signed [7:0]  m_pch,
    output wire signed [7:0]  m_rll,
    output reg  [2:0]         m_mode,

    // ── מצב, לתשובת MSTAT ──
    output wire [4:0]         st_code,
    output reg  [7:0]         reason,
    output wire [15:0]        checks,
    output reg  [9:0]         det_cm,     // FWD ברגע הזיהוי (3FF = אין)
    output reg  [9:0]         stop_cm,    // FWD אחרי העצירה (3FF = אין)
    output reg  [5:0]         saved_wp,
    output wire [3:0]         stage_code,
    output reg  [9:0]         det_v,      // מהירות אופקית ברגע הזיהוי, cm/s
    output reg  [7:0]         t_ack,      // מהזיהוי עד ש-INAV אישר POSHOLD, 10ms (FF = לא)
    output reg  [9:0]         t_stop,     // מהזיהוי עד עצירה יציבה, 10ms
    output reg  [9:0]         trav,       // תזוזה מנקודת הזיהוי עד העצירה, cm
    output reg  [7:0]         rebound,    // חזרה לאחור אחרי השיא, cm
    output wire [3:0]         flags_out,  // bit0 סט תקין · bit1 DOWN רואה קרקע · bit2 המראה מהקרקע · bit3 synced
    output reg  [7:0]         m_neutral   // נייטרל המצערת, חצאי אחוז
);

    // ═══════════════════════════════════════════════
    //   קבועים
    // ═══════════════════════════════════════════════
    localparam [2:0] M_POSHOLD = 3'd2, M_RTH = 3'd3, M_WP = 3'd4;

    localparam [2:0] C_FAULT = 3'd0, C_VALID = 3'd1, C_OOR = 3'd2, C_WEAK = 3'd3,
                     C_TOO_CLOSE = 3'd5, C_NOSIG = 3'd6;

    localparam FM_ARM = 0, FM_ALTHOLD = 4, FM_POSHOLD = 5, FM_RTH = 6, FM_WP = 7, FM_FAILSAFE = 10;

    localparam [3:0] NAVM_NAV = 4'd3, NAVM_EMERG = 4'd15;
    localparam [3:0] NAVE_FINISH = 4'd4;

    // מצבי ה-FSM
    localparam [4:0] S_IDLE        = 5'd0,
                     S_HOLD_START  = 5'd1,
                     S_WP_ENTER    = 5'd2,
                     S_WP_ACTIVE   = 5'd3,
                     S_OBST_STOP   = 5'd4,
                     S_CLIMB       = 5'd5,
                     S_CLEARANCE   = 5'd6,
                     S_CLEAR_HOLD  = 5'd7,
                     S_GROUND_REF  = 5'd8,
                     S_CROSS       = 5'd9,
                     S_REAR_EDGE   = 5'd10,
                     S_REAR_CLR    = 5'd11,
                     S_REAR_STOP   = 5'd12,
                     S_RET_ALT     = 5'd13,
                     S_ALT_CONFIRM = 5'd14,
                     S_RESUME_WP   = 5'd15,
                     S_WP_DONE     = 5'd16,
                     S_COMPLETE    = 5'd17,
                     S_SUSPENDED   = 5'd18,
                     S_ABORT_HOLD  = 5'd19,
                     S_ABORT_RTH   = 5'd20,
                     // 21-23: המראה אוטומטית מהקרקע - הוסרה ב-F1021E (הקודים לא בשימוש)
                     S_DONE_HOLD   = 5'd24,
                     S_HOLD_SYNC   = 5'd25;

    // סיבות
    localparam [7:0] R_NONE            = 8'h00,
                     R_START_REJECTED  = 8'h01,
                     R_PILOT_STOP      = 8'h02,
                     R_TAKEOVER        = 8'h03,
                     R_SAFETY          = 8'h04,
                     R_MODE_ACK        = 8'h05,
                     R_STALE_POSE      = 8'h06,
                     R_STALE_NAV       = 8'h07,
                     R_STALE_FLAGS     = 8'h08,
                     R_STALE_GPS       = 8'h09,
                     R_STALE_SENS      = 8'h0A,
                     R_STALE_FRONT     = 8'h0B,
                     R_STALE_DOWN      = 8'h0C,
                     R_MAG             = 8'h0D,
                     R_GPS             = 8'h0E,
                     R_FAILSAFE        = 8'h0F,
                     R_INAV_DISARMED   = 8'h10,
                     R_NAV_ERROR       = 8'h11,
                     R_FRONT_INACTIVE  = 8'h12,
                     R_STOP_TIMEOUT    = 8'h13,
                     R_STAGE_END       = 8'h14,
                     R_STOP_TOO_CLOSE  = 8'h15,
                     R_CEILING         = 8'h16,
                     R_CLIMB_TIMEOUT   = 8'h17,
                     R_MIN_GAP         = 8'h1A,
                     R_FRONT_OBST      = 8'h1C,
                     R_OBST_TOO_LONG   = 8'h1D,
                     R_ALT_TIMEOUT     = 8'h1F,
                     R_WP_MISMATCH     = 8'h20,
                     R_WP_STALL        = 8'h21,
                     R_BATTERY         = 8'h22,
                     R_MISSION_TIME    = 8'h23,
                     R_CROSS_TIMEOUT   = 8'h25,
                     R_OBST_LOST       = 8'h26,
                     R_CROSS_DISABLED  = 8'h27,
                     R_INAV_EMERG      = 8'h28,
                     R_STALE_SLOW      = 8'h29,
                     // 2A / 2B (NO_LIFTOFF / SETTLE_TIMEOUT) - הוסרו עם ההמראה האוטומטית
                     R_SETTINGS        = 8'h2C,
                     R_SYNC_TIMEOUT    = 8'h2D,   // אזהרה בלבד: ממשיכים להחזיק POSHOLD
                     R_CROSS_DRIFT     = 8'h2E,   // סטייה צדית / נסיגה במעבר
                     R_NO_DIRECTION    = 8'h2F,   // כיוון הטיסה בזיהוי לא ידוע
                     R_GPS_BLIND       = 8'h30,   // GPS לא מספיק טוב למעבר בלי DOWN
                     R_TOP_UNCONFIRMED = 8'h31;   // "פנוי" לפי אין-החזר, וה-DOWN לא ראה את ראש המכשול

    localparam [12:0] P_EXTRA = OBST_EXTRA_MAX_CM;
    localparam [11:0] P_HALF  = DRONE_HALF_CM;

    // ── זמנים ביחידות של 10ms ──
    localparam integer T10_ACK     = T_ACK_MS / 10,
                       T10_STOPPED = T_STOPPED_MS / 10,
                       T10_SETTLE  = T_SETTLE_MS / 10,
                       T10_STOP    = T_STOP_MS / 10,
                       T10_CLIMB   = T_CLIMB_MS / 10,
                       T10_GREF    = T_GREF_MS / 10,
                       T10_CROSS   = T_CROSS_MS / 10,
                       T10_RETALT  = T_RETALT_MS / 10,
                       T10_STALL   = T_WP_STALL_MS / 10,
                       T10_SYNC    = T_SYNC_MS / 10,
                       T10_FWALOSS = FWA_LOSS_MS / 10,
                       T10_OBSTWIN = OBST_WINDOW_MS / 10,
                       T10_POSE    = POSE_STALE_MS / 10,
                       T10_NAV     = NAV_STALE_MS / 10,
                       T10_FLG     = FLG_STALE_MS / 10,
                       T10_GPS     = GPS_STALE_MS / 10,
                       T10_SLOW    = SLOW_STALE_MS / 10,
                       T10_SENS    = SENS_STALE_MS / 10;

    // ═══ בדיקת טווח לפרמטרים, בזמן הקומפילציה ═══
    localparam OK_FRESH  = (T10_POSE >= 1) && (T10_POSE <= 511) && (T10_NAV  >= 1) && (T10_NAV  <= 511) &&
                           (T10_FLG  >= 1) && (T10_FLG  <= 511) && (T10_GPS  >= 1) && (T10_GPS  <= 511) &&
                           (T10_SLOW >= 1) && (T10_SLOW <= 511) && (T10_SENS >= 1) && (T10_SENS <= 511);
    localparam OK_TIMERS = (T10_ACK     >= 1) && (T10_ACK     <= 511) && (T10_STOPPED >= 1) && (T10_STOPPED <= 511) &&
                           (T10_SETTLE  >= 1) && (T10_SETTLE  <= 511) && (T10_FWALOSS >= 1) && (T10_FWALOSS <= 511) &&
                           (T10_OBSTWIN >= 1) && (T10_OBSTWIN <= 127) &&
                           (T10_STOP  >= 1) && (T10_STOP  <= 16383) && (T10_CLIMB  >= 1) && (T10_CLIMB  <= 16383) &&
                           (T10_GREF  >= 1) && (T10_GREF  <= 16383) && (T10_CROSS  >= 1) && (T10_CROSS  <= 16383) &&
                           (T10_RETALT >= 1) && (T10_RETALT <= 16383) &&
                           (T10_SYNC >= 1) && (T10_SYNC <= 16383) &&
                           (T10_STALL >= 1) && (T10_STALL <= 16383) &&
                           (MISSION_MAX_S >= 1) && (MISSION_MAX_S <= 1023) && (CLK_FREQ / 100 <= 1048575);
    localparam OK_COUNTS = (OBST_DETECT_N >= 1) && (OBST_DETECT_N <= 15) && (FRONT_CLEAR_N >= 1) && (FRONT_CLEAR_N <= 15) &&
                           (GND_N >= 1) && (GND_N <= 15) && (TOP_N >= 1) && (TOP_N <= 15) && (REAR_N >= 1) && (REAR_N <= 15);
    localparam OK_MISC   = (ABORT_POLICY <= 2) && (CROSS_MODE <= 1) &&
                           (CROSS_PITCH_ATTI <= 100) && (CROSS_PITCH_CRUISE <= 100) &&
                           (TAKEOVER_STICK <= 100) && (TAKEOVER_THR <= 200) &&
                           (CLIMB_THR_OFFSET <= 70) && (DESC_THR_OFFSET <= 70) &&
                           (THR_SLEW_PER_10MS >= 1) && (THR_SLEW_PER_10MS <= 20) &&
                           (MIN_GAP_DESC_CM <= MIN_GAP_CM) && (MIN_GAP_CM < DWN_GMAX_SHORT_CM) &&
                           (DWN_GMAX_LONG_CM <= 400) && (DWN_GMAX_SHORT_CM <= DWN_GMAX_LONG_CM) &&
                           (CROSS_SPEED_RESUME_CMS < CROSS_SPEED_MAX_CMS) && (MAX_CLIMB_DELTA_CM <= 1000) &&
                           (DRONE_LEN_CM + POS_MARGIN_CM + OBST_EXTRA_MAX_CM <= 1000) && (DRONE_HALF_CM <= DRONE_LEN_CM) &&
                           (AVOID_MIN_FRONT_CM <= 100) && (AUTO_SPEED_MAX <= 127) &&
                           (DIR_MIN_CMS >= 5) && (DIR_MIN_CMS <= 100) && (CROSS_LAT_MAX_CM <= 200) &&
                           (CROSS_BACK_MAX_CM <= 200) && (CROSS_BLIND_EPH_CM <= GPS_MAX_EPH_CM) && (GPS_MAX_EPH_CM <= 1023) &&
                           (CEIL_MARGIN_CM <= 100) && (TOP_CONFIRM_MARGIN_CM <= 200);
    generate
        if (!OK_FRESH)  begin : g_err_fresh  MISSION_FSM_ERROR_STALE_MS_OUT_OF_RANGE  u_err (); end
        if (!OK_TIMERS) begin : g_err_timers MISSION_FSM_ERROR_TIMER_OUT_OF_RANGE     u_err (); end
        if (!OK_COUNTS) begin : g_err_counts MISSION_FSM_ERROR_COUNT_N_OUT_OF_RANGE   u_err (); end
        if (!OK_MISC)   begin : g_err_misc   MISSION_FSM_ERROR_PARAMETER_OUT_OF_RANGE u_err (); end
    endgenerate

    // ═══════════════════════════════════════════════
    //   בסיס זמן: פולס כל 10ms, ופולס כל שנייה
    // ═══════════════════════════════════════════════
    localparam integer DIV10 = (CLK_FREQ >= 100) ? (CLK_FREQ / 100) : 1;

    reg [19:0] div_cnt;
    reg        tick;
    reg [6:0]  tick_in_s;
    reg        s_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt   <= 20'd0;
            tick      <= 1'b0;
            tick_in_s <= 7'd0;
            s_tick    <= 1'b0;
        end else begin
            tick   <= 1'b0;
            s_tick <= 1'b0;
            if (div_cnt >= DIV10 - 1) begin
                div_cnt <= 20'd0;
                tick    <= 1'b1;
                if (tick_in_s >= 7'd99) begin
                    tick_in_s <= 7'd0;
                    s_tick    <= 1'b1;
                end else begin
                    tick_in_s <= tick_in_s + 7'd1;
                end
            end else begin
                div_cnt <= div_cnt + 20'd1;
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   דגימות חדשות - לפי המונים בלבד
    // ═══════════════════════════════════════════════
    reg [3:0] pose_seq_l, nav_seq_l, flg_seq_l, gps_seq_l, slow_seq_l;
    reg [7:0] fseq_l, dseq_l;

    wire pose_evt = (pose_seq != pose_seq_l);
    wire nav_evt  = (nav_seq  != nav_seq_l);
    wire flg_evt  = (flg_seq  != flg_seq_l);
    wire gps_evt  = (gps_seq  != gps_seq_l);
    wire slow_evt = (slow_seq != slow_seq_l);
    wire fwd_evt  = sens_valid && (lidar_fwd_seq != fseq_l);
    wire dwn_evt  = sens_valid && (lidar_dwn_seq != dseq_l);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pose_seq_l <= 4'd0; nav_seq_l <= 4'd0; flg_seq_l <= 4'd0;
            gps_seq_l  <= 4'd0; slow_seq_l <= 4'd0; fseq_l <= 8'd0; dseq_l <= 8'd0;
        end else begin
            pose_seq_l <= pose_seq;
            nav_seq_l  <= nav_seq;
            flg_seq_l  <= flg_seq;
            gps_seq_l  <= gps_seq;
            slow_seq_l <= slow_seq;
            if (sens_valid) begin
                fseq_l <= lidar_fwd_seq;
                dseq_l <= lidar_dwn_seq;
            end
        end
    end

    // ═══ גיל כל מקור, ביחידות 10ms (רווי ב-511 = "מעולם לא התקבל") ═══
    wire [8:0] pose_age, nav_age, flg_age, gps_age, slow_age, sens_age, fwd_age, dwn_age;

    mission_age u_age_pose (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(pose_evt),   .age(pose_age));
    mission_age u_age_nav  (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(nav_evt),    .age(nav_age));
    mission_age u_age_flg  (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(flg_evt),    .age(flg_age));
    mission_age u_age_gps  (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(gps_evt),    .age(gps_age));
    mission_age u_age_slow (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(slow_evt),   .age(slow_age));
    mission_age u_age_sens (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(sens_valid), .age(sens_age));
    mission_age u_age_fwd  (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(fwd_evt),    .age(fwd_age));
    mission_age u_age_dwn  (.clk(clk), .rst_n(rst_n), .tick(tick), .clr(dwn_evt),    .age(dwn_age));

    wire pose_fresh = (pose_age < T10_POSE);
    wire nav_fresh  = (nav_age  < T10_NAV);
    wire flg_fresh  = (flg_age  < T10_FLG);
    wire gps_fresh  = (gps_age  < T10_GPS);
    wire slow_fresh = (slow_age < T10_SLOW);
    wire sens_fresh = (sens_age < T10_SENS);
    wire front_ok   = (fwd_age  < T10_SENS) && (fwd_cls != C_FAULT);
    wire down_ok    = (dwn_age  < T10_SENS) && (dwn_cls != C_FAULT);

    // ═══════════════════════════════════════════════
    //   הגדרות העקיפה - סט אחד שהוחל, ותקף רק אחרי שהתקבל מה-ESP
    // ═══════════════════════════════════════════════
    //   עותק אחד בלבד, ב-mission_rx. נכתב רק כש-set_allow, ותקף רק בטווח.
    wire [9:0] r_stop  = set_stop,  r_clear = set_clear, r_vclr = set_vclr,
               r_fclr  = set_fclr,  r_ceil  = set_ceil,  r_depth = set_depth;
    wire [2:0] r_stage = set_stage;

    // טווחי הערכים המותרים - זהים לאפליקציה ול-ESP
    //   (האפליקציה וה-ESP בודקים את הטווח המלא; כאן הגבולות שהבטיחות תלויה בהם)
    wire set_in_range = (set_stop  >= 10'd40)  && (set_stop  < set_clear) && (set_clear <= 10'd380) &&
                        (set_vclr  >= 10'd30)  && (set_vclr  <= 10'd150) &&
                        (set_fclr  >= 10'd30)  && (set_fclr  <= 10'd200) &&
                        (set_ceil  >= 10'd150) && (set_ceil  <= 10'd400) && (set_depth <= 10'd200) &&
                        (set_stage >= 3'd1)    && (set_stage <= 3'd5) && (set_flags == 2'd0);   // F1021E: אין המראה אוטומטית
    wire set_ok = set_valid && set_in_range;

    // ═══════════════════════════════════════════════
    //   עובדות נגזרות
    // ═══════════════════════════════════════════════
    wire gps_ok = gps_fresh && (gps_fix == 2'd2) && (gps_sats >= GPS_MIN_SATS) && (gps_eph <= GPS_MAX_EPH_CM);
    wire mag_ok = sens_fresh && (mag_health == 2'd1);

    wire [11:0] avx = vel_x[11] ? (12'd0 - vel_x) : vel_x;
    wire [11:0] avy = vel_y[11] ? (12'd0 - vel_y) : vel_y;
    wire [11:0] avz = vel_z[11] ? (12'd0 - vel_z) : vel_z;
    wire [12:0] vxy = {1'b0, avx} + {1'b0, avy};

    // ACK - המצב המבוקש פעיל בפועל ב-INAV
    wire ack_pos = fm_flags[FM_POSHOLD] && !fm_flags[FM_WP] && !fm_flags[FM_RTH];
    wire ack_wp  = fm_flags[FM_WP] && nav_fresh && (nav_mode == NAVM_NAV);
    wire ack_rth = fm_flags[FM_RTH];

    // ── LiDAR קדמי ──
    wire f_valid = (fwd_cls == C_VALID);
    wire f_hit   = (f_valid && ({1'b0, lidar_fwd} <= r_stop)) || (fwd_cls == C_TOO_CLOSE);
    wire f_far   = f_valid && ({1'b0, lidar_fwd} > r_clear);
    // "פנוי" מוכח = רק מדידת מרחק תקינה שרחוקה מ-CLEAR. אין-החזר (OOR / NO_SIGNAL, גם ב-LONG)
    // אינו הוכחה: הוא נספר ב-f_nr בלבד, ומחייב אישור ראש ב-DOWN במעבר (clr_inf).
    wire f_clr   = f_far;
    wire f_blk   = (f_valid && !f_far) || (fwd_cls == C_TOO_CLOSE) || (fwd_cls == C_WEAK);
    // אין החזר בכלל: NO_SIGNAL (status 2) או OUT_OF_RANGE - באותו אופן ב-LONG וב-SHORT
    wire f_nr    = (fwd_cls == C_NOSIG) || (fwd_cls == C_OOR);

    // ── LiDAR תחתון ──
    wire d_valid  = (dwn_cls == C_VALID);
    wire d_close  = (dwn_cls == C_TOO_CLOSE);
    wire d_oor    = (dwn_cls == C_OOR);
    wire [9:0] dwn_gmax = dwn_long ? DWN_GMAX_LONG_CM : DWN_GMAX_SHORT_CM;

    // ── גובה ותזוזה ברוחב מצומצם ──
    //   Z רווי ל-±2047cm (ההדגמה עד 4m). X/Y נלקחים מגולגלים ל-14 ביט: הפרש
    //   מגולגל נכון כל עוד ההפרש האמיתי קטן מ-81m - וה-FSM מודד רק כמה מטרים.
    wire signed [11:0] pz   = (pos_z > 16'sd2047) ? 12'sd2047 : (pos_z < -16'sd2047) ? -12'sd2047 : pos_z[11:0];
    wire        [13:0] px   = pos_x[13:0];
    wire        [13:0] py   = pos_y[13:0];
    wire signed [11:0] surf = pz - $signed({3'd0, lidar_dwn});

    // ═══════════════════════════════════════════════
    //   רגיסטרים של המשימה
    // ═══════════════════════════════════════════════
    reg [4:0]  st;
    reg        entry;
    reg        synced;             // מצערת האפליקציה כבר שווה לנייטרל
    reg        fin_ok;             // המשימה הושלמה (DONE_HOLD) - המסירה מסתיימת ב-COMPLETE
    reg        gnd_vis;            // DOWN ראה קרקע ב-GROUND_REF
    reg        ack_seen;
    reg [7:0]  thr_start;
    reg [5:0]  exp_wp;
    reg        resume_valid;
    reg [5:0]  last_wp;

    reg signed [11:0] h_start;
    reg signed [11:0] tgt_z;
    reg signed [11:0] lim_z;
    reg        [13:0] x0, y0;      // נקודת ייחוס לתזוזה
    reg signed [11:0] gref;
    reg signed [11:0] cand;
    reg        [11:0] cross_tot;   // התזוזה הנדרשת מנקודת העצירה עד סוף המעבר, cm
    reg        [11:0] conf_tot;    // עד כאן ה-DOWN חייב לראות ראש, כשה"פנוי" היה לפי אין-החזר
    reg               clr_inf;     // ה"פנוי" בטיפוס נקבע לפי אין-החזר (לא לפי VALID רחוק)
    reg               blk_seen;    // מאז העצירה ה-FRONT ראה את המכשול (VALID קרוב / TOO_CLOSE / WEAK)
    reg               top_late;    // clr_inf ועברנו את conf_tot בלי ראש

    reg [3:0]  obst_cnt, clr_cnt, nr_cnt, d_cnt, oor_cnt;
    reg [1:0]  alt_cnt;
    reg        spd_ok;

    reg [13:0] st_t;
    reg [13:0] wp_t;
    reg [8:0]  still_t, vz_t, band_t, ack_bad_t, fwa_t;
    reg [6:0]  obst_win;
    reg [9:0]  mission_s;
    reg [11:0] dmax;

    // ── גבהים ──
    wire signed [11:0] P_MIN_CLIMB = MIN_CLIMB_CM;
    wire signed [11:0] P_ALT_TOL   = ALT_TOL_CM;
    wire signed [11:0] ceil_s      = $signed({2'd0, r_ceil});
    wire signed [11:0] ceil_m      = ceil_s - CEIL_MARGIN_CM;     // גבול הטיפוס בפועל
    wire signed [11:0] vclr_s      = $signed({2'd0, r_vclr});
    wire signed [12:0] z_climb_max = pz + MAX_CLIMB_DELTA_CM;
    wire signed [12:0] z_clear_c   = pz + vclr_s;
    wire signed [11:0] lim_next_c  = (z_climb_max < ceil_m) ? z_climb_max[11:0] : ceil_m;

    // ── גובה המשטח מול מועמד / קרקע ──
    wire signed [11:0] ds_cand = surf - cand;
    wire signed [11:0] ds_gref = surf - gref;
    wire near_gnd_c  = (ds_cand <= GND_TOL_CM) && (ds_cand >= -GND_TOL_CM);
    wire is_top_c    = (ds_gref >= OBST_MIN_H_CM);
    wire is_ground_c = (ds_gref <= REAR_TOL_CM) && (ds_gref >= -REAR_TOL_CM);

    // ═══ תזוזה מנקודת העצירה - לאורך כיוון המעבר בלבד ═══
    //   הכיוון: וקטור המהירות ברגע הזיהוי (הרחפן טס אז ב-WP אל המכשול), מנורמל
    //   ל-u = v/|v| בסקאלה 128 (mission_dir). מהתזוזה d = p - p0:
    //     along = d·u   - התקדמות קדימה בלבד. תזוזה הצידה או אחורה אינה נספרת.
    //     lat   = |d×u| - סטייה צדית מקו המעבר.
    //   |u| ≤ 1 בכוונה (השורש מעוגל כלפי מעלה), ולכן along לעולם אינו גדול מהאמיתי.
    //   הכפלים במכפלים הייעודיים של ה-FPGA. חשבון 14 ביט מגולגל: נכון עד ~81m.
    wire signed [13:0] dx  = px - x0;
    wire signed [13:0] dy  = py - y0;
    reg  signed [13:0] dxr, dyr;
    wire signed [8:0]  ux, uy;
    wire               dir_ok;
    reg                dir_go;
    reg  signed [22:0] along_raw, lat_raw;
    wire signed [15:0] along = along_raw[22:7];
    wire        [15:0] lat_abs = lat_raw[22] ? (16'd0 - lat_raw[22:7]) : lat_raw[22:7];
    reg         [11:0] disp;           // along רשום, 0 כשהוא שלילי, רווי ב-4095
    reg                drift;          // סטייה צדית / נסיגה מעבר למותר
    reg         [2:0]  pcnt;           // מחזורים מאז entry - הצנרת מתייצבת אחרי 6

    mission_dir #(.DIR_MIN(DIR_MIN_CMS)) u_dir (
        .clk(clk), .rst_n(rst_n), .start(dir_go), .vx(vel_x), .vy(vel_y),
        .ux(ux), .uy(uy), .ok(dir_ok)
    );

    // ── השתלטות טייס ──
    wire [7:0] ayaw = yaw_app[7] ? (8'd0 - yaw_app) : yaw_app;
    wire [7:0] apch = pch_app[7] ? (8'd0 - pch_app) : pch_app;
    wire [7:0] arll = rll_app[7] ? (8'd0 - rll_app) : rll_app;
    wire [7:0] dthr = (thr_app > m_neutral) ? (thr_app - m_neutral) : (m_neutral - thr_app);
    wire       sticks_centered = (ayaw <= TAKEOVER_STICK) && (apch <= TAKEOVER_STICK) && (arll <= TAKEOVER_STICK);
    wire       mode_ok_app     = (mode_app == M_POSHOLD) || (mode_app == M_WP);
    wire       thr_in_sync     = (dthr <= TAKEOVER_THR);
    wire       takeover_c = !sticks_centered || !mode_ok_app || (synced && !thr_in_sync);
    reg  [4:0] sync_t;                 // 10ms ticks שבהם המצערת תואמת ברצף
    wire       thr_held   = thr_in_sync && (sync_t >= 5'd20);   // תואמת עכשיו, 200ms ב-HOLD_SYNC

    // ── בדיקות START: באוויר בלבד (F1021E: ההמראה האוטומטית מהקרקע הוסרה) ──
    wire       ground_mode = (phase != 2'd3);
    wire [7:0] mid_half    = {thr_mid, 1'b0};
    wire       cfg_ok_c    = slow_fresh && (cfg_atm <= 2'd1) && (cfg_ucm == CROSS_MODE) && (cfg_rst == 2'd1) &&
                             (cfg_aspd <= AUTO_SPEED_MAX) && (thr_mid >= 7'd30) && (thr_mid <= 7'd80) && set_ok;
    wire       alt_ok_c    = (mis_alt >= 11'd100) &&
                             ({1'b0, mis_alt} + {2'd0, r_vclr} + CEIL_MARGIN_CM <= {2'd0, r_ceil});

    // ═══════════════════════════════════════════════
    //   SELF_CHECK - 16 תנאים, מדווחים חיים בתשובת MSTAT
    //   START רק באוויר: AIRBORNE, POSHOLD פעיל, מקלות במרכז. על הקרקע bit2 תמיד false.
    // ═══════════════════════════════════════════════
    wire [15:0] chk_c;
    assign chk_c[0]  = armed && !rc_hold;
    assign chk_c[1]  = flg_fresh && inav_armed && fm_flags[FM_ARM];
    assign chk_c[2]  = sens_fresh && !ground_mode;
    assign chk_c[3]  = flg_fresh && ack_pos && mode_ok_app;
    assign chk_c[4]  = mag_ok;
    assign chk_c[5]  = gps_ok;
    assign chk_c[6]  = flg_fresh && !fm_flags[FM_FAILSAFE];
    assign chk_c[7]  = pose_fresh;
    assign chk_c[8]  = nav_fresh;
    assign chk_c[9]  = front_ok && down_ok && sens_fresh && fwd_active;
    assign chk_c[10] = slow_fresh && (mis_code == 4'd1) && alt_ok_c;
    assign chk_c[11] = cfg_ok_c;
    assign chk_c[12] = nav_fresh && ((nav_wp == 6'd1) || (resume_valid && (nav_wp == saved_wp)));
    assign chk_c[13] = slow_fresh && (batt_st == 2'd0);
    assign chk_c[14] = sticks_centered;
    assign chk_c[15] = pose_fresh && (vxy <= V_STOP_CMS) && (avz <= VZ_SETTLE_CMS);

    localparam [15:0] CHK_ALL   = 16'hFFFF;

    // ═══════════════════════════════════════════════
    //   צנרת - תנאים שדורשים חשבון רחב נרשמים מחזור אחד לפני השימוש
    // ═══════════════════════════════════════════════
    reg        near_gnd, is_top, is_ground;
    reg        z_ge_tgt, z_le_tgt, z_ge_lim, z_ge_ceil, z_lift;
    reg        v_stop, vz_set, spd_hi, spd_lo;
    reg        takeover;
    reg [15:0] chk_r;
    reg signed [11:0] lim_next, z_clear_tgt, z_min_climb, hs_min_climb, hs_plus_tol, hs_minus_tol;
    reg        clear_over, hs_ceil_bad;
    reg        disp_done, disp_far;
    reg [11:0] far_tot;            // מעבר לזה ה-DOWN עדיין רואה ראש ⇒ המכשול ארוך מהמוצהר

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            near_gnd <= 1'b0; is_top <= 1'b0; is_ground <= 1'b0;
            z_ge_tgt <= 1'b0; z_le_tgt <= 1'b0; z_ge_lim <= 1'b0; z_ge_ceil <= 1'b0; z_lift <= 1'b0;
            v_stop <= 1'b0; vz_set <= 1'b0; spd_hi <= 1'b0; spd_lo <= 1'b0;
            takeover <= 1'b1; chk_r <= 16'd0; dxr <= 14'sd0; dyr <= 14'sd0; disp <= 12'd0;
            along_raw <= 23'sd0; lat_raw <= 23'sd0; drift <= 1'b0; pcnt <= 3'd0;
            lim_next <= 12'sd0; z_clear_tgt <= 12'sd0; clear_over <= 1'b1; hs_ceil_bad <= 1'b1;
            z_min_climb <= 12'sd0; hs_min_climb <= 12'sd0; hs_plus_tol <= 12'sd0; hs_minus_tol <= 12'sd0;
            disp_done <= 1'b0; disp_far <= 1'b0; far_tot <= 12'd0; top_late <= 1'b0;
        end else begin
            near_gnd    <= near_gnd_c;
            is_top      <= is_top_c;
            is_ground   <= is_ground_c;
            z_ge_tgt    <= (pz >= tgt_z);
            z_le_tgt    <= (pz <= tgt_z);
            z_ge_lim    <= (pz >= lim_z);
            z_ge_ceil   <= (pz >= ceil_s);
            z_lift      <= (pz >= TO_LIFT_CM);
            v_stop      <= (vxy <= V_STOP_CMS);
            vz_set      <= (avz <= VZ_SETTLE_CMS);
            spd_hi      <= (vxy >= CROSS_SPEED_MAX_CMS);
            spd_lo      <= (vxy <= CROSS_SPEED_RESUME_CMS);
            // ── צנרת התזוזה: d → d·u, d×u → disp / drift ──
            //   x0/y0 נטענים עם המעבר. עד שהצנרת מתמלאת מחדש (pcnt) הכל 0 - כיוון בטוח.
            pcnt        <= entry ? 3'd0 : ((pcnt != 3'd7) ? pcnt + 3'd1 : pcnt);
            dxr         <= dx;
            dyr         <= dy;
            along_raw   <= dxr * ux + dyr * uy;
            lat_raw     <= dyr * ux - dxr * uy;
            disp        <= (pcnt < 3'd6) ? 12'd0 : along[15] ? 12'd0 :
                           (|along[14:12]) ? 12'd4095 : along[11:0];
            drift       <= (pcnt >= 3'd6) && dir_ok &&
                           ((lat_abs > CROSS_LAT_MAX_CM) || (along < -CROSS_BACK_MAX_CM));
            takeover    <= takeover_c;
            chk_r       <= chk_c;
            lim_next    <= lim_next_c;
            z_clear_tgt <= z_clear_c[11:0];
            clear_over  <= (z_clear_c > lim_z) || (z_clear_c > ceil_m);
            hs_ceil_bad <= (z_clear_c + P_MIN_CLIMB > ceil_m);
            z_min_climb  <= pz + P_MIN_CLIMB;
            // רק אחרי שהצנרת התמלאה מנקודת הייחוס הנוכחית
            disp_done    <= (pcnt == 3'd7) && dir_ok && (disp >= cross_tot);
            disp_far     <= (pcnt == 3'd7) && dir_ok && (disp >= far_tot);
            top_late     <= (pcnt == 3'd7) && dir_ok && clr_inf && (disp >= conf_tot);
            far_tot      <= cross_tot + P_EXTRA[11:0];
            hs_min_climb <= h_start + P_MIN_CLIMB;
            hs_plus_tol  <= h_start + P_ALT_TOL;
            hs_minus_tol <= h_start - P_ALT_TOL;
        end
    end
    assign checks = chk_r;

    // ═══════════════════════════════════════════════
    //   יציאות
    // ═══════════════════════════════════════════════
    wire act      = (st != S_IDLE) && (st != S_COMPLETE) && (st != S_SUSPENDED);
    wire in_abort = (st == S_ABORT_HOLD) || (st == S_ABORT_RTH);
    wire need_dwn = (st >= S_GROUND_REF) && (st <= S_ALT_CONFIRM);
    wire push     = (st == S_CROSS) || (st == S_REAR_EDGE) || (st == S_REAR_CLR);
    // באוויר: לפי שלב הטיסה או לפי Z - מסירה לאפליקציה דורשת מצערת מסונכרנת
    wire airborne_now = (phase >= 2'd2) || z_lift;

    always @* begin
        case (st)
            S_WP_ENTER, S_WP_ACTIVE, S_RESUME_WP: m_mode = M_WP;
            S_ABORT_RTH:                          m_mode = M_RTH;
            default:                              m_mode = M_POSHOLD;
        endcase
    end

    wire ack_ok = flg_fresh && ((m_mode == M_WP)  ? ack_wp  :
                                (m_mode == M_RTH) ? ack_rth : ack_pos);

    // ── מצערת: יעד לפי המצב, ויציאה דרך מגביל קצב ──
    wire [8:0] up_c   = {1'b0, m_neutral} + CLIMB_THR_OFFSET;
    wire [7:0] thr_climb = (up_c  > 9'd200) ? 8'd200 : up_c[7:0];
    wire [7:0] thr_desc  = (m_neutral > DESC_THR_OFFSET) ? (m_neutral - DESC_THR_OFFSET) : 8'd0;
    wire [7:0] thr_tgt   = ((st == S_CLIMB) || (st == S_CLEARANCE))  ? thr_climb :
                           (st == S_RET_ALT)                         ? thr_desc  : m_neutral;

    localparam signed [7:0] P_PITCH = (CROSS_MODE != 0) ? CROSS_PITCH_CRUISE : CROSS_PITCH_ATTI;
    wire signed [7:0] pch_tgt = (push && spd_ok) ? P_PITCH : 8'sd0;

    assign m_yaw = 8'sd0;
    assign m_rll = 8'sd0;

    assign m_active   = act;
    assign st_code    = st;
    assign stage_code = {1'b0, r_stage};
    assign flags_out  = {synced, 1'b0, gnd_vis, set_ok};   // bit2 (התחלה מהקרקע) - תמיד 0
    assign set_allow  = !act && !armed;

    // ── פסק הזמן של המצב הנוכחי: משווה אחד, והסף לפי המצב ──
    reg [13:0] tmo_lim;
    always @* begin
        case (st)
            S_HOLD_SYNC:                                   tmo_lim = T10_SYNC;
            S_CLIMB, S_CLEARANCE:                          tmo_lim = T10_CLIMB;
            S_GROUND_REF:                                  tmo_lim = T10_GREF;
            S_CROSS, S_REAR_EDGE, S_REAR_CLR:              tmo_lim = T10_CROSS;
            S_RET_ALT, S_ALT_CONFIRM:                      tmo_lim = T10_RETALT;
            default:                                       tmo_lim = T10_STOP;
        endcase
    end
    wire st_tmo  = (st_t >= tmo_lim);
    // חזרה לאחור אחרי שיא התזוזה (dmax ≥ disp תמיד בעצירה; רווי ב-255)
    wire [11:0] reb  = dmax - disp;
    wire [12:0] rear_tot = {1'b0, disp} + {3'd0, P_HALF[9:0]} + {3'd0, r_fclr};
    wire [7:0]  reb_sat = reb[11] ? 8'd0 : (|reb[10:8]) ? 8'hFF : reb[7:0];

    wire nav_bad   = (nav_err == 4'd1) || (nav_err == 4'd2) || (nav_err == 4'd3) ||
                     (nav_err == 4'd6) || (nav_err == 4'd7) || (nav_err == 4'd9);
    wire nav_emerg = (nav_mode == NAVM_EMERG) || (nav_state == 5'd14);
    wire nav_fin   = nav_fresh && (nav_err == NAVE_FINISH);
    wire front_obst  = (obst_cnt >= OBST_DETECT_N);
    wire front_clear = (clr_cnt  >= FRONT_CLEAR_N);
    wire front_nr    = (nr_cnt   >= FRONT_CLEAR_N);

    // דגימת DOWN שמפרה מרווח מינימלי
    wire gap_bad      = dwn_evt && (d_close || (d_valid && (lidar_dwn < MIN_GAP_CM)));
    wire gap_bad_desc = dwn_evt && (d_close || (d_valid && (lidar_dwn < MIN_GAP_DESC_CM)));

    // DOWN במעבר: "ראש" ו"פנוי". כשהקרקע לא נראתה ב-GROUND_REF (מעל הטווח),
    // כל VALID הוא משטח בתוך הטווח - כלומר המכשול - ו-OUT_OF_RANGE הוא פנוי.
    wire top_smp   = d_valid && (gnd_vis ? is_top : 1'b1);
    wire clear_smp = gnd_vis ? (d_valid && is_ground) : d_oor;

    // ═══════════════════════════════════════════════
    //   מונים - מתאפסים ב-entry
    // ═══════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_t <= 14'd0; still_t <= 9'd0; vz_t <= 9'd0; band_t <= 9'd0;
            obst_cnt <= 4'd0; clr_cnt <= 4'd0; nr_cnt <= 4'd0; blk_seen <= 1'b0;
            d_cnt <= 4'd0; oor_cnt <= 4'd0; alt_cnt <= 2'd0;
            spd_ok <= 1'b1; obst_win <= 7'd0;
        end else if (entry) begin
            st_t <= 14'd0; still_t <= 9'd0; vz_t <= 9'd0; band_t <= 9'd0;
            obst_cnt <= 4'd0; clr_cnt <= 4'd0; nr_cnt <= 4'd0;
            d_cnt <= 4'd0; oor_cnt <= 4'd0; alt_cnt <= 2'd0;
            spd_ok <= 1'b1;
        end else begin
            if (tick) begin
                if (st_t != 14'h3FFF) st_t <= st_t + 14'd1;
                if (obst_win != 7'h7F) obst_win <= obst_win + 7'd1;

                if (ack_ok && pose_fresh && v_stop) begin
                    if (still_t != 9'h1FF) still_t <= still_t + 9'd1;
                end else still_t <= 9'd0;

                if (pose_fresh && vz_set) begin
                    if (vz_t != 9'h1FF) vz_t <= vz_t + 9'd1;
                end else vz_t <= 9'd0;

                if (pose_fresh && z_le_tgt && z_ge_lim && vz_set) begin
                    if (band_t != 9'h1FF) band_t <= band_t + 9'd1;
                end else band_t <= 9'd0;
            end

            // ── FRONT: מכשול ──
            if (fwd_evt) begin
                if (f_hit) begin
                    if (obst_cnt != 4'hF) obst_cnt <= obst_cnt + 4'd1;
                    obst_win <= 7'd0;
                end
                else if (f_valid) obst_cnt <= 4'd0;
            end
            else if (obst_win >= T10_OBSTWIN) obst_cnt <= 4'd0;

            // ── FRONT: הקרן פנויה - רצף אמיתי: כל דגימה אחרת מאפסת ──
            if (fwd_evt) begin
                if (f_clr) begin
                    if (clr_cnt != 4'hF) clr_cnt <= clr_cnt + 4'd1;
                end
                else clr_cnt <= 4'd0;
            end

            // ── FRONT: אין החזר בטיפוס (ראיה חלקית) ──
            //   נספר רק אחרי שה-FRONT ראה את המכשול מאז העצירה (blk_seen: מתאפס ב-OBST_STOP,
            //   נשמר גם כשה-CLEARANCE חוזר ל-CLIMB אחרי זיהוי חוזר), ברצף של דגימות טריות,
            //   ובלי נסיגה / סטייה צדית מנקודת העצירה. כל דגימה אחרת, נתון ישן או
            //   סטייה מאפסים.
            if (st == S_OBST_STOP)                       blk_seen <= 1'b0;
            else if (fwd_evt && f_blk)                   blk_seen <= 1'b1;

            if (st == S_CLIMB && (!sens_fresh || drift)) nr_cnt <= 4'd0;
            else if (fwd_evt) begin
                if (f_blk) nr_cnt <= 4'd0;
                else if (f_nr && blk_seen) begin
                    if (nr_cnt != 4'hF) nr_cnt <= nr_cnt + 4'd1;
                end
                else nr_cnt <= 4'd0;
            end

            // ── DOWN: מונה רצף לפי המצב ──
            if (dwn_evt) begin
                case (st)
                    S_GROUND_REF: begin
                        if (d_valid) begin
                            if ((lidar_dwn > dwn_gmax) || (lidar_dwn < MIN_GAP_CM)) d_cnt <= 4'd0;
                            else if (d_cnt == 4'd0 || !near_gnd)                    d_cnt <= 4'd1;
                            else if (d_cnt != 4'hF)                                 d_cnt <= d_cnt + 4'd1;
                            oor_cnt <= 4'd0;
                        end
                        else if (d_oor) begin
                            if (oor_cnt != 4'hF) oor_cnt <= oor_cnt + 4'd1;
                        end
                    end
                    S_CROSS: begin
                        if (top_smp)           begin if (d_cnt != 4'hF) d_cnt <= d_cnt + 4'd1; end
                        else if (d_valid || clear_smp) d_cnt <= 4'd0;
                    end
                    S_REAR_EDGE: begin
                        if (clear_smp)         begin if (d_cnt != 4'hF) d_cnt <= d_cnt + 4'd1; end
                        else if (top_smp)      d_cnt <= 4'd0;
                    end
                    default: ;
                endcase
            end

            // ── אישור גובה בשתי דגימות מיקום, ומגביל המהירות במעבר ──
            if (pose_evt) begin
                if (((st == S_CLEARANCE) && z_ge_tgt) || ((st == S_RET_ALT) && z_le_tgt)) begin
                    if (alt_cnt != 2'd3) alt_cnt <= alt_cnt + 2'd1;
                end else alt_cnt <= 2'd0;

                if (spd_hi)      spd_ok <= 1'b0;
                else if (spd_lo) spd_ok <= 1'b1;
            end
        end
    end

    // ── המועמד לגובה הקרקע ב-GROUND_REF ──
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cand <= 12'sd0;
        else if (!entry && dwn_evt && d_valid && (st == S_GROUND_REF) && (d_cnt == 4'd0 || !near_gnd))
            cand <= surf;
    end

    // ── מונים גלובליים ──
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_bad_t <= 9'd0; fwa_t <= 9'd0;
        end else if (tick) begin

            if (ack_ok || !act || entry)     ack_bad_t <= 9'd0;
            else if (ack_bad_t != 9'h1FF)    ack_bad_t <= ack_bad_t + 9'd1;

            if (fwd_active || !sens_fresh)   fwa_t <= 9'd0;
            else if (fwa_t != 9'h1FF)        fwa_t <= fwa_t + 9'd1;
        end
    end

    // ── התאמת המצערת ב-HOLD_SYNC: נספרת רק מהכניסה למצב, ומתאפסת בכל חריגה ──
    //   כך ההחלטה נשענת על שורות שליטה שלמות שהגיעו **אחרי** הבקשה למסור:
    //   ב-cmd_parser מצב הטיסה מתעדכן באמצע השורה והמצערת רק בסופה, ולכן
    //   החלטה מיידית יכלה לראות מצב חדש עם מצערת ישנה.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                           sync_t <= 5'd0;
        else if (st != S_HOLD_SYNC || entry || !thr_in_sync)  sync_t <= 5'd0;
        else if (tick && sync_t != 5'h1F)                     sync_t <= sync_t + 5'd1;
    end

    // ── מגביל הקצב של המצערת וה-Pitch ──
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_thr <= 8'd0;
            m_pch <= 8'sd0;
        end else if (!act) begin
            m_thr <= thr_app;            // נקודת ההתחלה של המשימה הבאה = מה שעובר עכשיו
            m_pch <= 8'sd0;
        end else if (tick) begin
            if (m_thr + THR_SLEW_PER_10MS <= thr_tgt)       m_thr <= m_thr + THR_SLEW_PER_10MS;
            else if (m_thr >= thr_tgt + THR_SLEW_PER_10MS)  m_thr <= m_thr - THR_SLEW_PER_10MS;
            else                                            m_thr <= thr_tgt;

            if (m_pch < pch_tgt)      m_pch <= m_pch + 8'sd1;
            else if (m_pch > pch_tgt) m_pch <= m_pch - 8'sd1;
        end
    end

    // ═══════════════════════════════════════════════
    //   מעבר מצב
    // ═══════════════════════════════════════════════
    task go;
        input [4:0] ns;
        begin
            st    <= ns;
            entry <= 1'b1;
        end
    endtask

    // מסירה לטייס באוויר - תמיד דרך HOLD_SYNC: POSHOLD בנייטרל, ושחרור רק כשמצערת
    // האפליקציה תואמת לנייטרל **עכשיו**, ברצף 200ms (≥3 שורות שליטה שלמות) - ולא
    // לפי סנכרון שהיה פעם. כשהיא כבר תואמת זה עולה 200ms של החזקה בלבד.
    // חל על כל מסירה: STOP, השתלטות במקלות / במצערת, שינוי מצב, כשל, וסיום משימה.
    // ניתוק / ABORT / אובדן קשר (שכבת הבטיחות) עוקפים את זה, כמו תמיד.
    task release_ctl;
        begin
            if (airborne_now)            go(S_HOLD_SYNC);
            else if (fin_ok)             go(S_COMPLETE);
            else                         go(S_SUSPENDED);
        end
    endtask

    // כשל: SUSPENDED (דרך release_ctl), או ABORT_POLICY כשהכשל מוגדר "חמור"
    task fail;
        input [7:0] r;
        input       severe;
        begin
            reason <= r;
            fin_ok <= 1'b0;
            if (severe && ABORT_POLICY == 1)      go(S_ABORT_HOLD);
            else if (severe && ABORT_POLICY == 2) go(S_ABORT_RTH);
            else                                  release_ctl;
        end
    endtask

    // תחילת משימה מאושרת - באוויר בלבד
    task begin_mission;
        begin
            thr_start <= thr_app;
            // הנייטרל של INAV: ב-MID_STICK - thr_mid. ב-STICK - המצערת שנתפסה בכניסה
            // ל-POSHOLD, כלומר זו של הטייס.
            m_neutral <= (cfg_atm == 2'd1) ? mid_half : thr_app;
            synced    <= !(cfg_atm == 2'd1 && thr_app != mid_half);
            fin_ok    <= 1'b0;
            gnd_vis   <= 1'b0;
            exp_wp    <= (resume_valid && (nav_wp == saved_wp)) ? saved_wp : 6'd1;
            reason    <= R_NONE;
            mission_s <= 10'd0;
            det_cm    <= 10'h3FF;
            stop_cm   <= 10'h3FF;
            clr_inf   <= 1'b0;
            det_v     <= 10'd0;
            t_ack     <= 8'hFF;
            t_stop    <= 10'd0;
            trav      <= 10'd0;
            rebound   <= 8'd0;
            go(S_HOLD_START);
        end
    endtask

    // ═══════════════════════════════════════════════
    //   מכונת המצבים
    // ═══════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            entry        <= 1'b0;
            dir_go       <= 1'b0;
            synced       <= 1'b0;
            fin_ok       <= 1'b0;
            gnd_vis      <= 1'b0;
            ack_seen     <= 1'b0;
            thr_start    <= 8'd0;
            m_neutral    <= 8'd130;
            exp_wp       <= 6'd1;
            resume_valid <= 1'b0;
            saved_wp     <= 6'd0;
            last_wp      <= 6'd0;
            reason       <= R_NONE;
            det_cm       <= 10'h3FF;
            stop_cm      <= 10'h3FF;
            clr_inf      <= 1'b0;
            conf_tot     <= 12'd0;
            det_v        <= 10'd0;
            t_ack        <= 8'hFF;
            t_stop       <= 10'd0;
            trav         <= 10'd0;
            rebound      <= 8'd0;
            dmax         <= 12'd0;
            h_start      <= 12'sd0;
            tgt_z        <= 12'sd0;
            lim_z        <= 12'sd0;
            x0           <= 14'd0;
            y0           <= 14'd0;
            gref         <= 12'sd0;
            cross_tot    <= 12'd0;
            wp_t         <= 14'd0;
            mission_s    <= 10'd0;
        end else begin
            entry  <= 1'b0;
            dir_go <= 1'b0;

            if (tick && wp_t != 14'h3FFF) wp_t <= wp_t + 14'd1;
            if (s_tick && act && mission_s != 10'h3FF) mission_s <= mission_s + 10'd1;

            if (!armed) resume_valid <= 1'b0;

            // ── המצערת של האפליקציה הגיעה לנייטרל: מעכשיו שינוי בה הוא השתלטות ──
            if (act && thr_in_sync) synced <= 1'b1;

            // ═══════════════════════════════════════
            //   לא פעיל: IDLE / COMPLETE / SUSPENDED - מעבר ישיר
            // ═══════════════════════════════════════
            if (!act) begin
                if (cmd_mstart) begin
                    if (chk_r == CHK_ALL) begin_mission;
                    else begin reason <= R_START_REJECTED; go(S_IDLE); end
                end
            end

            // ═══════════════════════════════════════
            //   פעיל: קודם הבדיקות הגלובליות, אחר כך המצב עצמו
            // ═══════════════════════════════════════
            else if (!armed || rc_hold) begin reason <= R_SAFETY; fin_ok <= 1'b0; go(S_SUSPENDED); end
            else if (entry)                                    ;
            else if (st == S_HOLD_SYNC) begin
                // מחזיקים POSHOLD עד שמצערת האפליקציה תואמת, ואז מוסרים.
                // ⚠️ אין מסירה לא מסונכרנת באוויר - גם לא אחרי פסק זמן: אחרי T_SYNC
                //    רק מדווחים SYNC_TIMEOUT וממשיכים להחזיק. יציאה אחרת: הרחפן
                //    על הקרקע, או שכבת הבטיחות (ניתוק / ABORT / אובדן קשר) - למעלה.
                if (thr_held || !airborne_now) go(fin_ok ? S_COMPLETE : S_SUSPENDED);
                else if (st_tmo && !fin_ok)    reason <= R_SYNC_TIMEOUT;
            end
            // המשימה הושלמה (DONE_HOLD): STOP או השתלטות מוסרים את השליטה ⇒ COMPLETE
            else if ((cmd_mstop || takeover) && (st == S_DONE_HOLD)) release_ctl;
            else if (cmd_mstop)                                fail(R_PILOT_STOP, 1'b0);
            else if (takeover)                                 fail(R_TAKEOVER, 1'b0);
            else if (!flg_fresh)                               fail(R_STALE_FLAGS, 1'b0);
            else if (fm_flags[FM_FAILSAFE])                    fail(R_FAILSAFE, 1'b0);
            else if (!pose_fresh)                              fail(R_STALE_POSE, 1'b0);
            else if (!nav_fresh)                               fail(R_STALE_NAV, 1'b0);
            else if (!sens_fresh)                              fail(R_STALE_SENS, 1'b0);
            else if (!slow_fresh)                              fail(R_STALE_SLOW, 1'b0);
            else if (!mag_ok)                                  fail(R_MAG, 1'b0);
            else if (!gps_fresh)                               fail(R_STALE_GPS, 1'b0);
            else if (!gps_ok)                                  fail(R_GPS, 1'b0);
            else if (!inav_armed)                              fail(R_INAV_DISARMED, 1'b0);
            else if (!front_ok)                                fail(R_STALE_FRONT, 1'b0);
            else if (!down_ok && need_dwn)                     fail(R_STALE_DOWN, 1'b0);
            else if (ack_bad_t >= T10_ACK)                     fail(R_MODE_ACK, 1'b0);
            else if (!in_abort && (batt_st == 2'd2))           fail(R_BATTERY, 1'b1);
            else if (!in_abort && (mission_s >= MISSION_MAX_S)) fail(R_MISSION_TIME, 1'b1);
            else begin
                case (st)

                // ═══ התחלה באוויר: POSHOLD יציב ═══
                S_HOLD_START: begin
                    if (still_t >= T10_SETTLE) go(S_WP_ENTER);
                    else if (st_tmo) fail(R_STOP_TIMEOUT, 1'b0);
                end

                // ═══ כניסה ל-WP: רק אחרי ACK, ורק אם ה-WP הפעיל הוא הצפוי ═══
                S_WP_ENTER: begin
                    if (ack_ok) begin
                        if (nav_wp == exp_wp) begin
                            last_wp <= nav_wp;
                            wp_t    <= 14'd0;
                            go(S_WP_ACTIVE);
                        end
                        else fail(R_WP_MISMATCH, 1'b0);
                    end
                end

                S_WP_ACTIVE: begin
                    saved_wp     <= nav_wp;
                    resume_valid <= 1'b1;
                    if (nav_evt && (nav_wp != last_wp)) begin
                        last_wp <= nav_wp;
                        wp_t    <= 14'd0;
                    end

                    if (front_obst) begin
                        // נתוני הבלימה: נקודת הזיהוי, המהירות והמרחק ברגע ההחלטה
                        det_cm   <= {1'b0, lidar_fwd};
                        det_v    <= (vxy > 13'd1023) ? 10'd1023 : vxy[9:0];
                        x0       <= px;
                        y0       <= py;
                        dir_go   <= 1'b1;        // כיוון המעבר = כיוון הטיסה עכשיו
                        ack_seen <= 1'b0;
                        dmax     <= 12'd0;
                        go(S_OBST_STOP);
                    end
                    else if (nav_fin)                  go(S_WP_DONE);
                    else if (nav_bad)                  fail(R_NAV_ERROR, 1'b0);
                    else if (nav_emerg)                fail(R_INAV_EMERG, 1'b0);
                    else if (fwa_t >= T10_FWALOSS)     fail(R_FRONT_INACTIVE, 1'b0);
                    else if (wp_t >= T10_STALL)        fail(R_WP_STALL, 1'b0);
                end

                // ═══ OBSTACLE_STOP: POSHOLD ועצירה מאומתת. INAV בולם ═══
                S_OBST_STOP: begin
                    if (ack_ok && !ack_seen) begin
                        ack_seen <= 1'b1;
                        t_ack    <= (|st_t[13:8]) ? 8'hFE : st_t[7:0];
                    end
                    if (disp > dmax) dmax <= disp;

                    if (still_t >= T10_STOPPED) begin
                        t_stop  <= (|st_t[13:10]) ? 10'h3FF : st_t[9:0];
                        trav    <= (|disp[11:10]) ? 10'h3FF : disp[9:0];
                        rebound <= reb_sat;
                        h_start <= pz;
                        x0      <= px;          // מכאן נמדד המעבר: נקודת העצירה
                        y0      <= py;
                        lim_z   <= lim_next;
                        tgt_z   <= z_min_climb;
                        stop_cm <= f_valid ? {1'b0, lidar_fwd} : 10'h3FF;
                        clr_inf <= 1'b0;
                        if (r_stage <= 3'd2)                       fail(R_STAGE_END, 1'b0);
                        else if (!dir_ok)                          fail(R_NO_DIRECTION, 1'b0);
                        else if (!f_valid)                         fail(R_OBST_LOST, 1'b0);
                        else if (lidar_fwd < AVOID_MIN_FRONT_CM)   fail(R_STOP_TOO_CLOSE, 1'b0);
                        else if (hs_ceil_bad)                      fail(R_CEILING, 1'b0);
                        else                                       go(S_CLIMB);
                    end
                    else if (st_tmo) fail(R_STOP_TIMEOUT, 1'b0);
                end

                // ═══ טיפוס עד שה-FRONT פנוי ═══
                S_CLIMB: begin
                    if ((front_clear || front_nr) && z_ge_tgt) begin
                        tgt_z   <= z_clear_tgt;
                        clr_inf <= !front_clear;      // אין-החזר בלבד ⇒ חובה לאשר ראש ב-DOWN
                        if (clear_over) fail(R_CEILING, 1'b1);
                        else            go(S_CLEARANCE);
                    end
                    else if (z_ge_lim)            fail(R_CEILING, 1'b1);
                    else if (st_tmo)   fail(R_CLIMB_TIMEOUT, 1'b1);
                end

                // ═══ עוד מרווח אנכי (ברירת מחדל 70cm), מאושר בשתי דגימות ═══
                S_CLEARANCE: begin
                    if (front_obst) begin       // ה-FRONT רואה שוב מכשול - ההתנקות הייתה מוקדמת
                        tgt_z <= hs_min_climb;
                        go(S_CLIMB);
                    end
                    else if (alt_cnt >= 2'd2)     go(S_CLEAR_HOLD);
                    else if (z_ge_lim)            fail(R_CEILING, 1'b1);
                    else if (st_tmo)   fail(R_CLIMB_TIMEOUT, 1'b1);
                end

                S_CLEAR_HOLD: begin
                    if (vz_t >= T10_SETTLE) begin
                        if (r_stage <= 3'd3) fail(R_STAGE_END, 1'b0);
                        else                 go(S_GROUND_REF);
                    end
                    else if (st_tmo) fail(R_STOP_TIMEOUT, 1'b0);
                end

                // ═══ GROUND_REF: האם DOWN רואה קרקע. לא חוסם ═══
                //   VALID עקבי בטווח ⇒ גובה הקרקע ידוע. OUT_OF_RANGE עקבי, או פסק זמן ⇒
                //   מעבר לפי עומק המכשול, ו-DOWN יזהה את המכשול רק אם יגיע לטווח.
                S_GROUND_REF: begin
                    if (gap_bad)                                   fail(R_MIN_GAP, 1'b0);
                    else if (d_cnt >= GND_N || oor_cnt >= GND_N || st_tmo) begin
                        gnd_vis   <= (d_cnt >= GND_N);
                        gref      <= cand;
                        // מרחק העצירה + עומק + אורך הרחפן + מרווח + אי-ודאות המיקום (EPH/2)
                        // אין-החזר: ה-DOWN (במרכז) מגיע לחזית המכשול אחרי העצירה + (אורך - חצי)
                        conf_tot  <= ((stop_cm == 10'h3FF) ? {2'd0, r_stop} : {2'd0, stop_cm}) +
                                     (DRONE_LEN_CM - DRONE_HALF_CM + TOP_CONFIRM_MARGIN_CM);
                        cross_tot <= ((stop_cm == 10'h3FF) ? {2'd0, r_stop} : {2'd0, stop_cm}) +
                                     {2'd0, r_depth} + {2'd0, r_fclr} + {3'd0, gps_eph[9:1]} +
                                     (DRONE_LEN_CM + POS_MARGIN_CM);
                        // בלי DOWN על הקרקע המעבר נשען רק על המיקום ⇒ רק עם GPS טוב
                        if (d_cnt < GND_N && gps_eph > CROSS_BLIND_EPH_CM) fail(R_GPS_BLIND, 1'b0);
                        else                                               go(S_CROSS);
                    end
                end

                // ═══ CROSS: קדימה לאט. DOWN רואה ראש ⇒ REAR_EDGE; אחרת לפי המרחק ═══
                S_CROSS: begin
                    if (gap_bad)                                  fail(R_MIN_GAP, 1'b0);
                    else if (drift)                               fail(R_CROSS_DRIFT, 1'b0);
                    else if (front_obst)                          fail(R_FRONT_OBST, 1'b0);
                    else if (d_cnt >= TOP_N)                      go(S_REAR_EDGE);
                    else if (top_late)                            fail(R_TOP_UNCONFIRMED, 1'b0);
                    else if (disp_done)                           go(S_REAR_STOP);
                    else if (st_tmo)                   fail(R_CROSS_TIMEOUT, 1'b0);
                end

                // ═══ REAR_EDGE: ממשיכים עד ש-DOWN רואה שוב קרקע / מחוץ לטווח ═══
                S_REAR_EDGE: begin
                    if (gap_bad)                                  fail(R_MIN_GAP, 1'b0);
                    else if (drift)                               fail(R_CROSS_DRIFT, 1'b0);
                    else if (front_obst)                          fail(R_FRONT_OBST, 1'b0);
                    else if (d_cnt >= REAR_N) begin
                        // ה-DOWN (במרכז) עבר את הקצה האחורי: מכאן עוד חצי רחפן + המרווח
                        // האופקי, לאורך אותו קו מעבר ומאותה נקודת ייחוס (העצירה)
                        cross_tot <= (rear_tot[12]) ? 12'd4095 : rear_tot[11:0];
                        go(S_REAR_CLR);
                    end
                    else if (disp_far)                            fail(R_OBST_TOO_LONG, 1'b0);
                    else if (st_tmo)                   fail(R_CROSS_TIMEOUT, 1'b0);
                end

                // ═══ REAR_CLR: לפי תזוזת INAV, לא לפי זמן ═══
                S_REAR_CLR: begin
                    if (gap_bad)                                  fail(R_MIN_GAP, 1'b0);
                    else if (drift)                               fail(R_CROSS_DRIFT, 1'b0);
                    else if (front_obst)                          fail(R_FRONT_OBST, 1'b0);
                    else if (disp_done)                           go(S_REAR_STOP);
                    else if (st_tmo)                   fail(R_CROSS_TIMEOUT, 1'b0);
                end

                S_REAR_STOP: begin
                    if (still_t >= T10_STOPPED) begin
                        tgt_z <= hs_plus_tol;
                        lim_z <= hs_minus_tol;
                        if (r_stage <= 3'd4)      fail(R_STAGE_END, 1'b0);
                        else                      go(S_RET_ALT);
                    end
                    else if (st_tmo) fail(R_STOP_TIMEOUT, 1'b0);
                end

                // ═══ חזרה לגובה המשימה: ירידה אנכית בלבד ═══
                S_RET_ALT: begin
                    if (gap_bad_desc)                 fail(R_MIN_GAP, 1'b0);
                    else if (alt_cnt >= 2'd2)         go(S_ALT_CONFIRM);
                    else if (front_obst)              fail(R_FRONT_OBST, 1'b0);
                    else if (st_tmo)      fail(R_ALT_TIMEOUT, 1'b0);
                end

                S_ALT_CONFIRM: begin
                    if (band_t >= T10_SETTLE)         go(S_RESUME_WP);
                    else if (front_obst)              fail(R_FRONT_OBST, 1'b0);
                    else if (st_tmo)      fail(R_ALT_TIMEOUT, 1'b0);
                end

                // ═══ RESUME_WP: חייב לחזור ל-WP שנקטע ═══
                S_RESUME_WP: begin
                    if (ack_ok) begin
                        if (nav_wp == saved_wp) begin
                            last_wp <= nav_wp;
                            wp_t    <= 14'd0;
                            go(S_WP_ACTIVE);
                        end
                        else fail(R_WP_MISMATCH, 1'b0);
                    end
                end

                // ═══ סיום: POSHOLD בנקודה B ═══
                S_WP_DONE: begin
                    if (still_t >= T10_STOPPED) begin
                        reason       <= R_NONE;
                        fin_ok       <= 1'b1;
                        resume_valid <= 1'b0;
                        go(S_DONE_HOLD);
                    end
                    else if (st_tmo) fail(R_STOP_TIMEOUT, 1'b0);
                end

                S_DONE_HOLD:  ;
                S_ABORT_HOLD: ;
                S_ABORT_RTH:  ;

                default: fail(R_SAFETY, 1'b0);
                endcase
            end
        end
    end

endmodule

// ═══════════════════════════════════════════════════════════
//   גיל מקור מידע ביחידות tick, רווי ב-511 (= "מעולם לא התקבל")
// ═══════════════════════════════════════════════════════════
module mission_age (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,
    input  wire       clr,
    output reg  [8:0] age
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                        age <= 9'h1FF;
        else if (clr)                      age <= 9'd0;
        else if (tick && age != 9'h1FF)    age <= age + 9'd1;
    end
endmodule

// ═══════════════════════════════════════════════════════════
//   mission_dir - כיוון המעבר: u = v / |v| בסקאלה 128
//
//   נדגם פעם אחת, ברגע הזיהוי (start). סדרתי, כ-17 מחזורי שעון:
//     |v| = isqrt(vx² + vy²) + 1  - מעוגל כלפי מעלה, ולכן |u| ≤ 1:
//                                   ההתקדמות המחושבת לעולם אינה גדולה מהאמיתית
//     ux = vx·128 / |v|,  uy = vy·128 / |v|   (חלוקה ב-7 ביטים)
//   ok = 1 רק כש-|v| ≥ DIR_MIN (cm/s). עד הסוף u = 0 ⇒ אין התקדמות.
//   הרכיבים מוגבלים ל-±127 cm/s (מהירות WP מותרת עד 100).
// ═══════════════════════════════════════════════════════════
module mission_dir #(
    parameter DIR_MIN = 20
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [11:0] vx,
    input  wire signed [11:0] vy,
    output reg  signed [8:0]  ux,
    output reg  signed [8:0]  uy,
    output reg                ok
);
    wire [11:0] avx = vx[11] ? (12'd0 - vx) : vx;
    wire [11:0] avy = vy[11] ? (12'd0 - vy) : vy;
    wire [6:0]  cx  = (avx > 12'd127) ? 7'd127 : avx[6:0];
    wire [6:0]  cy  = (avy > 12'd127) ? 7'd127 : avy[6:0];

    reg [1:0]  ph;          // 0 מנוחה · 1 שורש · 2 חלוקה
    reg [2:0]  k;
    reg [15:0] num;         // vx² + vy² (≤ 32258)
    reg [10:0] rem;         // ≤ 2·root + 1
    reg [7:0]  root;
    reg [6:0]  ax, ay;
    reg        sx, sy;
    reg [8:0]  m;           // root + 1
    reg [8:0]  rx, ry;      // שאריות החלוקה
    reg [6:0]  qx, qy;

    wire [12:0] rem_sh = {rem, num[15:14]};
    wire [12:0] trial  = {3'd0, root, 2'b01};
    wire [9:0] rx_sh  = {rx[8:0], 1'b0};
    wire [9:0] ry_sh  = {ry[8:0], 1'b0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph <= 2'd0; k <= 3'd0; num <= 16'd0; rem <= 11'd0; root <= 8'd0;
            ax <= 7'd0; ay <= 7'd0; sx <= 1'b0; sy <= 1'b0; m <= 9'd0;
            rx <= 9'd0; ry <= 9'd0; qx <= 7'd0; qy <= 7'd0;
            ux <= 9'sd0; uy <= 9'sd0; ok <= 1'b0;
        end else if (start) begin
            ux <= 9'sd0; uy <= 9'sd0; ok <= 1'b0;
            ax <= cx; ay <= cy; sx <= vx[11]; sy <= vy[11];
            num  <= cx * cx + cy * cy;
            rem  <= 11'd0; root <= 8'd0; k <= 3'd7; m <= 9'd0;
            ph   <= 2'd1;
        end else if (ph == 2'd1) begin
            // שורש בשיטת ספרה-אחר-ספרה: 8 צעדים
            num <= {num[13:0], 2'b00};
            if (rem_sh >= trial) begin rem <= rem_sh[10:0] - trial[10:0]; root <= {root[6:0], 1'b1}; end
            else begin                 rem <= rem_sh[10:0];                root <= {root[6:0], 1'b0}; end
            if (k == 3'd0) ph <= 2'd2; else k <= k - 3'd1;
            if (k == 3'd0) begin
                // root עוד לא כולל את הצעד הזה - m נטען בשלב הבא
                rx <= {2'd0, ax}; ry <= {2'd0, ay}; qx <= 7'd0; qy <= 7'd0;
            end
        end else if (ph == 2'd2) begin
            if (k == 3'd0 && m == 9'd0) begin
                m <= {1'b0, root} + 9'd1;   // צעד ראשון של השלב: רק טעינת המחלק
            end else begin
                // חלוקה: 7 ביטים של a/m (a < m תמיד)
                if (rx_sh >= {1'b0, m}) begin rx <= rx_sh[8:0] - m; qx <= {qx[5:0], 1'b1}; end
                else begin                    rx <= rx_sh[8:0];     qx <= {qx[5:0], 1'b0}; end
                if (ry_sh >= {1'b0, m}) begin ry <= ry_sh[8:0] - m; qy <= {qy[5:0], 1'b1}; end
                else begin                    ry <= ry_sh[8:0];     qy <= {qy[5:0], 1'b0}; end
                if (k == 3'd6) begin
                    ph <= 2'd3;
                end else k <= k + 3'd1;
            end
        end else if (ph == 2'd3) begin
            ux <= sx ? -$signed({2'b00, qx}) : $signed({2'b00, qx});
            uy <= sy ? -$signed({2'b00, qy}) : $signed({2'b00, qy});
            ok <= (root >= DIR_MIN);
            m  <= 9'd0;
            ph <= 2'd0;
        end
    end
endmodule
