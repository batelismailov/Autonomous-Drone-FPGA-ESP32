`timescale 1us/1ns
// ═══════════════════════════════════════════════════════════
//   tb_mission.sv - בדיקת ה-Mission FSM בלולאה סגורה
//
//   drone_top מחובר כמו במציאות, דרך ה-UART:
//     esp_rx ← מודל ה-ESP: שליטה (כולל RPL), חיישנים (כולל FLM/DLM), INAV מהיר,
//              איטי (כולל CTM/MAL), שורת ההגדרות MSE, ו-MSTAT
//     fc_tx  → מפענח חבילות MSP_SET_RAW_RC → מודל INAV + עולם עם מכשולים
//     esp_tx → מפענח שורות MST (56 תווים)
//
//   מודל INAV 9.1 (מפושט):
//     · חימוש רק ב-ANGLE ובמצערת נמוכה על הקרקע.
//     · ACK למצב אחרי 50ms. כניסה למצב שמחזיק גובה: על הקרקע, או ב-MID_STICK,
//       הנייטרל הוא thr_mid; באוויר ב-STICK - המצערת שבכניסה.
//     · POSHOLD במצב CRUISE: Pitch = מהירות (deadband 10µs); מקלות במרכז ⇒ בלימה.
//     · מצערת מעל/מתחת לנייטרל (deadband 50µs) ⇒ קצב אנכי עד 100cm/s.
//     · WP במהירות 50cm/s, RESUME, ובסוף המשימה NER=4 (FINISH) והחזקת מיקום.
//       אין LAND ואין RTH במשימה.
//   חיישנים:
//     FRONT - מקצה הפרופלור (30cm לפני המרכז), 8cm מעל התחתית, קרן ±13.5°.
//             פוגע בחזית, בראש המכשול, בקרקע או בקיר רחוק. LONG 400cm · SHORT 130cm.
//     DOWN  - ממרכז התחתית. SHORT 130cm · LONG 400cm.
//
//   front_nr_cls: מה ה-FRONT מחזיר כשאין מטרה בטווח - 2 OUT_OF_RANGE (ברירת מחדל)
//                 או 6 NO_SIGNAL (כמו בשטח פתוח: status 2). angle_vz: ב-ANGLE המצערת
//                 מזיזה את הרחפן אנכית (המראה ידנית), רק בתרחישים שמבקשים.
//   הרצה: +SCEN=<n>. כל התרחישים, מתוך FPGA-Drone/sim:
//     python3 run_mission.py .. /tmp/tbm
// ═══════════════════════════════════════════════════════════
module tb_mission;
    localparam integer CLK  = 1_000_000;
    localparam integer BAUD = 100_000;
    localparam integer BIT  = CLK / BAUD;

    reg clk = 1'b0;
    always #0.5 clk = ~clk;

    reg  esp_rx = 1'b1;
    wire esp_tx, fc_tx;
    wire [2:0] led;

    drone_top #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD)) dut (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(esp_tx), .fc_tx(fc_tx), .fc_rx(1'b1), .led(led));

    int scen = 1;
    int errors = 0;
    task automatic check(bit cond, string msg);
        if (cond) $display("[%8.2f s] PASS  %s", $realtime / 1e6, msg);
        else begin $display("[%8.2f s] FAIL  %s", $realtime / 1e6, msg); errors++; end
    endtask

    `define FSM dut.g_mission.u_mfsm
    `define MRX dut.g_mission.u_mrx

    // ═══════════════════════════════════════════════
    //   UART אל esp_rx
    // ═══════════════════════════════════════════════
    byte unsigned txq[$];
    initial begin
        #200;
        forever begin
            if (txq.size() == 0) #1;
            else begin
                byte unsigned b;
                b = txq.pop_front();
                esp_rx = 1'b0; #(BIT);
                for (int i = 0; i < 8; i++) begin esp_rx = b[i]; #(BIT); end
                esp_rx = 1'b1; #(BIT);
            end
        end
    end
    task automatic line(string s);
        for (int i = 0; i < s.len(); i++) txq.push_back(s[i]);
        txq.push_back(8'h0D);
        txq.push_back(8'h0A);
    endtask

    // ═══════════════════════════════════════════════
    //   fc_tx → ערוצי RC (חבילות תקינות בלבד)
    // ═══════════════════════════════════════════════
    int rc[8] = '{1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000};
    int frames = 0, bad_frames = 0;
    int max_dthr = 0, max_dpch = 0;          // קפיצה מרבית בין חבילות בזמן משימה
    int max_thr_to = 0;                      // מצערת מרבית בזמן ההמראה
    int min_thr_air = 9999;                  // מצערת מזערית שיצאה בזמן שהרחפן באוויר (z > 30)
    byte unsigned fr[$];
    initial begin
        forever begin
            byte unsigned b;
            @(negedge fc_tx);
            #(BIT * 1.5);
            for (int i = 0; i < 8; i++) begin b[i] = fc_tx; #(BIT); end
            fr.push_back(b);
            while (fr.size() > 0 && fr[0] != 8'h24) void'(fr.pop_front());
            if (fr.size() >= 22) begin
                byte unsigned cs = 0;
                for (int i = 3; i <= 20; i++) cs ^= fr[i];
                if (fr[1] == 8'h4D && fr[2] == 8'h3C && fr[3] == 16 && fr[4] == 200 && fr[21] == cs) begin
                    int old_thr = rc[2], old_pch = rc[1];
                    for (int c = 0; c < 8; c++) rc[c] = int'(fr[5 + 2*c]) | (int'(fr[6 + 2*c]) << 8);
                    if (`FSM.m_active && frames > 0) begin
                        int a = rc[2] - old_thr, p = rc[1] - old_pch;
                        if (a < 0) a = -a;
                        if (p < 0) p = -p;
                        if (a > max_dthr) max_dthr = a;
                        if (p > max_dpch) max_dpch = p;
                    end
                    if ((`FSM.st == 5'd21 || `FSM.st == 5'd22 || `FSM.st == 5'd23) && rc[2] > max_thr_to)
                        max_thr_to = rc[2];
                    if (z > 30.0 && inav_armed && rc[2] < min_thr_air) min_thr_air = rc[2];
                    if (inav_armed && z < 100.0 && rc[6] >= 1400 && rc[6] < 1650) ch7_pos_low++;
                    if (inav_armed && z < 80.0 && rc[6] >= 1400 && rc[6] < 1650) ch7_pos_lt80++;
                    if (`FSM.st >= 5'd9 && `FSM.st <= 5'd11 && rc[1] > max_pch_cross) max_pch_cross = rc[1];
                    if (rc[6] < 1400 || rc[6] >= 1650) ch7_not_pos++;
                    frames++;
                end else bad_frames++;
                fr.delete();
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   esp_tx → שורות MST
    //   MST:SS:RR:CCCC:DDD:TTT:WW:G:VVV:AA:PPP:LLL:BB:KKKK:F:NN
    // ═══════════════════════════════════════════════
    int mst_st = -1, mst_rs = -1, mst_chk = -1, mst_det = -1, mst_stp = -1, mst_wp = -1, mst_stage = -1;
    int mst_dv = -1, mst_ta = -1, mst_tsp = -1, mst_trv = -1, mst_rbk = -1, mst_sck = -1, mst_flg = -1, mst_ntr = -1;
    int mst_lines = 0, mst_flg_or = 0;
    function automatic int hexv(string s);
        int v = 0;
        for (int i = 0; i < s.len(); i++) begin
            byte c = s[i];
            v = v * 16 + ((c >= "A") ? (c - "A" + 10) : (c - "0"));
        end
        return v;
    endfunction
    initial begin
        string cur = "";
        forever begin
            byte unsigned b;
            @(negedge esp_tx);
            #(BIT * 1.5);
            for (int i = 0; i < 8; i++) begin b[i] = esp_tx; #(BIT); end
            if (b == 8'h0A) begin
                if (cur.len() == 55 && cur.substr(0, 3) == "MST:") begin
                    mst_st    = hexv(cur.substr(4, 5));
                    mst_rs    = hexv(cur.substr(7, 8));
                    mst_chk   = hexv(cur.substr(10, 13));
                    mst_det   = hexv(cur.substr(15, 17));
                    mst_stp   = hexv(cur.substr(19, 21));
                    mst_wp    = hexv(cur.substr(23, 24));
                    mst_stage = hexv(cur.substr(26, 26));
                    mst_dv    = hexv(cur.substr(28, 30));
                    mst_ta    = hexv(cur.substr(32, 33));
                    mst_tsp   = hexv(cur.substr(35, 37));
                    mst_trv   = hexv(cur.substr(39, 41));
                    mst_rbk   = hexv(cur.substr(43, 44));
                    mst_sck   = hexv(cur.substr(46, 49));
                    mst_flg   = hexv(cur.substr(51, 51));
                    mst_ntr   = hexv(cur.substr(53, 54));
                    mst_flg_or |= mst_flg;
                    mst_lines++;
                end
                cur = "";
            end else cur = {cur, $sformatf("%c", b)};
        end
    end
    function automatic bit mst_active();
        return mst_st > 0 && mst_st != 17 && mst_st != 18;
    endfunction

    // ═══════════════════════════════════════════════
    //   העולם
    // ═══════════════════════════════════════════════
    real x = 0.0, y = 0.0, z = 0.0, vx = 0.0, vy = 0.0, vz = 0.0;
    real heading_deg = 0.0;                      // כיוון הגוף מול קו הטיסה (הזרקה: 90 הצידה, 180 אחורה)
    localparam real HALF = 30.0;                 // מהמרכז עד קצה הפרופלור
    // מכשולים: קופסאות [x0, x1] בגובה h. far = קיר רחוק אופציונלי
    real bx0[3] = '{300.0, -1.0, -1.0};
    real bx1[3] = '{350.0, -1.0, -1.0};
    real bh[3]  = '{150.0, 0.0, 0.0};
    real wp_x[2]   = '{700.0, 900.0};
    real wp_alt[2] = '{100.0, 100.0};
    int  wp_n = 2, wp_idx = 1;
    bit  wp_fin = 1'b0;

    // ═══ INAV ═══
    int  inav_mode = 0;                          // 0 ANGLE 1 ALTHOLD 2 POSHOLD 3 RTH 4 WP
    int  mode_pend = -1;  real mode_t = 0.0;
    bit  inav_armed = 1'b0;
    real neutral_us = 1650.0;
    int  thr_mid_pct = 65;
    int  cfg_atm = 0, cfg_ucm = 1, cfg_rst = 1, cfg_cas = 50;
    real z_noise_amp = 0.0;
    real t_now = 0.0;

    // ── מדדים ──
    real max_vx_retalt = 0.0, max_vz_cross = 0.0, min_clear_top = 1000.0, max_vx_to = 0.0;
    real z_det = 0.0, max_dz_stop = 0.0, z_clr_evt = 0.0, z_clear_hold = 0.0, x_rear_stop = 0.0;
    real x_rear_edge = -1.0;
    bit  collided = 1'b0, ever_landed = 1'b0;
    bit  clr_inf_at_cross = 1'b0, clr_inf_at_hold = 1'b0;
    real z_max_act = -1000.0;
    int  ch7_pos_low = 0, ch7_not_pos = 0, ch7_pos_lt80 = 0, max_pch_cross = 1500;
    int  prev_st = 0;

    // ── הזרקות ──
    bit inj_wp_reject   = 1'b0;
    bit inj_wp_jump     = 1'b0;
    bit inj_pose_freeze = 1'b0;
    bit inj_dwn_nosig   = 1'b0;
    bit inj_dwn_oor     = 1'b0;
    bit inj_front_hit   = 1'b0;
    bit inj_no_climb    = 1'b0;
    int inj_batt        = 0;
    int inj_sats        = 14;
    int inj_eph         = 80;                     // cm
    int inj_mis         = 1;
    int inj_mal         = -1;                     // -1 = wp_alt[0]
    bit inj_fwa_off     = 1'b0;
    int flm = 1, dlm = 0;                         // FRONT LONG · DOWN SHORT
    int front_nr_cls    = 2;                      // אין מטרה בטווח: 2 OOR / 6 NO_SIGNAL
    bit angle_vz        = 1'b0;                   // ANGLE: מצערת ⇒ קצב אנכי (המראה ידנית)
    bit dwn_nosig_from8 = 1'b0;                   // DOWN = NO_SIGNAL מ-GROUND_REF והלאה
    bit dwn_freeze      = 1'b0;                   // DOWN מפסיק לעדכן (DSEQ קפוא)
    // רוח: כש-INAV ב-POSHOLD/CRUISE "מזיז" (מקל מחוץ ל-deadband) הוא לא מחזיק מיקום -
    // רק מהירות. הסחיפה הזו נמדדה בטיסה: 4-8 cm/s (21.10). ניצב לכיוון הגוף / אחורה.
    real wind_lat       = 0.0;                    // cm/s, ניצב (+Y כשהגוף לאורך X)
    real wind_back      = 0.0;                    // cm/s, נגד כיוון הגוף

    // ═══ האפליקציה ═══
    int app_mode  = 0;                            // 0 ANGLE .. 4 WP
    int app_pch   = 0, app_rll = 0, app_yaw = 0;
    int app_thr_h = 0;                            // חצאי אחוז
    int app_rpl   = 1;
    bit app_track = 1'b1;                         // שולחת את הנייטרל (NN) בזמן משימה באוויר

    function automatic int ch7_mode(int us);
        if (us < 1150) return 0;
        if (us < 1400) return 1;
        if (us < 1650) return 2;
        if (us < 1900) return 3;
        return 4;
    endfunction

    // ═══ INAV 9.1, POSHOLD + nav_user_control_mode = CRUISE: מקל Pitch ⇒ מהירות יעד ═══
    //   (navigation.c / navigation_multicopter.c, ההגדרות שנבדקו ב-CLI 21.10)
    //   rcCommand = rcLookup(deadband(rc − 1500, deadband=2), rc_expo=55):
    //     x = הסטה/100,  rcCommand = (2500 + expo·(x² − 25))·x/25   (אומת מול Blackbox ב-ANGLE)
    //   adj = deadband(rcCommand, pos_hold_deadband=10)  ⇒  v = adj · nav_manual_speed(50) / 490
    //   ⇒ 40µs = 0.8cm/s · 250µs = 14 · 400µs = 32 · 500µs = 51
    function automatic real inav_cruise_v(int us_off);
        real d = (us_off < 0) ? -us_off : us_off;
        real x, rcc, adj, v;
        if (d <= 2.0) return 0.0;
        d = (d - 2.0) * 500.0 / 498.0;
        if (d > 500.0) d = 500.0;
        x   = d / 100.0;
        rcc = (2500.0 + 55.0 * (x * x - 25.0)) * x / 25.0;
        if (rcc <= 10.0) return 0.0;
        adj = (rcc - 10.0) * 500.0 / 490.0;
        v   = adj * 50.0 / 490.0;
        return (us_off < 0) ? -v : v;
    endfunction

    function automatic real surf_at(real xc);
        real s = 0.0;
        for (int i = 0; i < 3; i++)
            if (bx0[i] >= 0.0 && xc >= bx0[i] && xc <= bx1[i] && bh[i] > s) s = bh[i];
        return s;
    endfunction

    real z_meas = 0.0;
    initial begin
        forever begin
            real dt = 0.01, ax = 0.0, vzdes = 0.0;
            int req;
            #10000;
            t_now = $realtime / 1e6;

            // ── חימוש לפי CH5: על הקרקע רק ב-ANGLE ובמצערת נמוכה ──
            if (rc[4] > 1700 && !inav_armed &&
                (z > 1.0 || (ch7_mode(rc[6]) == 0 && rc[2] < 1100))) inav_armed = 1'b1;
            if (rc[4] < 1300) inav_armed = 1'b0;

            // ── מצב לפי CH7, ACK אחרי 50ms ──
            req = ch7_mode(rc[6]);
            if (!inav_armed) req = 0;
            if (req == 4 && inj_wp_reject) req = inav_mode;
            if (req != inav_mode) begin
                if (mode_pend != req) begin mode_pend = req; mode_t = t_now; end
                else if (t_now - mode_t >= 0.05) begin
                    if (req != 0 && inav_mode == 0) begin
                        // כניסה למצב שמחזיק גובה: על הקרקע / MID_STICK ⇒ thr_mid, אחרת STICK
                        if (z < 1.0 || cfg_atm == 1) neutral_us = 1000.0 + 10.0 * thr_mid_pct;
                        else                         neutral_us = real'(rc[2]);
                    end
                    if (req == 4 && inj_wp_jump) begin wp_idx = wp_idx + 1; inj_wp_jump = 1'b0; end
                    inav_mode = req;
                    mode_pend = -1;
                end
            end else mode_pend = -1;

            // ── אופקי (דו-ממדי) ──
            //   POSHOLD במצב CRUISE: Pitch = מהירות בכיוון הגוף (inav_cruise_v), ובזמן שהמקל
            //   מחוץ ל-deadband אין החזקת מיקום ⇒ הרוח (wind_lat / wind_back) מזיזה.
            //   WP: לאורך X בלבד. מקלות במרכז / מצבים אחרים: בלימה בכל ציר.
            begin
                real vdx = 0.0, vdy = 0.0, ay = 0.0;
                bit  brake = 1'b1;
                if (inav_mode == 2) begin
                    real sp = inav_cruise_v(rc[1] - 1500);
                    if (sp != 0.0) begin                 // rcCommand מעבר ל-pos_hold_deadband
                        real c = $cos(heading_deg * 3.14159265 / 180.0), sn = $sin(heading_deg * 3.14159265 / 180.0);
                        vdx = (sp - wind_back) * c - wind_lat * sn;
                        vdy = (sp - wind_back) * sn + wind_lat * c;
                        brake = 1'b0;
                    end
                end else if (inav_mode == 4 && !wp_fin) begin
                    real d = wp_x[wp_idx - 1] - x;
                    vdx = (d > 50.0) ? 50.0 : (d < -50.0 ? -50.0 : d);
                    vdy = 0.0;
                    brake = 1'b0;
                    if (d < 15.0 && d > -15.0) begin
                        if (wp_idx < wp_n) wp_idx++;
                        else wp_fin = 1'b1;
                    end
                end
                if (brake) begin
                    ax = (vx > 0.0) ? -60.0 : (vx < 0.0 ? 60.0 : 0.0);
                    ay = (vy > 0.0) ? -60.0 : (vy < 0.0 ? 60.0 : 0.0);
                    if (vx * (vx + ax * dt) <= 0.0) begin ax = 0.0; vx = 0.0; end
                    if (vy * (vy + ay * dt) <= 0.0) begin ay = 0.0; vy = 0.0; end
                end else begin
                    ax = (vdx - vx) / 0.5;  ay = (vdy - vy) / 0.5;
                    if (ax > 60.0) ax = 60.0; if (ax < -60.0) ax = -60.0;
                    if (ay > 60.0) ay = 60.0; if (ay < -60.0) ay = -60.0;
                end
                if (z <= 0.0 && inav_mode != 4) begin ax = 0.0; ay = 0.0; vx = 0.0; vy = 0.0; end
                vx = vx + ax * dt;  vy = vy + ay * dt;
                x  = x + vx * dt;   y  = y + vy * dt;
            end

            // ── אנכי ──
            if (!inav_armed) vzdes = 0.0;
            else if (inav_mode == 4) begin
                vzdes = wp_alt[wp_idx - 1] - z;
                if (vzdes > 50.0) vzdes = 50.0; if (vzdes < -50.0) vzdes = -50.0;
            end
            else if (inav_mode != 0) begin
                real adj = real'(rc[2]) - neutral_us;
                if (adj > 50.0)       vzdes = (adj - 50.0) / (2000.0 - neutral_us - 50.0) * 100.0;
                else if (adj < -50.0) vzdes = (adj + 50.0) / (neutral_us - 1000.0 - 50.0) * 100.0;
                else                  vzdes = 0.0;
            end
            else if (angle_vz) begin
                vzdes = (real'(rc[2]) - 1650.0) / 3.0;
                if (vzdes > 100.0) vzdes = 100.0; if (vzdes < -100.0) vzdes = -100.0;
            end
            else vzdes = 0.0;
            if (inj_no_climb && vzdes > 0.0) vzdes = 0.0;
            if (vz < vzdes) vz = (vz + 200.0 * dt > vzdes) ? vzdes : vz + 200.0 * dt;
            else            vz = (vz - 200.0 * dt < vzdes) ? vzdes : vz - 200.0 * dt;
            z = z + vz * dt;
            if (z <= 0.0) begin
                z = 0.0;
                if (vz < 0.0) vz = 0.0;
            end
            if (z < 0.5 && prev_st != 0 && `FSM.st != 5'd21 && `FSM.st != 5'd22 && `FSM.m_active) ever_landed = 1'b1;
            z_meas = z + z_noise_amp * $sin(2.0 * 3.14159 * 0.7 * t_now);

            // ── מדדים ──
            begin
                int s = `FSM.st;
                real avx = (vx > 0.0) ? vx : -vx, avz = (vz > 0.0) ? vz : -vz;
                if (s != prev_st) begin
                    if (s == 4)  begin z_det = z; max_dz_stop = 0.0; end
                    if (s == 6)  z_clr_evt = z;
                    if (s == 7)  z_clear_hold = z;
                    if (s == 10 && x_rear_edge < 0.0) x_rear_edge = x;
                    if (s == 12) x_rear_stop = x;
                    if (s == 9)  clr_inf_at_cross = `FSM.clr_inf;
                    if (s == 7)  clr_inf_at_hold  = `FSM.clr_inf;
                    if (s == 8 && dwn_nosig_from8) inj_dwn_nosig = 1'b1;
                    prev_st = s;
                end
                if (s == 4 && (z - z_det > max_dz_stop || z_det - z > max_dz_stop))
                    max_dz_stop = (z > z_det) ? z - z_det : z_det - z;
                if (`FSM.m_active && z > z_max_act) z_max_act = z;
                if (s == 13 && avx > max_vx_retalt) max_vx_retalt = avx;
                if (s >= 9 && s <= 11 && avz > max_vz_cross) max_vz_cross = avz;
                if ((s >= 21 && s <= 23) && avx > max_vx_to) max_vx_to = avx;
                for (int i = 0; i < 3; i++) if (bx0[i] >= 0.0) begin
                    if (x + HALF >= bx0[i] && x - HALF <= bx1[i]) begin
                        if (z - bh[i] < min_clear_top) min_clear_top = z - bh[i];
                        if (z < bh[i]) collided = 1'b1;
                    end
                end
            end
        end
    end

    // ═══════════════════════════════════════════════
    //   מודל ה-ESP
    // ═══════════════════════════════════════════════
    int fseq = 0, dseq = 0;
    int f_dist = 0, f_cls = 2, d_dist = 0, d_cls = 2;
    int npq = 0, nnq = 0, nfq = 0, gsq = 0, slq = 0;
    bit esp_on = 1'b0, stat_on = 1'b0, set_on = 1'b0;
    int dwn_log[$];
    int set_v[8] = '{100, 150, 70, 70, 400, 50, 5, 0};
    int set_ck_err = 0;
    int set_n = 8;

    // FRONT: היעד הקרוב ביותר בתוך הקרן ובתוך הטווח
    task automatic front_read(output int o_d, output int o_c);
        real zf = z + 8.0, xt = x + HALF, best = 1.0e9, rng = flm ? 400.0 : 130.0;
        real dg = zf / 0.24;
        if (dg < best) best = dg;                                    // הקרקע
        for (int i = 0; i < 3; i++) if (bx0[i] >= 0.0 && bx1[i] > xt) begin
            real dface = bx0[i] - xt, dtop;
            if (zf < bh[i] && dface <= 0.0) dtop = 0.0;
            else dtop = (zf - bh[i]) / 0.24;
            if (dface > 0.0 && zf - 0.24 * dface < bh[i]) begin if (dface < best) best = dface; end
            else if (dtop >= dface && dtop <= bx1[i] - xt && dtop < best) best = dtop;
        end
        if (inj_front_hit) begin o_d = 60; o_c = 1; end
        else if (best > rng) begin o_d = 0; o_c = front_nr_cls; end
        else if (best < 4.0) begin o_d = 0; o_c = 5; end
        else begin o_d = int'(best); o_c = 1; end
    endtask

    // דגימות לידאר כל 55ms
    initial begin
        forever begin
            real dd, rng;
            #55000;
            if (!esp_on) continue;
            front_read(f_dist, f_cls);
            fseq = (fseq + 1) % 256;
            dd  = z - surf_at(x);
            rng = dlm ? 400.0 : 130.0;
            if (inj_dwn_nosig)    begin d_dist = 0; d_cls = 6; end
            else if (inj_dwn_oor) begin d_dist = 0; d_cls = 2; end
            else if (dd < 4.0)    begin d_dist = 0; d_cls = 5; end
            else if (dd <= rng)   begin d_dist = int'(dd); d_cls = 1; end
            else                  begin d_dist = 0; d_cls = 2; end
            if (!dwn_freeze) dseq = (dseq + 1) % 256;
            if (`FSM.st >= 5'd8 && `FSM.st <= 5'd11 && d_cls == 1) dwn_log.push_back(d_dist);
        end
    end

    function automatic int wrap16(real v);
        int i = int'(v);
        i = i & 32'hFFFF;
        if (i >= 32768) i = i - 65536;
        return i;
    endfunction
    function automatic int fok(int c);           // קוד FOK/DOK הישן של cmd_parser
        case (c) 1: return 1; 2: return 2; 5: return 4; 0: return 0; default: return 3; endcase
    endfunction
    function automatic int phs();
        return !inav_armed ? 0 : (z > 12.0) ? 3 : 1;
    endfunction

    real XOFF = 32600.0;            // X חוצה את 32767 באמצע המשימה

    // שליטה + חיישנים, 20Hz
    initial begin
        forever begin
            string mnames[5] = '{"ANGLE", "ALTHOLD", "POSHOLD", "RTH", "WP"};
            #50000;
            if (!esp_on) continue;
            // האפליקציה: בזמן משימה באוויר שולחת את נייטרל המצערת שה-FPGA מדווח
            if (app_track && mst_active() && phs() == 3 && mst_ntr >= 0) app_thr_h = mst_ntr;
            line($sformatf("THR:%0d THH:%0d YAW:%0d PCH:%0d RLL:%0d MODE:%s DSC:0 PHA:%0d TH2:%0d RPL:%0d",
                           app_thr_h / 2, app_thr_h, app_yaw, app_pch, app_rll, mnames[app_mode], phs(),
                           app_thr_h, app_rpl));
            line($sformatf("FWD:%0d FOK:%0d DWN:%0d DOK:%0d FSEQ:%0d DSEQ:%0d PHS:%0d FWA:%0d MGH:1 FRC:%0d DRC:%0d FLM:%0d DLM:%0d",
                           f_dist, fok(f_cls), d_dist, fok(d_cls), fseq, dseq, phs(),
                           (z >= 30.0 && !inj_fwa_off) ? 1 : 0, f_cls, d_cls, flm, dlm));
        end
    end

    // שורת INAV מהירה, 10Hz
    initial begin
        forever begin
            int nmd, nst, nfm;
            #100000;
            if (!esp_on) continue;
            if (!inj_pose_freeze) npq = (npq + 1) % 256;
            nnq = (nnq + 1) % 256;  nfq = (nfq + 1) % 256;  gsq = (gsq + 1) % 256;
            nmd = !inav_armed ? 0 : (inav_mode == 2) ? 1 : (inav_mode == 4) ? 3 : (inav_mode == 3) ? 2 : 0;
            nst = !inav_armed ? 0 : (inav_mode == 2) ? 3 : (inav_mode == 4) ? (wp_fin ? 6 : 5) :
                  (inav_mode == 3) ? 2 : 0;
            nfm = (inav_armed ? 1 : 0) | ((inav_mode == 0) ? 2 : 0) |
                  ((inav_mode != 0 && inav_armed) ? 16 : 0) | ((inav_mode == 2) ? 32 : 0) |
                  ((inav_mode == 3) ? 64 : 0) | ((inav_mode == 4) ? 128 : 0);
            line($sformatf("NPX:%0d NPY:%0d NPZ:%0d NVX:%0d NVY:%0d NVZ:%0d NPQ:%0d NMD:%0d NST:%0d NWP:%0d NER:%0d NNQ:%0d NFM:%0d NAR:%0d NFQ:%0d GFX:2 GSA:%0d GEP:%0d GSQ:%0d",
                           wrap16(x + XOFF), wrap16(y - 31000.0), int'(z_meas), int'(vx), int'(vy), int'(vz), npq,
                           nmd, nst, wp_idx, (inav_mode == 4 && wp_fin) ? 4 : 0,
                           nnq, nfm, inav_armed ? 1 : 0, nfq, inj_sats, inj_eph, gsq));
        end
    end

    // שורה איטית, 2Hz
    initial begin
        forever begin
            #500000;
            if (!esp_on) continue;
            slq = (slq + 1) % 256;
            line($sformatf("MIS:%0d CAT:%0d CUC:%0d CRS:%0d CAS:%0d BTS:%0d SLQ:%0d CTM:%0d MAL:%0d",
                           inj_mis, cfg_atm, cfg_ucm, cfg_rst, cfg_cas, inj_batt, slq, thr_mid_pct,
                           (inj_mal >= 0) ? inj_mal : int'(wp_alt[0])));
        end
    end

    // שורת ההגדרות, 1Hz (ה-ESP שולח תמיד; ה-FPGA מחיל רק כשהוא מנוטרל)
    function automatic int set_sum();
        int s = 0;
        for (int i = 0; i < 8; i++) s += (8 - i) * ((set_v[i] > 1023) ? 1023 : set_v[i]);
        return s;
    endfunction
    task automatic send_settings();
        string s = "MSE:";
        for (int i = 0; i < set_n; i++) s = {s, $sformatf("%0d,", set_v[i])};
        s = {s, $sformatf("%0d", set_sum() + set_ck_err)};
        line(s);
    endtask
    initial begin
        forever begin
            #1000000;
            if (esp_on && set_on) send_settings();
        end
    end

    // MSTAT, 5Hz
    initial begin
        forever begin
            #200000;
            if (esp_on && stat_on) line("CMD:MSTAT");
        end
    end

    // ═══════════════════════════════════════════════
    //   רישום מצבי ה-FSM
    // ═══════════════════════════════════════════════
    int resume_saved = -1, resume_nav = -1;
    int trace[$];
    int last_st = -1;
    always @(posedge clk) begin
        int s = `FSM.st;
        if (s != last_st) begin
            trace.push_back(s);
            $display("[%8.2f s] st %0d -> %0d  reason=%02h  x=%0.0f z=%0.0f vx=%0.0f vz=%0.0f  CH7=%0d CH3=%0d CH2=%0d",
                     $realtime / 1e6, last_st, s, `FSM.reason, x, z, vx, vz, rc[6], rc[2], rc[1]);
            last_st = s;
        end
    end

    function automatic string tstr();
        string r = "";
        foreach (trace[i]) r = {r, $sformatf("%0d ", trace[i])};
        return r;
    endfunction

    task automatic wait_st(int s, real tmo_s);
        real t0 = $realtime;
        while (`FSM.st != s && ($realtime - t0) < tmo_s * 1e6) #1000;
    endtask
    task automatic wait_s(real s);
        #(s * 1e6);
    endtask
    function automatic bit chk_bit(int b);
        return `FSM.checks[b];
    endfunction

    // ═══ אתחול: על הקרקע, משימה מוכנה (WP נבחר), מחומש - בלי START ═══
    task automatic setup_ground();
        z = 0.0; x = 0.0; vx = 0.0; vz = 0.0;
        set_v[7] = 0;                            // F1021E: אין המראה אוטומטית
        esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
        app_mode = 4; app_thr_h = 0;
        wait_s(1.6);
        line("CMD:CHECK"); line("CMD:ARM");
        wait_s(1.0);
        check(inav_armed && inav_mode == 0 && z == 0.0,
              $sformatf("setup: armed on the ground in ANGLE (mode=%0d CH7=%0d), motors at idle", inav_mode, rc[6]));
    endtask

    // ═══ אתחול: מחומש, באוויר, POSHOLD ═══
    task automatic setup_airborne(real z0);
        z = z0; x = 0.0; vx = 0.0; vz = 0.0;
        esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
        app_mode = 0; app_thr_h = 131;
        wait_s(1.6);
        line("CMD:CHECK"); line("CMD:ARM");
        wait_s(0.5);
        app_mode = 2;
        wait_s(2.0);
        check(inav_armed && inav_mode == 2, $sformatf("setup: INAV armed and in POSHOLD (mode=%0d), z=%0.0f", inav_mode, z));
    endtask

    task automatic takeover_end();
        // המשימה הושלמה: הטייס לוקח שליטה (STOP)
        line("CMD:MSTOP");
        wait_s(0.5);
    endtask

    // ═══════════════════════════════════════════════
    //   תרחישים
    // ═══════════════════════════════════════════════
    initial begin
        if (!$value$plusargs("SCEN=%d", scen)) scen = 1;
        #100000;
        $display("=== SCENARIO %0d ===", scen);

        case (scen)

        // ── 1: משימה מלאה באוויר, שלב 5. DOWN SHORT לא רואה קרקע מגובה המעבר ──
        1: begin
            setup_airborne(100.0);
            check(`FSM.checks == 16'hFFFF, $sformatf("1: all 16 SELF_CHECK bits pass (%04h)", `FSM.checks));
            line("CMD:MSTART");
            wait_st(5'd15, 120.0);
            resume_saved = `FSM.saved_wp;
            wait_st(5'd3, 10.0);
            resume_nav = wp_idx;
            wait_st(5'd24, 120.0);
            wait_s(2.0);
            check(`FSM.st == 5'd24 && rc[6] == 1500 && inav_mode == 2,
                  $sformatf("1: at B: DONE_HOLD, CH7=POSHOLD, INAV POSHOLD (st=%0d)", `FSM.st));
            check(inav_armed && !ever_landed && z > 80.0, $sformatf("1: no landing, no RTH - hovering at %0.0f cm", z));
            takeover_end();
            check(`FSM.st == 5'd17 && `FSM.reason == 8'h00,
                  $sformatf("1: pilot takes control -> COMPLETE (st=%0d reason=%02h)", `FSM.st, `FSM.reason));
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 25 17 ",
                  $sformatf("1: exact state sequence: %s", tstr()));
            check(!collided, $sformatf("1: no collision; min clearance over the top %0.0f cm", min_clear_top));
            check(min_clear_top >= 60.0, "1: bottom stayed >= ~70cm (vertical clearance) over the top");
            check(z_clear_hold - z_clr_evt >= 65.0,
                  $sformatf("1: +%0.0f cm after FRONT clear (setting 70)", z_clear_hold - z_clr_evt));
            check(x_rear_stop - HALF >= bx1[0] + 65.0,
                  $sformatf("1: rear prop %0.0f cm past the rear edge (setting 70)", x_rear_stop - HALF - bx1[0]));
            check(max_dz_stop < 3.0, $sformatf("1: no vertical motion before the stop was confirmed (%0.1f cm)", max_dz_stop));
            check(max_vx_retalt < 5.0, $sformatf("1: return to mission altitude is vertical (max |vx| %0.1f)", max_vx_retalt));
            check(max_vz_cross < 8.0, $sformatf("1: crossing is horizontal (max |vz| %0.1f)", max_vz_cross));
            check(x > 870.0 && x < 930.0, $sformatf("1: stopped at B (x=%0.0f)", x));
            check(`FSM.det_cm <= 100 && `FSM.stop_cm != 10'h3FF,
                  $sformatf("1: detection %0d cm, stop %0d cm", `FSM.det_cm, `FSM.stop_cm));
            check(resume_saved == 1 && resume_nav == 1,
                  $sformatf("1: saved WP %0d, RESUME went back to WP%0d", resume_saved, resume_nav));
            check(mst_st == 17 && mst_lines > 100, $sformatf("1: MST path: st=%0d after %0d lines", mst_st, mst_lines));
            check(((mst_flg_or >> 1) & 1) == 0, "1: DOWN never saw the ground (SHORT, 2.4m) - crossing used the known depth");
            check(`FSM.m_active == 1'b0 && rc[6] == 1500, "1: after COMPLETE: pass-through (CH7 = app POSHOLD)");
            check(max_dthr <= 25 && max_dpch <= 12,
                  $sformatf("1: smooth commands: max step per frame thr %0d us, pitch %0d us", max_dthr, max_dpch));
            begin
                int nt = 0;
                foreach (dwn_log[i]) if (dwn_log[i] >= 60 && dwn_log[i] <= 130) nt++;
                check(nt > 5, $sformatf("1: DOWN saw the TOP as VALID (%0d samples)", nt));
            end
        end

        // ── 2: שלב 2 - עצירה בלבד, ומדדי הבלימה ──
        2: begin
            set_v[6] = 2;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 60.0);
            check(tstr() == "0 1 2 3 4 25 18 ", $sformatf("2: WP -> OBST_STOP -> SUSPENDED: %s", tstr()));
            check(`FSM.reason == 8'h14, $sformatf("2: reason STAGE_END (%02h)", `FSM.reason));
            check(!collided && vx < 2.0 && vx > -2.0, $sformatf("2: stopped before the obstacle (x=%0.0f)", x));
            check(max_dz_stop < 3.0, $sformatf("2: no climb during the stop (%0.1f cm)", max_dz_stop));
            wait_s(1.0);
            check(mst_rs == 8'h14 && mst_det > 0 && mst_det <= 100 && mst_stp > 30 && mst_stp < 100,
                  $sformatf("2: MST: detection %0d cm, stop %0d cm", mst_det, mst_stp));
            check(mst_dv >= 40 && mst_dv <= 60 && mst_ta > 0 && mst_ta < 30 && mst_tsp > mst_ta &&
                  mst_trv >= 15 && mst_trv <= 45 && mst_rbk <= 5,
                  $sformatf("2: brake metrics: v=%0d cm/s ack=%0d0 ms stop=%0d0 ms travel=%0d cm rebound=%0d cm",
                            mst_dv, mst_ta, mst_tsp, mst_trv, mst_rbk));
            check(rc[6] == 1500 && rc[1] == 1500, "2: SUSPENDED = pass-through: CH7 POSHOLD, sticks centered");
        end

        // ── 3: שלב 3 - עד CLEAR_HOLD ──
        3: begin
            set_v[6] = 3;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 60.0);
            check(tstr() == "0 1 2 3 4 5 6 7 25 18 " && `FSM.reason == 8'h14,
                  $sformatf("3: stops after CLEAR_HOLD with STAGE_END (%s rs=%02h)", tstr(), `FSM.reason));
            check(z_clear_hold - z_clr_evt >= 65.0 && !collided,
                  $sformatf("3: climbed until FRONT clear (%0.0f) then +%0.0f cm", z_clr_evt, z_clear_hold - z_clr_evt));
            wait_s(1.0);
            check(vz < 3.0 && vz > -3.0, "3: hovering after SUSPENDED");
        end

        // ── 4: שלב 4 - עד REAR_STOP ──
        4: begin
            set_v[6] = 4;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 80.0);
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 25 18 " && `FSM.reason == 8'h14,
                  $sformatf("4: stops after REAR_STOP (%s rs=%02h)", tstr(), `FSM.reason));
            check(x - HALF >= bx1[0] + 65.0 && !collided,
                  $sformatf("4: whole drone past the rear edge + forward clearance (%0.0f cm)", x - HALF - bx1[0]));
        end

        // ── 5: דחיות START באוויר ──
        5: begin
            setup_airborne(100.0);
            inj_mis = 0; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && `FSM.reason == 8'h01 && !chk_bit(10),
                  "5a: mission not verified (MIS:0) -> START rejected, bit10");
            wait_s(0.5);
            check(mst_rs == 1 && ((mst_chk >> 10) & 1) == 0, $sformatf("5a: reason over MST (rs=%0d chk=%04h)", mst_rs, mst_chk));
            inj_mis = 1; inj_sats = 5; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(5), "5b: GPS 5 sats -> rejected (bit5)");
            inj_sats = 14; app_mode = 1; wait_s(1.5);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(3), "5c: not in POSHOLD -> rejected (bit3)");
            app_mode = 2; inj_batt = 1; wait_s(1.5);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(13), "5d: battery WARN -> rejected (bit13)");
            inj_batt = 0; inj_fwa_off = 1'b1; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(9), "5e: FRONT not active -> rejected (bit9)");
            inj_fwa_off = 1'b0; app_pch = 30; wait_s(0.5);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(14), "5f: pilot holding the pitch stick -> rejected (bit14)");
            app_pch = 0; inj_pose_freeze = 1'b1; wait_s(1.0);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(7), "5g: pose stale -> rejected (bit7)");
            inj_pose_freeze = 1'b0; cfg_ucm = 0; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(11), "5h: nav_user_control_mode=ATTI (not CRUISE) -> rejected (bit11)");
            cfg_ucm = 1; cfg_cas = 150; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(11), "5i: effective WP speed 150 cm/s > 100 -> rejected (bit11)");
            cfg_cas = 50; cfg_rst = 0; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(11), "5j: nav_wp_mission_restart != RESUME -> rejected (bit11)");
            cfg_rst = 1; inj_mal = 0; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(10), "5k: WP1 altitude unknown -> rejected (bit10)");
            inj_mal = 350; wait_s(1.2);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(10), "5l: WP1 350 + clearance 70 > ceiling 400 -> rejected (bit10)");
            inj_mal = -1; wait_s(1.5);
            check(trace.size() == 1, $sformatf("5: the FSM never left IDLE during all rejections (%s)", tstr()));
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st != 0 && `FSM.m_active, "5m: everything valid -> START accepted");
        end

        // ── 6: השתלטות טייס ──
        6: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            app_pch = 40;
            wait_s(0.5);
            check(`FSM.st == 18 && `FSM.reason == 8'h03,
                  $sformatf("6: pitch stick during WP -> SUSPENDED / TAKEOVER (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            wait_s(0.2);
            check(rc[6] == 1500 && rc[1] == 1700, $sformatf("6: control back to the pilot: CH7=%0d, CH2=%0d (no 60%% limit in POSHOLD)", rc[6], rc[1]));
        end

        // ── 7: נתון מיקום ישן ──
        7: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            inj_pose_freeze = 1'b1;
            wait_s(0.8);
            check(`FSM.st == 18 && `FSM.reason == 8'h06,
                  $sformatf("7: pose stops updating -> SUSPENDED / STALE_POSE (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            check(rc[6] == 1500, "7: CH7 back to POSHOLD");
        end

        // ── 8: INAV לא מאשר WP ──
        8: begin
            inj_wp_reject = 1'b1;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 20.0);
            check(tstr() == "0 1 2 25 18 " && `FSM.reason == 8'h05,
                  $sformatf("8: CH7=WP sent but INAV never confirms -> MODE_ACK (%s rs=%02h)", tstr(), `FSM.reason));
            check(vx < 1.0 && vx > -1.0 && x < 1.0, "8: the drone never moved");
        end

        // ── 9: RESUME לא חוזר ל-WP הצפוי ──
        9: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd15, 120.0);
            inj_wp_jump = 1'b1;
            wait_st(5'd18, 10.0);
            check(`FSM.reason == 8'h20,
                  $sformatf("9: INAV resumes at WP%0d instead of WP1 -> WP_MISMATCH (rs=%02h)", wp_idx, `FSM.reason));
            wait_s(0.3);
            check(rc[6] == 1500, "9: POSHOLD, not continuing blind");
        end

        // ── 10: מכשול גבוה מהתקרה ──
        10: begin
            bh[0] = 380.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 80.0);
            check(tstr() == "0 1 2 3 4 5 25 18 " && `FSM.reason == 8'h16,
                  $sformatf("10: FRONT never clear below the ceiling -> CEILING (%s rs=%02h)", tstr(), `FSM.reason));
            check(z <= 400.0 + 15.0, $sformatf("10: the climb stopped at the ceiling (z=%0.0f)", z));
            check(z_max_act <= 400.0 - 30.0 + 10.0,
                  $sformatf("10: climb stopped 30cm before the ceiling (max z %0.0f, ceiling 400)", z_max_act));
            wait_s(1.5);
            check(vz < 3.0 && vz > -3.0 && !collided, "10: hovering after SUSPENDED, no collision");
        end

        // ── 11: התקרה לא מאפשרת את המרווח - נבדק לפני הטיפוס ──
        11: begin
            set_v[4] = 200;
            setup_airborne(100.0);
            check(chk_bit(10) && chk_bit(11), "11: MAL 100 + 70 + margin 30 <= ceiling 200: START allowed");
            line("CMD:MSTART");
            wait_st(5'd18, 60.0);
            check(tstr() == "0 1 2 3 4 25 18 " && `FSM.reason == 8'h16,
                  $sformatf("11: stop at 100cm, 100+70+10 > 200-30 -> CEILING without climbing (%s rs=%02h)", tstr(), `FSM.reason));
            check(max_dz_stop < 3.0, "11: no climb at all");
        end

        // ── 12: DOWN LONG רואה את הקרקע (תרחיש A) ──
        12: begin
            dlm = 1;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 150.0);
            takeover_end();
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 25 17 ",
                  $sformatf("12: full sequence with DOWN LONG: %s", tstr()));
            check(((mst_flg_or >> 1) & 1) == 1, "12: DOWN saw the ground in GROUND_REF (flag bit1)");
            check(!collided && x_rear_stop - HALF >= bx1[0] + 65.0,
                  $sformatf("12: rear prop %0.0f cm past the rear edge", x_rear_stop - HALF - bx1[0]));
        end

        // ── 13: DOWN לא רואה כלום במעבר, וה"פנוי" היה אין-החזר ⇒ אין מעבר עיוור (TOP_UNCONFIRMED) ──
        13: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd8, 80.0);
            inj_dwn_oor = 1'b1;
            wait_st(5'd18, 40.0);
            // F1021A: ה"פנוי" כאן היה אין-החזר (OOR ב-LONG) ⇒ חובה לאשר ראש ב-DOWN. DOWN מחוץ
            // לטווח כל המעבר ⇒ אין מעבר עיוור לפי העומק: TOP_UNCONFIRMED ועצירה (קודם: מעבר לפי המרחק).
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 25 18 " && `FSM.reason == 8'h31,
                  $sformatf("13: inferred clear + DOWN OOR during CROSS -> no blind crossing, TOP_UNCONFIRMED (%s rs=%02h)", tstr(), `FSM.reason));
            check(!collided, $sformatf("13: no collision (prop %0.0f cm past the face)", x + HALF - bx0[0]));
        end

        // ── 14: DOWN NO_SIGNAL מעל הראש - אף פעם לא הוכחה ──
        14: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd10, 80.0);
            inj_dwn_nosig = 1'b1;
            wait_st(5'd18, 40.0);
            check(`FSM.reason == 8'h1D && !collided,
                  $sformatf("14: no rear edge from NO_SIGNAL -> OBST_TOO_LONG stop (rs=%02h, x=%0.0f)", `FSM.reason, x));
            check(trace.size() > 0 && trace[trace.size() - 3] == 10, $sformatf("14: never entered REAR_CLR (%s)", tstr()));
        end

        // ── 15: סוללה קריטית ──
        15: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            inj_batt = 2;
            wait_s(1.5);
            check(`FSM.st == 18 && `FSM.reason == 8'h22,
                  $sformatf("15: battery CRIT -> SUSPENDED / BATTERY (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            check(rc[6] == 1500 && inav_mode == 2, "15: POSHOLD - no automatic RTH / LAND");
        end

        // ── 16: MSTOP באמצע טיפוס ──
        16: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd5, 60.0);
            wait_s(0.5);
            line("CMD:MSTOP");
            wait_s(0.5);
            check(`FSM.st == 18 && `FSM.reason == 8'h02, "16: CMD:MSTOP -> SUSPENDED / PILOT_STOP (via 200ms throttle check)");
            wait_s(1.0);
            check(vz < 3.0 && vz > -3.0 && rc[2] == 1655, "16: climb stops - the pilot's neutral throttle is back");
        end

        // ── 17: מרחק זיהוי קצר מדי ⇒ עצירה קרובה ⇒ לא עוקפים ──
        17: begin
            set_v[0] = 40;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 60.0);
            check(`FSM.reason == 8'h15 && tstr() == "0 1 2 3 4 25 18 " && !collided,
                  $sformatf("17: stopped %0d cm from the obstacle -> STOP_TOO_CLOSE, no climb (%s)", `FSM.stop_cm, tstr()));
        end

        // ── 18: מכשול נמוך מתחת לקרן - אין זיהוי (מגבלת הגאומטריה) ──
        18: begin
            bh[0] = 70.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 80.0);
            check(`FSM.det_cm == 10'h3FF && tstr() == "0 1 2 3 16 24 ",
                  $sformatf("18: a 70cm top under a 100cm flight path is never a detection (%s)", tstr()));
            check(!collided, $sformatf("18: (no collision only because the bottom is %0.0f cm above the top)", min_clear_top));
        end

        // ── 19 (F1021E): START מהקרקע אינו אפשרי יותר - גם עם WP, ARM וכל השאר מוכן ──
        19: begin
            setup_ground();
            check(rc[2] == 1000 && rc[6] == 1000, $sformatf("19: WP prepared + armed on the ground: throttle %0d, CH7 %0d (ANGLE)", rc[2], rc[6]));
            check(!chk_bit(2), $sformatf("19: SELF_CHECK bit2 (AIRBORNE) false on the ground (%04h)", `FSM.checks));
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && `FSM.reason == 8'h01, $sformatf("19: START on the ground rejected (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            wait_s(3.0);
            check(z == 0.0 && !`FSM.m_active && rc[2] == 1000 && trace.size() == 1,
                  "19: nothing moved - no state of the removed auto takeoff (21-23) was ever entered");
        end

        // ── 20: הכנה על הקרקע - WP לא מניע מנועים, ודגל ההמראה (הוסר) הופך את ההגדרות ללא תקינות ──
        20: begin
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            check(rc[4] == 1000 && rc[2] == 1000, "20a: WP selected while disarmed: CH5 disarmed, throttle min");
            check(chk_bit(11) && (mst_flg & 1) == 1, "20a: settings with flag 0 -> OK (bit11, MST set_ok)");
            set_v[7] = 1; wait_s(1.5);
            check(`MRX.set_valid && `MRX.set_flags == 1 && (mst_flg & 1) == 0 && !chk_bit(11),
                  "20b: auto-takeoff flag = 1 -> set received but NOT OK (feature removed), bit11 false");
            line("CMD:CHECK"); line("CMD:ARM"); wait_s(1.0);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && `FSM.reason == 8'h01 && z == 0.0, "20c: START rejected, nothing moves");
            line("CMD:DISARM"); wait_s(0.3);
            set_v[7] = 0; wait_s(1.5);
            check((mst_flg & 1) == 1 && chk_bit(11), "20d: flag back to 0 -> settings OK");
            check(z == 0.0 && trace.size() == 1 && rc[2] == 1000, "20: during all of this the motors stayed at idle");
        end

        // ── 21: באוויר: דגל ההמראה = 1 ⇒ START נדחה (bit11). דגל 0 ⇒ START מתקבל ──
        21: begin
            set_v[7] = 1;
            setup_airborne(100.0);
            check(!chk_bit(11), $sformatf("21: flag 1 -> bit11 false in the air (%04h)", `FSM.checks));
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && `FSM.reason == 8'h01, $sformatf("21: START rejected (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            line("CMD:DISARM"); wait_s(0.3);
            check(!inav_armed, "21: disarmed to change the settings");
            z = 0.0; vz = 0.0;
            set_v[7] = 0; wait_s(1.5);
            setup_airborne(100.0);
            check(`FSM.checks == 16'hFFFF, $sformatf("21: flag 0 -> all 16 SELF_CHECK bits pass in the air (%04h)", `FSM.checks));
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            check(`FSM.st == 3, "21: START accepted in the air -> WP_ACTIVE");
        end

        // ── 22: מסירה באוויר בלי מצערת מסונכרנת ⇒ HOLD_SYNC. גם אחרי פסק הזמן
        //        אין מסירה: POSHOLD נשאר, רק מדווח SYNC_TIMEOUT. סנכרון ⇒ מסירה חלקה.
        //        (F1021E: התחלה באוויר; חוסר הסנכרון נוצר מהטייס שהוריד מצערת = השתלטות) ──
        22: begin
            app_track = 1'b0;
            setup_airborne(150.0);
            line("CMD:MSTART");
            wait_st(5'd3, 40.0);
            wait_s(1.0);
            app_thr_h = 60;
            wait_s(0.5);
            check(`FSM.st == 25 && rc[6] == 1500 && rc[2] == 1655,
                  $sformatf("22: app throttle 60 != neutral -> HOLD_SYNC keeps POSHOLD (st=%0d CH3=%0d)", `FSM.st, rc[2]));
            begin
                real z0 = z;
                wait_s(12.0);
                check(`FSM.st == 25 && `FSM.m_active && rc[6] == 1500 && rc[2] == 1655 && inav_mode == 2,
                      $sformatf("22: 12 s later - still HOLD_SYNC in POSHOLD, neutral throttle (st=%0d CH3=%0d)", `FSM.st, rc[2]));
                check(`FSM.reason == 8'h2D && mst_rs == 8'h2D, "22: SYNC_TIMEOUT reported to the app as a warning");
                check(z > z0 - 5.0 && z < z0 + 5.0, $sformatf("22: altitude held (%0.0f -> %0.0f cm)", z0, z));
            end
            app_thr_h = 131;
            wait_s(0.5);
            check(`FSM.st == 18 && rc[2] == 1655, $sformatf("22: app throttle matched -> released, CH3 %0d", rc[2]));
            wait_s(1.0);
            check(vz < 3.0 && vz > -3.0, "22: no altitude change at the handover");
        end

        // ── 23: HOLD_SYNC שמסתיים בסנכרון בתוך ה-deadband ⇒ מסירה חלקה ──
        23: begin
            app_track = 1'b0;
            setup_airborne(150.0);
            line("CMD:MSTART");
            wait_st(5'd3, 40.0);
            wait_s(1.0);
            app_thr_h = 60;
            wait_s(1.0);
            check(`FSM.st == 25 && `FSM.reason == 8'h03, $sformatf("23: throttle takeover -> HOLD_SYNC (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            app_thr_h = 128;
            wait_s(0.5);
            check(`FSM.st == 18 && `FSM.reason == 8'h03,
                  $sformatf("23: app throttle reached the neutral -> released (st=%0d rs=%02h)", `FSM.st, `FSM.reason));
            check(rc[2] == 1640 && inav_mode == 2, $sformatf("23: throttle handed over within the deadband (%0d us)", rc[2]));
            wait_s(1.0);
            check(vz < 3.0 && vz > -3.0, "23: no altitude jump after the handover");
        end

        // ── 24: שורת ההגדרות ──
        24: begin
            esp_on = 1'b1; stat_on = 1'b1;
            wait_s(1.0);
            check(!`MRX.set_valid && !chk_bit(11), "24a: after power-up the set is not valid until the ESP sends it");
            set_v = '{120, 200, 80, 90, 350, 60, 3, 0};
            set_ck_err = 1; send_settings(); wait_s(0.3);
            check(!`MRX.set_valid, "24b: wrong checksum -> not valid");
            set_ck_err = 0; set_n = 7; send_settings(); wait_s(0.3);
            check(!`MRX.set_valid, "24c: 7 values -> not valid");
            set_n = 8; send_settings(); wait_s(0.6);
            check(`MRX.set_valid && `MRX.set_stop == 120 && `MRX.set_clear == 200 && `MRX.set_vclr == 80 &&
                  `MRX.set_fclr == 90 && `MRX.set_ceil == 350 && `MRX.set_depth == 60 && `MRX.set_stage == 3 &&
                  `MRX.set_flags == 0, "24d: full set applied");
            check(mst_sck == set_sum() && (mst_flg & 1) == 1,
                  $sformatf("24d: MST confirms SCK %04h = %04h and set_ok", mst_sck, set_sum()));
            set_v[0] = 220; send_settings(); wait_s(0.6);
            check(`MRX.set_valid && (mst_flg & 1) == 0, "24e: stop >= clear -> set received but not OK (flag bit0 = 0)");
            set_v[0] = 120; send_settings(); wait_s(0.3);
            app_mode = 0; app_thr_h = 0;
            wait_s(0.5);
            line("CMD:CHECK"); line("CMD:ARM"); wait_s(0.5);
            set_v[0] = 60; send_settings(); wait_s(0.5);
            check(`MRX.set_valid && `MRX.set_stop == 120, "24f: FPGA armed -> new set ignored, old set kept valid");
            line("CMD:DISARM"); wait_s(0.3);
            send_settings(); wait_s(0.5);
            check(`MRX.set_valid && `MRX.set_stop == 60, "24g: disarmed again -> applied");
        end

        // ── 25: הגבלת Roll/Pitch ל-60% ──
        25: begin
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 0; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM"); wait_s(0.5);
            app_pch = 100; app_rll = -100; app_yaw = 100; app_thr_h = 20;
            wait_s(0.8);
            check(rc[1] == 1875 && rc[0] == 1125, $sformatf("25a: ANGLE: pitch %0d, roll %0d (1875 / 1125)", rc[1], rc[0]));
            check(rc[3] == 2000 && rc[2] == 1100, $sformatf("25a: yaw %0d and throttle %0d not limited", rc[3], rc[2]));
            app_rpl = 0; wait_s(0.12);
            check(rc[1] > 1875 && rc[1] < 2000, $sformatf("25b: disable -> gradual (pitch %0d)", rc[1]));
            wait_s(0.6);
            check(rc[1] == 2000 && rc[0] == 1000, $sformatf("25b: limit off: pitch %0d roll %0d", rc[1], rc[0]));
            app_rpl = 1; app_mode = 1; wait_s(0.8);
            check(rc[6] == 1250 && rc[1] == 1875, $sformatf("25c: ALTHOLD limited (CH7 %0d pitch %0d)", rc[6], rc[1]));
            app_mode = 4; wait_s(0.8);
            check(rc[6] == 1000 && rc[1] == 1875, "25d: WP selected on the ground = ANGLE -> limited");
            app_mode = 2; wait_s(0.8);
            check(rc[6] == 1500 && rc[1] == 2000, $sformatf("25e: POSHOLD not limited (pitch %0d)", rc[1]));
            app_pch = 0; app_rll = 0; app_yaw = 0; app_mode = 0; wait_s(0.8);
            check(rc[1] == 1500 && rc[0] == 1500, "25f: centered sticks stay centered");
        end

        // ── 26: אובדן קשר באמצע משימה - אין חזרה אוטומטית ──
        26: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            esp_on = 1'b0;
            wait_s(2.0);
            check(`FSM.st == 18 && !`FSM.m_active, $sformatf("26: comm loss -> SUSPENDED (rs=%02h)", `FSM.reason));
            esp_on = 1'b1;
            wait_s(3.0);
            check(`FSM.st == 18 && !`FSM.m_active, "26: link back -> still SUSPENDED, no automatic restart");
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st != 2 && !`FSM.m_active && !chk_bit(0), "26: START rejected while the safety layer holds the stream");
        end

        // ── 27: ה-FRONT רואה שוב מכשול ב-CLEARANCE ⇒ חזרה לטיפוס ──
        27: begin
            set_v[6] = 3;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd6, 60.0);
            wait_s(0.3);
            inj_front_hit = 1'b1;
            wait_st(5'd5, 2.0);
            inj_front_hit = 1'b0;
            wait_st(5'd18, 30.0);
            check(tstr() == "0 1 2 3 4 5 6 5 6 7 25 18 " && `FSM.reason == 8'h14,
                  $sformatf("27: re-detection during CLEARANCE -> back to CLIMB, then clear again (%s)", tstr()));
        end

        // ── 28: FRONT SHORT - אין החזר מעל המכשול ⇒ "פנוי" חלקי, DOWN מאשר את הראש ⇒ מעבר מלא ──
        28: begin
            flm = 0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 80.0);
            takeover_end();
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 25 17 ",
                  $sformatf("28: SHORT mode: no return above the obstacle + DOWN sees the top -> full crossing (%s)", tstr()));
            check(clr_inf_at_cross && !collided,
                  $sformatf("28: the clear was inferred (no return) and confirmed by DOWN; no collision (min %0.0f cm)", min_clear_top));
        end

        // ── 29: מגבלת זמן המשימה (MISSION_MAX_S=20 בקבוצה הזו) ──
        29: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 40.0);
            check(`FSM.reason == 8'h23, $sformatf("29: mission time limit -> SUSPENDED / MISSION_TIME (rs=%02h)", `FSM.reason));
            check(rc[6] == 1500, "29: POSHOLD");
        end

        // ── 30: מכשול ארוך מהמוצהר ⇒ OBST_TOO_LONG ──
        30: begin
            bx1[0] = 800.0; wp_x[0] = 1000.0; wp_x[1] = 1100.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 80.0);
            check(`FSM.reason == 8'h1D && !collided,
                  $sformatf("30: DOWN still sees a top 150cm past the declared depth -> OBST_TOO_LONG (rs=%02h x=%0.0f)", `FSM.reason, x));
        end

        // ── 31: ABORT באמצע משימה ──
        31: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd5, 60.0);
            line("CMD:ABORT");
            wait_s(0.2);
            check(`FSM.st == 18 && `FSM.reason == 8'h04 && rc[4] == 1000,
                  $sformatf("31: ABORT wins: SUSPENDED / SAFETY, CH5 disarmed (rs=%02h CH5=%0d)", `FSM.reason, rc[4]));
        end

        // ── 32: הגוף מסובב 90° - הדחיפה הולכת הצידה. אינה נספרת כהתקדמות ⇒ עצירה ──
        32: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd9, 80.0);
            heading_deg = 90.0;
            wait_st(5'd18, 30.0);
            check(`FSM.reason == 8'h2E && tstr() == "0 1 2 3 4 5 6 7 8 9 25 18 ",
                  $sformatf("32: sideways motion -> CROSS_DRIFT, never REAR_STOP (%s rs=%02h, y=%0.0f)", tstr(), `FSM.reason, y));
            check(y > 35.0 && y < 60.0 && !collided, $sformatf("32: stopped after ~40cm of side drift (y=%0.0f)", y));
            wait_s(2.0);
            check(vz < 3.0 && vz > -3.0 && z > 200.0, "32: no descent - still at clearance altitude");
        end

        // ── 33: הגוף מסובב 180° - נסיגה. אינה נספרת כהתקדמות ⇒ עצירה ──
        33: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd9, 80.0);
            heading_deg = 180.0;
            wait_st(5'd18, 30.0);
            check(`FSM.reason == 8'h2E && tstr() == "0 1 2 3 4 5 6 7 8 9 25 18 ",
                  $sformatf("33: backward motion -> CROSS_DRIFT, never REAR_STOP (%s rs=%02h)", tstr(), `FSM.reason));
            check(!collided, "33: no collision");
        end

        // ── 34: תנאי GPS: START נדחה ב-10 לוויינים או EPH 2m; מעבר עיוור נדחה ב-EPH 1.2m ──
        34: begin
            setup_airborne(100.0);
            inj_sats = 10; wait_s(1.0);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(5), "34a: 10 satellites -> START rejected (bit5)");
            inj_sats = 14; inj_eph = 200; wait_s(1.0);
            line("CMD:MSTART"); wait_s(0.5);
            check(`FSM.st == 0 && !chk_bit(5), "34b: EPH 2.0m -> START rejected (bit5)");
            inj_eph = 120; wait_s(1.0);
            line("CMD:MSTART");
            wait_st(5'd18, 80.0);
            check(`FSM.reason == 8'h30 && tstr() == "0 1 2 3 4 5 6 7 8 25 18 ",
                  $sformatf("34c: EPH 1.2m and DOWN cannot see the ground -> no blind crossing (%s rs=%02h)", tstr(), `FSM.reason));
            check(x < bx0[0] - 30.0 && !collided, "34c: stayed in front of the obstacle");
        end

        // ── 35: EPH מידרדר באמצע המעבר ⇒ עצירה ──
        35: begin
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd9, 80.0);
            wait_s(1.0);
            inj_eph = 200;
            wait_st(5'd18, 5.0);
            check(`FSM.reason == 8'h0E && !collided, $sformatf("35: GPS degraded during the crossing -> GPS stop (rs=%02h)", `FSM.reason));
            wait_s(2.0);
            check(vz < 3.0 && vz > -3.0 && z > 200.0, "35: holding at clearance altitude, no descent");
        end

        // ── 36: מצערת מסונכרנת בתחילה, יורדת לנמוך באמצע המשימה ⇒ השתלטות במצערת.
        //        אין מסירה עד שהמצערת חוזרת לנייטרל: HOLD_SYNC, CH3 נשאר בנייטרל ──
        36: begin
            app_track = 1'b0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            check(`FSM.synced, "36: throttle synced at the start of the mission");
            min_thr_air = 9999;
            app_thr_h = 20;                               // 1100us - כמעט מינימום
            wait_s(0.5);
            check(`FSM.st == 25 && rc[2] == 1655 && rc[6] == 1500,
                  $sformatf("36: low app throttle -> HOLD_SYNC, CH3 stays %0d (neutral 1655), POSHOLD", rc[2]));
            line("CMD:MSTOP");                            // גם STOP לא מוסר כשהמצערת לא תואמת
            wait_s(3.0);
            check(`FSM.st == 25 && rc[2] == 1655, "36: STOP with a low throttle -> still held");
            app_thr_h = 131;
            wait_s(0.6);
            check(`FSM.st == 18 && rc[2] == 1655, $sformatf("36: throttle back at neutral -> released (CH3 %0d)", rc[2]));
            check(min_thr_air >= 1640, $sformatf("36: lowest throttle sent in the air: %0d us - no sudden drop", min_thr_air));
            wait_s(1.0);
            check(vz < 3.0 && vz > -3.0 && z > 90.0, $sformatf("36: altitude held (z=%0.0f)", z));
        end

        // ── 37: המשימה הושלמה (DONE_HOLD) ו-STOP כשמצערת האפליקציה נמוכה ⇒ לא COMPLETE עד התאמה ──
        37: begin
            app_track = 1'b0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 150.0);
            min_thr_air = 9999;
            app_thr_h = 0;
            wait_s(0.3);
            line("CMD:MSTOP");
            wait_s(1.0);
            check(`FSM.st == 25 && rc[2] == 1655, $sformatf("37: STOP at B with throttle 0 -> HOLD_SYNC (st=%0d CH3=%0d)", `FSM.st, rc[2]));
            app_thr_h = 129;
            wait_s(0.6);
            check(`FSM.st == 17 && `FSM.reason == 8'h00, $sformatf("37: throttle matched -> COMPLETE (st=%0d)", `FSM.st));
            check(min_thr_air >= 1640, $sformatf("37: lowest throttle sent in the air: %0d us", min_thr_air));
        end

        // ── 38: שינוי מצב באפליקציה (ANGLE) כשהמצערת נמוכה ⇒ מוחזק עד התאמה, ואז ANGLE ──
        38: begin
            app_track = 1'b0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd3, 20.0);
            wait_s(1.0);
            min_thr_air = 9999;
            app_thr_h = 40; app_mode = 0;
            wait_s(1.0);
            check(`FSM.st == 25 && rc[6] == 1500 && rc[2] == 1655,
                  $sformatf("38: mode change with a low throttle -> HOLD_SYNC, CH7 POSHOLD, CH3 %0d", rc[2]));
            app_thr_h = 131;
            wait_s(0.6);
            check(`FSM.st == 18 && rc[6] == 1000 && rc[2] == 1655, $sformatf("38: matched -> released to ANGLE (CH7 %0d)", rc[6]));
            check(min_thr_air >= 1640, $sformatf("38: lowest throttle sent in the air: %0d us", min_thr_air));
        end

        // ── 39: שטח פתוח (LONG): מעל המכשול ה-FRONT מחזיר NO_SIGNAL, כמו בטיסות ──
        39: begin
            front_nr_cls = 6;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 120.0);
            takeover_end();
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 25 17 ",
                  $sformatf("39: LONG + NO_SIGNAL above the obstacle -> full crossing (%s)", tstr()));
            check(clr_inf_at_cross, "39: the clear was inferred from no-return (the old code climbed to the ceiling)");
            check(!collided && min_clear_top >= 60.0, $sformatf("39: no collision, min clearance over the top %0.0f cm", min_clear_top));
            check(z_clear_hold - z_clr_evt >= 65.0, $sformatf("39: +%0.0f cm after the inferred clear (setting 70)", z_clear_hold - z_clr_evt));
        end

        // ── 40: אין החזר, וה-DOWN לא רואה ראש ⇒ עצירה לפני מגע, בלי מעבר עיוור ──
        40: begin
            front_nr_cls = 6; dwn_nosig_from8 = 1'b1;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 120.0);
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 25 18 " && `FSM.reason == 8'h31,
                  $sformatf("40: top not confirmed by DOWN -> TOP_UNCONFIRMED, hold (%s rs=%02h)", tstr(), `FSM.reason));
            check(!collided, $sformatf("40: no collision (x=%0.0f, box %0.0f..%0.0f)", x, bx0[0], bx1[0]));
            // עד ש-DOWN (במרכז) יכול לראות ראש, קצה הפרופלור כבר 30cm מעבר לחזית - זו
            // הגאומטריה. הבדיקה: לא ממשיכים מעבר ל-30 + מרווח 30 + בלימה, ובלי מעבר עיוור.
            check(x + HALF <= bx0[0] + 80.0,
                  $sformatf("40: front prop %0.0f cm past the face (<= 30 to the DOWN + margin 30 + brake 20)", x + HALF - bx0[0]));
            wait_s(1.0);
            check(rc[6] == 1500 && vx < 2.0 && vx > -2.0, "40: POSHOLD in place, no forward motion");
        end

        // ── 41: SHORT + NO_SIGNAL ──
        41: begin
            flm = 0; front_nr_cls = 6;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 120.0);
            takeover_end();
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 25 17 " && clr_inf_at_cross && !collided,
                  $sformatf("41: SHORT + NO_SIGNAL -> full crossing, confirmed by DOWN (%s)", tstr()));
        end

        // ── 42: WP נבחר, המראה ידנית (בלי START): ANGLE עד גובה אמיתי, ואז POSHOLD ──
        42: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            check(inav_armed && rc[6] == 1000 && z == 0.0, $sformatf("42: armed on the ground, WP selected -> CH7 ANGLE (%0d)", rc[6]));
            app_thr_h = 150;                         // 75% ⇒ טיפוס ידני
            while (z < 60.0) #1000;
            check(rc[6] == 1000, $sformatf("42: at z=%0.0f (above the airborne threshold) still ANGLE (CH7 %0d)", z, rc[6]));
            while (z < 100.0) #1000;
            check(rc[6] == 1000, $sformatf("42: at z=%0.0f still ANGLE (CH7 %0d)", z, rc[6]));
            wait_s(3.0);
            check(rc[6] == 1500 && inav_mode == 2, $sformatf("42: high enough -> POSHOLD (CH7 %0d, INAV %0d, z=%0.0f)", rc[6], inav_mode, z));
            check(ch7_pos_low == 0, $sformatf("42: POSHOLD was never sent below 1m (%0d frames)", ch7_pos_low));
            wait_s(2.0);
            check(vz < 3.0 && vz > -3.0 && z >= 100.0, $sformatf("42: holding altitude (z=%0.0f vz=%0.1f)", z, vz));
            check(!`FSM.m_active && trace.size() == 1, "42: the Mission FSM never started");
        end

        // ── 43: באוויר ב-POSHOLD, הטייס בוחר WP ⇒ נשאר POSHOLD, אין ANGLE לרגע ──
        43: begin
            setup_airborne(100.0);
            ch7_not_pos = 0;
            app_mode = 4;
            wait_s(3.0);
            check(ch7_not_pos == 0 && rc[6] == 1500 && inav_mode == 2,
                  $sformatf("43: WP selected in the air from POSHOLD -> CH7 stayed POSHOLD (%0d other frames)", ch7_not_pos));
            check(vz < 3.0 && vz > -3.0, "43: still hovering");
        end

        // ── 44: LONG + OUT_OF_RANGE מעל המכשול, וה-DOWN לא רואה ראש ⇒ TOP_UNCONFIRMED ──
        //        (OOR ב-LONG כבר אינו "פנוי" מוכח - גם הוא מחייב אישור ראש)
        44: begin
            front_nr_cls = 2; dwn_nosig_from8 = 1'b1;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 120.0);
            check(tstr() == "0 1 2 3 4 5 6 7 8 9 25 18 " && `FSM.reason == 8'h31 && clr_inf_at_cross,
                  $sformatf("44: LONG + OOR is partial evidence -> DOWN must confirm -> TOP_UNCONFIRMED (%s rs=%02h)", tstr(), `FSM.reason));
            check(!collided && x + HALF <= bx0[0] + 80.0, $sformatf("44: no collision, prop %0.0f cm past the face", x + HALF - bx0[0]));
        end

        // ── 45: מדידה תקינה רחוקה מ-CLEAR (קיר רחוק) = "פנוי" מוכח ⇒ בלי דרישת ראש (כמו קודם) ──
        45: begin
            set_v[6] = 3;
            bx0[1] = 560.0; bx1[1] = 580.0; bh[1] = 450.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd18, 80.0);
            check(tstr() == "0 1 2 3 4 5 6 7 25 18 " && `FSM.reason == 8'h14 && !clr_inf_at_hold,
                  $sformatf("45: VALID far reading -> proven clear (clr_inf=%0d) (%s rs=%02h)", clr_inf_at_hold, tstr(), `FSM.reason));
        end

        // ── 46: WP + המראה ידנית, DOWN מחזיר NO_SIGNAL מהקרקע ⇒ אף פעם לא POSHOLD ──
        46: begin
            angle_vz = 1'b1; inj_dwn_nosig = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 200.0) #1000;
            app_thr_h = 131;
            wait_s(2.0);
            check(rc[6] == 1000 && ch7_pos_low == 0,
                  $sformatf("46: no-return is not altitude evidence -> still ANGLE at z=%0.0f (CH7 %0d)", z, rc[6]));
        end

        // ── 47: DOWN מפסיק לעדכן באמצע חלון העדות ⇒ איפוס, ואין POSHOLD עד עדות חדשה ──
        47: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 90.0) #1000;
            dwn_freeze = 1'b1;
            wait_s(1.5);
            check(rc[6] == 1000, $sformatf("47: DOWN frozen -> no POSHOLD (z=%0.0f CH7 %0d)", z, rc[6]));
            dwn_freeze = 1'b0;
            wait_s(2.0);
            check(rc[6] == 1000 && z > 130.0,
                  $sformatf("47: after the freeze DOWN is above its range - no new VALID run -> still ANGLE (z=%0.0f)", z));
        end

        // ── 48: אחרי POSHOLD, הטייס עובר ל-ANGLE ויורד, וחוזר ל-WP נמוך ⇒ ANGLE (אין אישור ישן) ──
        48: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            wait_s(5.0);
            check(rc[6] == 1500, $sformatf("48: qualified -> POSHOLD (z=%0.0f)", z));
            app_mode = 0; app_thr_h = 110;                // ANGLE, יורדים
            while (z > 50.0) #1000;
            app_thr_h = 130;                              // ריחוף ב-ANGLE
            wait_s(0.5);
            app_mode = 4;
            wait_s(1.5);
            check(rc[6] == 1000, $sformatf("48: back to WP at z=%0.0f -> ANGLE, the old approval is gone (CH7 %0d)", z, rc[6]));
            ch7_pos_low = 0;
            app_thr_h = 150;
            wait_s(5.0);
            check(rc[6] == 1500 && ch7_pos_low == 0, $sformatf("48: climbed again -> new evidence -> POSHOLD (z=%0.0f)", z));
        end

        // ── 49: WP + המראה ידנית, ריחוף ממושך ב-85-90 ס"מ ⇒ ANGLE (אין עדות עלייה ≥110), טיפוס ⇒ POSHOLD ──
        49: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 80.0) #1000;
            app_thr_h = 130;                              // ריחוף מתחת ל-WP_HI_CM (90)
            wait_s(4.0);
            check(rc[6] == 1000 && z > 80.0 && z < 90.0 && ch7_pos_low == 0,
                  $sformatf("49: 4 s hover at %0.0f cm (< 90) with WP selected -> still ANGLE (CH7 %0d)", z, rc[6]));
            app_thr_h = 150;
            wait_s(4.0);
            check(rc[6] == 1500 && inav_mode == 2 && ch7_pos_low == 0,
                  $sformatf("49: real climb (>=90 seen + 16 samples) -> POSHOLD at z=%0.0f, never below 1m", z));
        end

        // ── 50: עדות מלאה ב-WP, יציאה ל-ANGLE, ירידה ל-88 ס"מ (DOWN תקין ≥ 80 כל הזמן), חזרה ל-WP
        //        ⇒ העדות התאפסה ביציאה מ-WP ⇒ ANGLE ──
        50: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (rc[6] != 1500) #1000;
            app_thr_h = 130;
            check(z >= 100.0, $sformatf("50: full evidence (VALID >= 90 + 16 samples) -> POSHOLD at %0.0f cm", z));
            wait_s(0.5);
            app_mode = 0; app_thr_h = 120;                // ANGLE, ירידה - לא מתחת ל-80, כך שרק היציאה מ-WP מאפסת
            while (z > 88.0) #1000;
            app_thr_h = 130;
            app_mode = 4;
            wait_s(4.0);
            check(rc[6] == 1000 && ch7_pos_low == 0,
                  $sformatf("50: back to WP at %0.0f cm -> the earlier evidence was reset -> ANGLE (CH7 %0d)", z, rc[6]));
        end

        // ── 51: WP נבחר, מגיע ל-91 (≥ 90), יורד מהר מתחת ל-80 ומרחף ⇒ העדות מתאפסת, אין POSHOLD;
        //        טיפוס חוזר ⇒ עדות חדשה ⇒ POSHOLD, אף פעם לא מתחת ל-80 ──
        51: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 91.0) #1000;
            app_thr_h = 60;                               // ירידה מהירה מיד (פחות מ-16 דגימות ≥ 80)
            begin
                realtime t_lim = $realtime + 5.0e6;
                while (z > 72.0 && $realtime < t_lim) #1000;
            end
            app_thr_h = 130;                              // ריחוף מתחת ל-80
            check(rc[6] == 1000 && z <= 72.0, $sformatf("51: touched 91 and came down to %0.0f -> still ANGLE", z));
            wait_s(4.0);
            check(rc[6] == 1000 && z > 40.0 && z < 80.0 && ch7_pos_lt80 == 0,
                  $sformatf("51: 4 s hover at %0.0f cm (< 80) after having reached 91 -> still ANGLE (CH7 %0d)", z, rc[6]));
            app_thr_h = 150;
            wait_s(4.0);
            check(rc[6] == 1500 && ch7_pos_lt80 == 0,
                  $sformatf("51: climbed again -> POSHOLD at z=%0.0f, never below 80cm", z));
        end

        // ── 52: כמו 42, אבל DOWN ב-LONG (טווח 400): עדות הגובה עובדת גם בלי לצאת מהטווח ──
        52: begin
            angle_vz = 1'b1; dlm = 1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 100.0) #1000;
            check(rc[6] == 1000, $sformatf("52: DOWN LONG, at z=%0.0f still ANGLE", z));
            wait_s(3.0);
            check(rc[6] == 1500 && inav_mode == 2 && ch7_pos_low == 0,
                  $sformatf("52: DOWN LONG -> POSHOLD at z=%0.0f, never below 1m", z));
        end

        // ── 53 (F1021E): המראה ידנית ב-WP ועצירה ב-~93 ס"מ ⇒ POSHOLD מותר כבר שם (≥ 90 ואז 16 × ≥ 80),
        //        ואף פעם לא מתחת ל-80 ──
        53: begin
            angle_vz = 1'b1;
            esp_on = 1'b1; stat_on = 1'b1; set_on = 1'b1;
            app_mode = 4; app_thr_h = 0;
            wait_s(1.6);
            line("CMD:CHECK"); line("CMD:ARM");
            wait_s(1.0);
            app_thr_h = 150;
            while (z < 91.0) #1000;
            app_thr_h = 130;                              // ריחוף ב-~93 (עצירה מ-33cm/s: +~3cm)
            wait_s(2.0);
            check(rc[6] == 1500 && inav_mode == 2 && z >= 85.0 && z < 100.0 && ch7_pos_lt80 == 0,
                  $sformatf("53: hover at %0.0f cm after a VALID >= 90 -> POSHOLD (CH7 %0d), never below 80", z, rc[6]));
        end

        // ── 54 (F1021E): מעבר מלא עם רוח צד 4cm/s (נמדד בטיסה 4-8) ⇒ עובר, בלי מגע ──
        54: begin
            wind_lat = 4.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd24, 150.0);
            check(`FSM.st == 5'd24 && !collided && tstr() == "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 3 16 24 ",
                  $sformatf("54: side wind 4 cm/s -> full crossing (%s)", tstr()));
        end

        // ── 55 (F1021E): רוח צד 15cm/s ⇒ CROSS_DRIFT, POSHOLD נשאר (HOLD_SYNC), ה-Pitch חוזר למרכז ──
        55: begin
            wind_lat = 15.0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd9, 120.0);
            wait_st(5'd18, 30.0);
            check(`FSM.reason == 8'h2E && !collided,
                  $sformatf("55: side wind 15 cm/s -> CROSS_DRIFT (rs=%02h), no collision", `FSM.reason));
            wait_s(1.0);
            check(rc[1] == 1500 && rc[6] == 1500 && inav_mode == 2,
                  $sformatf("55: after the failure: pitch centered (%0d), CH7 POSHOLD", rc[1]));
            begin
                real y0 = y; wait_s(3.0);
                check(vy < 1.0 && vy > -1.0 && y - y0 < 3.0 && y0 - y < 3.0,
                      $sformatf("55: sticks centered -> INAV holds position again (drift %0.1f cm in 3 s)", y - y0));
            end
        end

        // ── 56 (F1021E): מהירות המעבר - Pitch 400µs ⇒ ~32cm/s במודל INAV, מתחת למגביל (40) ──
        56: begin
            real vmax = 0.0, vsum = 0.0; int vn = 0;
            setup_airborne(100.0);
            line("CMD:MSTART");
            wait_st(5'd9, 120.0);
            wait_s(2.0);
            while (`FSM.st == 5'd9 || `FSM.st == 5'd10 || `FSM.st == 5'd11) begin
                real v = (vx > 0.0) ? vx : -vx;
                if (v > vmax) vmax = v;
                vsum += v; vn++;
                #20000;
            end
            check(vn > 0 && vsum / vn > 25.0 && vmax < 40.0,
                  $sformatf("56: crossing speed avg %0.1f cm/s, max %0.1f (limiter 40)", (vn > 0) ? vsum / vn : 0.0, vmax));
            check(max_pch_cross == 1900, $sformatf("56: crossing pitch %0d us (= 1500 + 80 x 5)", max_pch_cross));
        end

        default: check(0, "unknown scenario");
        endcase

        check(bad_frames == 0 && frames > 50, $sformatf("every RC frame on the wire is well formed (%0d frames, %0d bad)", frames, bad_frames));
        $display("=== DONE  scen=%0d errors=%0d ===", scen, errors);
        $finish;
    end
endmodule
