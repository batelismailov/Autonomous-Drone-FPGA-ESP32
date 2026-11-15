`timescale 1ms/1us
// ═══════════════════════════════════════════════════════════
//   tb_stage1.sv - בדיקת שלב 1: cmd_parser + flight_fsm + msp_builder
//
//   הארכיטקטורה הנבדקת: ה-FPGA הוא שכבת בטיחות ובוררות בלבד. הוא בונה
//   את זרם ה-RC ומחליט מתי להפסיק אותו. אין כאן נחיתה, אין הורדת מצערת,
//   אין זיהוי נגיעה בקרקע ואין ALT / VSP - כל אלה באחריות INAV.
//
//   CLK_FREQ=4000 ⇒ מחזור = 0.25ms, בית UART אחד בכל מחזור
//   מודל ה-ESP: שורת שליטה כל 50ms, שורת חיישן כל 50ms,
//                דגימת FWD כל 55ms ודגימת DWN כל 57ms (FSEQ / DSEQ נפרדים)
//
//   msp_builder מחובר כמו ב-drone_top, כולל שער הבטיחות על הסטיקים, וכל
//   חבילה שיוצאת ממנו מפוענחת כאן בית אחר בית: כותרת, checksum וערכי
//   CH1-CH8. כך T11-T13 בודקות את מה שבאמת יוצא על החוט אל בקר הטיסה.
//   ה-busy של msp_builder אינו מחובר ל-flight_fsm: T3 מדמה אותו ידנית.
//
//   הרצה (ModelSim):
//     vlog -timescale 1ms/1us cmd_parser.v flight_fsm.v msp_builder.v tb_stage1.sv
//     vsim -c -voptargs=+acc tb_stage1 -do "run -all; quit -f"
// ═══════════════════════════════════════════════════════════
module tb_stage1;
    localparam integer CLK_FREQ = 4000;

    reg clk = 1'b0;
    always #0.125 clk = ~clk;
    reg rst_n = 1'b0;

    // ── מזין בתים ל-cmd_parser ──
    reg  [7:0] rx_data  = 8'd0;
    reg        rx_valid = 1'b0;
    byte unsigned q[$];

    always @(posedge clk) begin
        if (q.size() > 0) begin
            rx_data  <= q.pop_front();
            rx_valid <= 1'b1;
        end else begin
            rx_valid <= 1'b0;
        end
    end

    // ── חיווטים ──
    wire [7:0]        thr;              // חצאי אחוז, 0..200
    wire signed [7:0] yaw, pch, rll;
    wire [2:0]        mode;
    wire              ctrl_valid;
    wire [8:0]        lidar_fwd, lidar_dwn;
    wire [2:0]        lidar_fwd_st, lidar_dwn_st;
    wire [7:0]        lidar_fwd_seq, lidar_dwn_seq;
    wire [1:0]        mag_health;
    wire [2:0]        mode_out;         // אחרי שער המצפן - זה מה שמגיע ל-CH7
    wire              sens_valid;
    wire              cmd_arm, cmd_disarm, cmd_check, cmd_abort, cmd_land, cmd_ping;

    wire              armed, send_now, failsafe, rc_hold;
    wire [2:0]        state_dbg;
    wire [7:0]        esp_tx_data;
    wire              esp_tx_send;

    reg               msp_busy = 1'b0;          // מדמה את msp_builder תפוס עבור ה-FSM
    reg  [1:0]        busy_cnt = 2'd0;
    wire              esp_tx_busy = (busy_cnt != 2'd0);

    cmd_parser u_parser (
        .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid),
        .thr(thr), .yaw(yaw), .pch(pch), .rll(rll), .mode(mode), .ctrl_valid(ctrl_valid),
        .lidar_fwd(lidar_fwd), .lidar_dwn(lidar_dwn),
        .lidar_fwd_st(lidar_fwd_st), .lidar_dwn_st(lidar_dwn_st),
        .lidar_fwd_seq(lidar_fwd_seq), .lidar_dwn_seq(lidar_dwn_seq), .mag_health(mag_health),
        .sens_valid(sens_valid),
        .cmd_arm(cmd_arm), .cmd_disarm(cmd_disarm), .cmd_check(cmd_check),
        .cmd_abort(cmd_abort), .cmd_land(cmd_land), .cmd_ping(cmd_ping)
    );

    flight_fsm #(.CLK_FREQ(CLK_FREQ)) u_fsm (
        .clk(clk), .rst_n(rst_n),
        .cmd_arm(cmd_arm), .cmd_disarm(cmd_disarm), .cmd_check(cmd_check),
        .cmd_abort(cmd_abort), .cmd_ping(cmd_ping),
        .ctrl_valid(ctrl_valid),
        .sens_valid(sens_valid),
        .lidar_fwd_seq(lidar_fwd_seq), .lidar_dwn_seq(lidar_dwn_seq),
        .mode_req(mode), .mag_health(mag_health), .mode_out(mode_out),
        .armed(armed), .send_now(send_now), .msp_busy(msp_busy),
        .esp_tx_data(esp_tx_data), .esp_tx_send(esp_tx_send), .esp_tx_busy(esp_tx_busy),
        .rc_hold(rc_hold), .failsafe(failsafe), .state_dbg(state_dbg)
    );

    // ── שער הבטיחות, כמו ב-drone_top: מנוטרל ⇒ מצערת 0 וסטיקים במרכז ──
    wire [7:0]        thr_safe = armed ? thr : 8'd0;
    wire signed [7:0] yaw_safe = armed ? yaw : 8'sd0;
    wire signed [7:0] pch_safe = armed ? pch : 8'sd0;
    wire signed [7:0] rll_safe = armed ? rll : 8'sd0;

    // ── msp_builder, עם מודל UART מהיר ──
    //    busy עולה במחזור שבו tx_send פעיל, כך שבית אחד לוקח שלושה מחזורים
    //    וחבילה שלמה (~17ms) מסתיימת לפני ה-send_now הבא (20ms). כך אף חבילה
    //    לא נבלעת, וכל חבילה נושאת את הערכים העדכניים.
    wire [7:0] fc_tx_data;
    wire       fc_tx_send;
    wire       fc_pkt_busy;
    wire       fc_tx_busy = fc_tx_send;

    msp_builder u_msp (
        .clk(clk), .rst_n(rst_n),
        .thr(thr_safe), .yaw(yaw_safe), .pch(pch_safe), .rll(rll_safe),
        .mode(mode_out), .armed(armed), .surf(1'b0),
        .send_now(send_now), .busy(fc_pkt_busy),
        .tx_data(fc_tx_data), .tx_send(fc_tx_send), .tx_busy(fc_tx_busy)
    );

    // ── מפענח החבילות שיוצאות אל ה-FC ──
    byte unsigned frame[$];
    int  frames_ok = 0, frames_bad = 0;
    int  last_ch[8];                 // CH1..CH8 של החבילה התקינה האחרונה (אינדקס 0..7)
    int  ch7_seen[int];              // כל ערכי CH7 שהופיעו מאז האיפוס האחרון
    int  ch3_seen[int];              // כל ערכי CH3 שהופיעו מאז האיפוס האחרון

    always @(posedge clk) begin : fc_frames
        byte unsigned cs;
        if (fc_tx_send) begin
            frame.push_back(fc_tx_data);
            if (frame.size() == 22) begin
                cs = 8'd0;
                for (int i = 3; i <= 20; i++) cs = cs ^ frame[i];
                if (frame[0] == 8'h24 && frame[1] == 8'h4D && frame[2] == 8'h3C &&
                    frame[3] == 8'd16 && frame[4] == 8'd200 && frame[21] == cs) begin
                    for (int c = 0; c < 8; c++)
                        last_ch[c] = int'(frame[5 + 2*c]) | (int'(frame[6 + 2*c]) << 8);
                    ch7_seen[last_ch[6]] = 1;
                    ch3_seen[last_ch[2]] = 1;
                    frames_ok++;
                end else begin
                    frames_bad++;
                end
                frame.delete();
            end
        end
    end

    // ── לוכד התשובה ל-ESP, מונה חבילות MSP ומצב החימוש בחבילה האחרונה ──
    string  resp = "";
    integer msp_pulses = 0;
    bit     last_frame_armed = 0;

    always @(posedge clk) begin
        if (esp_tx_send) begin
            busy_cnt <= 2'd2;
            resp = {resp, $sformatf("%c", esp_tx_data)};
        end
        else if (busy_cnt != 2'd0) busy_cnt <= busy_cnt - 2'd1;

        if (send_now) begin
            msp_pulses       = msp_pulses + 1;
            last_frame_armed = armed;           // msp_builder נועל את armed באותו מחזור
        end
    end

    // ── מחוללי שורות ודגימות, כמו ה-ESP ──
    bit ctrl_on = 0, sens_on = 0, fsamp_on = 1, dsamp_on = 1;
    int thr_v = 50, fwd_v = 100, fok_v = 2, dwn_v = 0, dok_v = 2;
    int fseq_v = 0, dseq_v = 0;
    int mgh_v = 1;                  // בריאות המצפן בשורת החיישנים: T1-T14 רצות עם מצפן מאומת
    string mode_v = "ANGLE";
    string ctrl_tail = "";          // שדות נוספים בסוף שורת השליטה, למשל " THH:131 TH2:131"

    string mode_names[5] = '{"ANGLE", "ALTHOLD", "POSHOLD", "RTH", "WP"};
    int    mode_ch7[5]   = '{1000, 1250, 1500, 1750, 2000};

    task automatic push_line(string s);
        for (int i = 0; i < s.len(); i++) q.push_back(s[i]);
        q.push_back(8'h0D);
        q.push_back(8'h0A);
    endtask

    initial forever begin #55; if (fsamp_on) fseq_v = (fseq_v + 1) % 256; end
    initial forever begin #57; if (dsamp_on) dseq_v = (dseq_v + 1) % 256; end
    // שורת השליטה המלאה, כולל DSC ו-PHA שנועדו ל-ESP ואמורים לעבור בלי השפעה
    initial forever begin #50; if (ctrl_on) push_line($sformatf("THR:%0d YAW:0 PCH:0 RLL:0 MODE:%s DSC:0 PHA:0%s",
                                                                thr_v, mode_v, ctrl_tail)); end
    initial forever begin #50; if (sens_on) push_line($sformatf("FWD:%0d FOK:%0d DWN:%0d DOK:%0d FSEQ:%0d DSEQ:%0d PHS:3 FWA:1 MGH:%0d",
                                                                fwd_v, fok_v, dwn_v, dok_v, fseq_v, dseq_v, mgh_v)); end

    int errors = 0;
    task automatic check(bit cond, string msg);
        if (cond) $display("[%6.0f ms] PASS  %s", $realtime, msg);
        else begin $display("[%6.0f ms] FAIL  %s   (st=%0d)", $realtime, msg, u_fsm.st); errors++; end
    endtask

    // דוגם את תשובת ה-STATUS: מנקה, שולח PING וממתין לתשובה מלאה
    task automatic ping_status();
        resp = "";
        push_line("CMD:PING");
        #200;
    endtask

    initial begin
        #5 rst_n = 1'b1;

        // ═══ T1 - פענוח ═══
        ctrl_on = 1; sens_on = 1;
        fwd_v = 70; fok_v = 1; dwn_v = 12; dok_v = 4;
        #200;
        check(lidar_fwd == 70 && lidar_fwd_st == 1 && lidar_dwn == 12 && lidar_dwn_st == 4,
              "T1 parser: FWD:70 FOK:1 DWN:12 DOK:4");
        check(lidar_fwd_seq != 0 && lidar_dwn_seq != 0, "T1 parser: FSEQ / DSEQ counted");
        check(thr == 100 && mode == 0, "T1 parser: THR:50 -> thr=100 half-% units, MODE:ANGLE (DSC/PHA pass without effect)");
        fok_v = 7; #150;
        check(lidar_fwd_st == 0, "T1 parser: unknown FOK:7 -> SENSOR_FAULT");
        fok_v = 1;

        // שורת ALT / VSP ישנה: אינה חלק מהפרוטוקול יותר ונבלעת בלי לפגוע בכלום
        push_line("ALT:300 VSP:-40"); #150;
        check(thr == 100 && lidar_dwn == 12 && u_fsm.st == 0,
              "T1 parser: legacy ALT/VSP line ignored, no control/command effect");

        dok_v = 2; dwn_v = 0; #150;

        // ═══ T2 - CHECK ותשובת STATUS ═══
        //   דגלים: bit3 זרם עצור · bit2 FWD טרי · bit1 DWN טרי · bit0 שליטה טרייה
        resp = "";
        push_line("CMD:CHECK"); #200;
        check(u_fsm.st == 1, "T2 CHECK -> READY");
        check(resp.len() == 22 && resp.substr(0, 13) == "STATUS:OK:1:7-",
              $sformatf("T2 STATUS = %s (state 1, all fresh, no stop)", resp.substr(0, 13)));
        check(resp.substr(14, 20) == ":F1022B",
              $sformatf("T2 STATUS version suffix = %s", resp.substr(14, 20)));

        // ═══ T3 - חימוש וזרם ה-RC ═══
        push_line("CMD:ARM"); #100;
        check(u_fsm.st == 2 && armed == 1, "T3 ARM -> ARMED");
        check(u_fsm.msp_hold == 0 && rc_hold == 0, "T3 ARMED: RC stream running");

        msp_pulses = 0; #500;
        check(msp_pulses >= 20 && last_frame_armed == 1,
              $sformatf("T3 armed MSP frames at 50Hz (%0d frames / 500ms)", msp_pulses));

        // msp_builder תפוס ⇒ אין send_now חדש (כותב יחיד)
        msp_busy = 1'b1; msp_pulses = 0; #120;
        check(msp_pulses == 0, "T3 msp_busy: no new send_now while a frame is in flight");
        msp_busy = 1'b0; msp_pulses = 0; #200;
        check(msp_pulses >= 8, "T3 msp_busy released: stream resumes");

        // ═══ T4 - טריות דגימות לפי FSEQ / DSEQ ═══
        //   השורה נשלחת כל 50ms גם בלי דגימה חדשה, ולכן רק המונה קובע.
        fsamp_on = 0; dsamp_on = 0; #400;
        check(u_fsm.fwd_fresh == 0 && u_fsm.dwn_fresh == 0 && lidar_fwd_st == 1,
              "T4 lines repeat a cached sample (FSEQ/DSEQ frozen) -> both stale although the line says VALID");
        check(u_fsm.st == 2 && armed == 1, "T4 stale sensors do not change the flight state");
        ping_status();
        check(resp.substr(0, 13) == "STATUS:OK:2:1-",
              $sformatf("T4 STATUS = %s (armed, sensors stale)", resp.substr(0, 13)));

        fsamp_on = 1; #150;
        check(u_fsm.fwd_fresh == 1 && u_fsm.dwn_fresh == 0, "T4 FSEQ advances only -> FWD fresh, DWN stale");
        ping_status();
        check(resp.substr(0, 13) == "STATUS:OK:2:5-",
              $sformatf("T4 STATUS = %s (FWD fresh, DWN stale)", resp.substr(0, 13)));

        dsamp_on = 1; #150;
        check(u_fsm.fwd_fresh == 1 && u_fsm.dwn_fresh == 1, "T4 both counters advance -> both fresh");

        // ═══ T5 - אובדן פקודות שליטה (ה-ESP חי, שורות חיישן ממשיכות) ═══
        ctrl_on = 0; #700;
        check(u_fsm.st == 2 && u_fsm.msp_hold == 0, "T5 700ms without control: still ARMED, stream running");
        #500;
        check(u_fsm.st == 3 && u_fsm.msp_hold == 1 && failsafe == 1,
              "T5 1s without control -> LINK_LOSS, RC stream stopped (sensor lines do not mask it)");
        check(last_frame_armed == 1,
              "T5 the last frame before the stop was armed - INAV sees RX loss, not a disarm");

        msp_pulses = 0; #500;
        check(msp_pulses == 0, "T5 no MSP frames while stopped (INAV enters its own failsafe)");

        ping_status();
        check(resp.substr(0, 13) == "STATUS:OK:3:EL",
              $sformatf("T5 STATUS = %s (stopped, sensors fresh, control lost, reason L)", resp.substr(0, 13)));
        check(u_fsm.st == 3, "T5 CMD:PING does not clear the link loss");

        ctrl_on = 1; msp_pulses = 0; #300;
        check(u_fsm.st == 3 && msp_pulses == 0, "T5 control back: no automatic return, stream stays stopped");

        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 0 && armed == 0 && u_fsm.msp_hold == 0, "T5 DISARM -> DISARMED, stop released");
        msp_pulses = 0; #400;
        check(msp_pulses >= 15 && last_frame_armed == 0,
              $sformatf("T5 disarmed stream resumed (%0d frames / 400ms)", msp_pulses));

        // ═══ T6 - על הקרקע לא עוצרים את הזרם ═══
        ctrl_on = 0; sens_on = 0; msp_pulses = 0; #1700;
        check(u_fsm.st == 0 && u_fsm.esp_dead == 1 && u_fsm.msp_hold == 0,
              "T6 ESP silent while DISARMED: no stop, no state change");
        check(msp_pulses >= 60 && armed == 0,
              $sformatf("T6 disarmed frames keep flowing on the ground (%0d frames / 1.7s)", msp_pulses));
        ctrl_on = 1; sens_on = 1; #200;

        // ═══ T7 - ה-ESP עצמו מת תוך כדי חימוש ═══
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #150;
        check(u_fsm.st == 2 && armed == 1, "T7 re-ARM");

        ctrl_on = 0; sens_on = 0; #1200;
        check(u_fsm.st == 3 && u_fsm.last_end == 1, "T7 1s without control -> LINK_LOSS (reason L)");
        #500;
        // ESP מת אינו ABORT: הזרם נשאר עצור, המנועים נשארים באחריות INAV,
        // ורק סיבת האבחון משתנה. ניתוק מנועים מתבצע רק בפקודה מפורשת.
        check(u_fsm.st == 3 && u_fsm.last_end == 2 && armed == 1 && u_fsm.msp_hold == 1,
              "T7 1.5s without any line -> still LINK_LOSS, reason changes to E (no ABORT)");
        check(last_frame_armed == 1, "T7 the last frame before the stop was armed");

        msp_pulses = 0; #600;
        check(msp_pulses == 0, "T7 no frames while stopped");

        ctrl_on = 1; sens_on = 1; #400;
        check(u_fsm.st == 3 && msp_pulses == 0, "T7 ESP back: still latched, stream still stopped");
        ping_status();
        check(resp.substr(0, 13) == "STATUS:OK:3:FE",
              $sformatf("T7 STATUS = %s (stopped, everything fresh again, reason E)", resp.substr(0, 13)));

        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 0 && u_fsm.msp_hold == 0, "T7 DISARM -> DISARMED, stop released");

        // ═══ T8 - ABORT הוא מפסק חירום מיידי ═══
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #150;
        check(u_fsm.st == 2 && armed == 1, "T8 ARM");
        push_line("CMD:ABORT"); #100;
        check(u_fsm.st == 5 && armed == 0 && failsafe == 1, "T8 ABORT -> motors off immediately");
        check(u_fsm.msp_hold == 0, "T8 ABORT keeps the stream running - INAV must receive AUX1=1000");
        msp_pulses = 0; #300;
        check(msp_pulses >= 10 && last_frame_armed == 0,
              $sformatf("T8 disarmed frames after ABORT (%0d frames / 300ms)", msp_pulses));
        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 0, "T8 DISARM clears the ABORT");

        // ═══ T9 - DISARM רגיל מחזיר ל-READY ═══
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #150;
        check(u_fsm.st == 2 && armed == 1, "T9 ARM");
        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 1 && armed == 0, "T9 DISARM while ARMED -> READY (motors off)");
        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 0, "T9 DISARM in READY -> DISARMED");
        check(u_fsm.failsafe == 0 && u_fsm.msp_hold == 0, "T9 clean state: no failsafe, stream running");

        // ═══ T10 - ABORT מתוך אובדן קשר חייב לשחרר את עצירת הזרם ═══
        //   המסלול הזה לא נבדק בסבב הקודם: ABORT נבדק רק ישירות מ-ARMED, שם
        //   הזרם ממילא היה פעיל. כאן הזרם כבר עצור, ו-INAV לא יקבל את פקודת
        //   הניתוק אלא אם העצירה משוחררת.
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #150;
        check(u_fsm.st == 2 && armed == 1, "T10 ARM");

        ctrl_on = 0; #1200;
        check(u_fsm.st == 3 && rc_hold == 1, "T10 control lost -> LINK_LOSS, RC stream stopped");
        msp_pulses = 0; #200;
        check(msp_pulses == 0, "T10 no frames while stopped");

        ctrl_on = 1; #150;                 // הקשר חוזר, המצב נשאר נעול בכוונה
        check(u_fsm.st == 3, "T10 control back: still latched in LINK_LOSS");

        push_line("CMD:ABORT"); #150;
        check(u_fsm.st == 5 && armed == 0, "T10 ABORT from LINK_LOSS -> ABORT, motors off");
        check(rc_hold == 0, "T10 ABORT releases the RC stream (INAV must receive AUX1=1000)");
        msp_pulses = 0; #300;
        check(msp_pulses >= 10 && last_frame_armed == 0,
              $sformatf("T10 disarmed frames resume after ABORT (%0d frames / 300ms)", msp_pulses));
        push_line("CMD:DISARM"); #150;
        check(u_fsm.st == 0 && u_fsm.msp_hold == 0, "T10 DISARM clears the ABORT");

        // ═══ T11 - בורר מצבי הטיסה: CH7 לכל מצב, CH6 קבוע, CH5 לפי החימוש ═══
        //   המערכת מנוטרלת (סוף T10), והזרם פעיל גם כך - זו בדיוק בדיקת הקרקע
        //   בלשונית Receiver של INAV. הבקשה עוברת גם כשלא מחומשים; INAV הוא
        //   שמחליט אם להפעיל את המצב.
        #200;
        check(last_ch[4] == 1000, $sformatf("T11 not armed -> CH5=%0d (AUX1 disarm)", last_ch[4]));
        check(last_ch[2] == 1000 && last_ch[0] == 1500 && last_ch[1] == 1500 && last_ch[3] == 1500,
              "T11 not armed -> CH3=1000 and sticks centered although the line says THR:50");

        for (int m = 0; m < 5; m++) begin
            mode_v = mode_names[m]; #200;
            check(mode == m && last_ch[6] == mode_ch7[m] && last_ch[5] == 1500 && last_ch[7] == 1000,
                  $sformatf("T11 MODE:%s -> CH7=%0d CH6=%0d CH8=%0d",
                            mode_names[m], last_ch[6], last_ch[5], last_ch[7]));
        end

        mode_v = "ANGLE"; #150;
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #200;
        check(u_fsm.st == 2 && armed == 1 && last_ch[4] == 2000,
              $sformatf("T11 ARM -> CH5=%0d (AUX1 arm)", last_ch[4]));
        check(last_ch[2] == 1500 && last_ch[6] == 1000,
              $sformatf("T11 armed, THR:50 ANGLE -> CH3=%0d CH7=%0d", last_ch[2], last_ch[6]));

        // ═══ T12 - מעבר ישיר, בלי אף חבילת ביניים ═══
        //   הטווחים ב-INAV צמודים: מעבר הדרגתי מ-ALTHOLD ל-WP היה עובר דרך
        //   POSHOLD (1500) ו-RTH (1750) ומפעיל אותם בדרך.
        mode_v = "ALTHOLD"; #200;
        check(last_ch[6] == 1250, "T12 ALTHOLD selected, CH7=1250");

        ch7_seen.delete();
        mode_v = "WP"; #300;
        check(last_ch[6] == 2000 && ch7_seen.exists(2000), "T12 ALTHOLD -> WP reaches CH7=2000");
        check(!ch7_seen.exists(1500) && !ch7_seen.exists(1750) && !ch7_seen.exists(1000),
              "T12 ALTHOLD -> WP: not a single frame with CH7=1500 (POSHOLD) or 1750 (RTH) on the way");

        ch7_seen.delete();
        mode_v = "ANGLE"; #300;
        check(last_ch[6] == 1000 &&
              !ch7_seen.exists(1250) && !ch7_seen.exists(1500) && !ch7_seen.exists(1750),
              "T12 WP -> ANGLE: straight to 1000, no RTH/POSHOLD/ALTHOLD frame on the way");

        // ═══ T13 - ערך MODE לא מוכר אינו משנה את המצב ═══
        mode_v = "RTH"; #200;
        check(mode == 3 && last_ch[6] == 1750, "T13 RTH selected, CH7=1750");
        mode_v = "MAN"; #200;
        check(mode == 3 && last_ch[6] == 1750, "T13 legacy MODE:MAN is ignored - the mode stays RTH");
        mode_v = "RTL"; #200;
        check(mode == 3 && last_ch[6] == 1750, "T13 MODE:RTL (one letter off RTH) is ignored");
        mode_v = "ANGLE"; #200;
        check(mode == 0 && last_ch[6] == 1000, "T13 back to ANGLE, CH7=1000");

        push_line("CMD:DISARM"); #200;
        check(armed == 0 && last_ch[4] == 1000 && last_ch[2] == 1000,
              $sformatf("T13 DISARM -> CH5=%0d CH3=%0d", last_ch[4], last_ch[2]));
        check(frames_bad == 0 && frames_ok > 0,
              $sformatf("T11-T13 every frame on the wire is well formed (%0d frames, %0d bad)",
                        frames_ok, frames_bad));

        // ═══ T14 - מצערת בחצאי אחוז (TH2) ═══
        //   thr נשמר ביחידות של חצי אחוז. THR לבדו (ESP ישן) נותן בדיוק את מה
        //   שנתן קודם; TH2 - שה-ESP מוסיף אחרי אכיפת הרצפה - דורס אותו. THH של
        //   האפליקציה אינו מפוענח ב-FPGA.
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #200;
        check(u_fsm.st == 2 && armed == 1, "T14 ARM");

        thr_v = 65; ctrl_tail = ""; #200;
        check(thr == 130 && last_ch[2] == 1650, $sformatf("T14 THR:65 only -> thr=%0d CH3=%0d (same as before)", thr, last_ch[2]));

        ctrl_tail = " THH:131 TH2:131"; #200;
        check(thr == 131 && last_ch[2] == 1655, $sformatf("T14 THR:65 THH:131 TH2:131 -> thr=%0d CH3=%0d (65.5%%)", thr, last_ch[2]));
        // thr ננעל רק בסוף השורה: אף חבילה לא נושאת את ערך הביניים של THR:65 (1650)
        ch3_seen.delete(); #300;
        check(ch3_seen.exists(1655) && !ch3_seen.exists(1650),
              "T14 300ms of THR:65 ... TH2:131 lines: every frame CH3=1655, not one frame with the mid-line 1650");

        ctrl_tail = " THH:131 TH2:130"; #200;
        check(thr == 130 && last_ch[2] == 1650, $sformatf("T14 TH2 wins over THH (floored by the ESP) -> CH3=%0d", last_ch[2]));

        ctrl_tail = " THH:140"; #200;
        check(thr == 130 && last_ch[2] == 1650, $sformatf("T14 THH alone is ignored by the FPGA -> CH3=%0d from THR:65", last_ch[2]));

        thr_v = 100; ctrl_tail = " TH2:250"; #200;
        check(thr == 200 && last_ch[2] == 2000, $sformatf("T14 TH2:250 clamps to 200 -> CH3=%0d", last_ch[2]));

        thr_v = 0; ctrl_tail = " TH2:1"; #200;
        check(thr == 1 && last_ch[2] == 1005, $sformatf("T14 TH2:1 -> CH3=%0d (0.5%%)", last_ch[2]));

        thr_v = 50; ctrl_tail = " TH2:131"; #100;
        push_line("CMD:DISARM"); #200;
        check(armed == 0 && last_ch[2] == 1000 && last_ch[4] == 1000,
              $sformatf("T14 DISARM -> CH3=%0d although TH2:131 (safety gate)", last_ch[2]));
        ctrl_tail = ""; push_line("CMD:DISARM"); #150;
        check(frames_bad == 0,
              $sformatf("T14 every frame on the wire is well formed (%0d frames, %0d bad)", frames_ok, frames_bad));

        // ═══ T15 - שער מצבי הניווט לפי בריאות המצפן (MGH) ═══
        //   0 לא מאומת · 1 תקין · 2 קפוא. ANGLE ו-ALTHOLD תמיד מותרים.
        //   POSHOLD / RTH / WP: כניסה חדשה רק עם 1. מצב פעיל נשאר גם ב-0.
        //   2 בזמן מצב ניווט ⇒ ALTHOLD, בלי DISARM ובלי לגעת במצערת.
        push_line("CMD:CHECK"); #100;
        push_line("CMD:ARM");   #200;
        check(u_fsm.st == 2 && armed == 1, "T15 ARM");

        mode_v = "ANGLE"; mgh_v = 0; #300;
        check(mag_health == 0 && last_ch[6] == 1000, "T15 MGH:0 (not verified), ANGLE -> CH7=1000");
        ch7_seen.delete();
        mode_v = "POSHOLD"; #300;
        check(mode == 2 && mode_out == 0 && last_ch[6] == 1000 && !ch7_seen.exists(1500),
              "T15 not verified: POSHOLD requested -> blocked, CH7 stays 1000, not one POSHOLD frame");
        mode_v = "RTH"; #300;
        check(last_ch[6] == 1000 && !ch7_seen.exists(1750), "T15 not verified: RTH blocked");
        mode_v = "WP"; #300;
        check(last_ch[6] == 1000 && !ch7_seen.exists(2000), "T15 not verified: WP blocked");
        mode_v = "ALTHOLD"; #300;
        check(last_ch[6] == 1250, "T15 not verified: ALTHOLD allowed -> CH7=1250");
        mode_v = "POSHOLD"; #300;
        check(last_ch[6] == 1250, "T15 not verified: ALTHOLD -> POSHOLD blocked, stays ALTHOLD");
        mode_v = "ANGLE"; #300;
        check(last_ch[6] == 1000, "T15 not verified: ANGLE allowed -> CH7=1000");

        mgh_v = 1; #200;
        mode_v = "POSHOLD"; #300;
        check(mag_health == 1 && last_ch[6] == 1500, "T15 MGH:1 verified: POSHOLD -> CH7=1500");

        // מאומת → לא מאומת בזמן POSHOLD (למשל הפסקה בנתוני ה-FC): אין מעבר אוטומטי
        mgh_v = 0; #400;
        check(last_ch[6] == 1500 && armed == 1,
              "T15 in POSHOLD, MGH drops to 0: stays POSHOLD (UNVERIFIED never forces a transition)");
        mode_v = "RTH"; #300;
        check(last_ch[6] == 1500, "T15 ...but a new switch POSHOLD -> RTH is blocked (stays POSHOLD)");

        // קפוא בזמן POSHOLD ⇒ ALTHOLD ישירות, בלי ניתוק ובלי לגעת במצערת
        mode_v = "POSHOLD"; mgh_v = 1; #200;
        ch7_seen.delete(); ch3_seen.delete(); msp_pulses = 0;
        mgh_v = 2; #300;
        check(mag_health == 2 && last_ch[6] == 1250 && !ch7_seen.exists(1000) && !ch7_seen.exists(1750),
              "T15 MGH:2 frozen in POSHOLD -> ALTHOLD (1250) directly, no ANGLE/RTH frame on the way");
        check(armed == 1 && u_fsm.st == 2 && last_ch[4] == 2000 && msp_pulses >= 10,
              $sformatf("T15 failover: still ARMED, CH5=%0d, stream running (%0d frames) - no DISARM", last_ch[4], msp_pulses));
        check(ch3_seen.size() == 1 && ch3_seen.exists(1500),
              "T15 failover does not touch the throttle: every frame CH3=1500 (THR:50)");
        #500;
        check(last_ch[6] == 1250, "T15 frozen, request still POSHOLD -> stays ALTHOLD (no re-entry)");
        mode_v = "RTH"; #300;
        check(last_ch[6] == 1250, "T15 frozen: RTH blocked -> stays ALTHOLD");
        mode_v = "WP"; #300;
        check(last_ch[6] == 1250, "T15 frozen: WP blocked -> stays ALTHOLD");
        mode_v = "ANGLE"; #300;
        check(last_ch[6] == 1000, "T15 frozen: ANGLE allowed");
        mode_v = "POSHOLD"; #300;
        check(last_ch[6] == 1000, "T15 frozen: ANGLE -> POSHOLD blocked, stays ANGLE");
        mode_v = "ALTHOLD"; #300;
        check(last_ch[6] == 1250, "T15 frozen: ALTHOLD allowed");

        // אותו failover מ-RTH ומ-WP
        mgh_v = 1; mode_v = "ANGLE"; #300;
        mode_v = "RTH"; #300;
        check(last_ch[6] == 1750, "T15 verified: RTH -> 1750");
        mgh_v = 2; #300;
        check(last_ch[6] == 1250 && armed == 1, "T15 MGH:2 frozen in RTH -> ALTHOLD (1250), still armed");
        mgh_v = 1; mode_v = "ANGLE"; #300;
        mode_v = "WP"; #300;
        check(last_ch[6] == 2000, "T15 verified: WP -> 2000");
        mgh_v = 2; #300;
        check(last_ch[6] == 1250 && armed == 1, "T15 MGH:2 frozen in WP -> ALTHOLD (1250), still armed");

        // דיווח ישן: שורות החיישנים נעצרו (השליטה ממשיכה) ⇒ כניסה חדשה חסומה
        mgh_v = 1; mode_v = "ANGLE"; #300;
        sens_on = 0; #400;
        mode_v = "POSHOLD"; #300;
        check(last_ch[6] == 1000 && u_fsm.st == 2,
              "T15 last MGH:1 but no sensor line for 400ms -> POSHOLD blocked, flight state unchanged");
        sens_on = 1; #200;
        check(last_ch[6] == 1500, "T15 sensor lines back with MGH:1 -> POSHOLD enters");

        // ערך לא מוכר נחשב לא מאומת
        mode_v = "ANGLE"; #200;
        mgh_v = 7; #200;
        check(mag_health == 0, "T15 unknown MGH:7 -> parsed as 0 (not verified)");
        mode_v = "POSHOLD"; #300;
        check(last_ch[6] == 1000, "T15 unknown MGH value: POSHOLD blocked");

        mode_v = "ANGLE"; mgh_v = 1; #200;
        push_line("CMD:DISARM"); #200;
        check(armed == 0 && last_ch[4] == 1000, "T15 DISARM");
        check(frames_bad == 0,
              $sformatf("T15 every frame on the wire is well formed (%0d frames, %0d bad)", frames_ok, frames_bad));

        $display("=== DONE  errors=%0d ===", errors);
        $finish;
    end
endmodule
