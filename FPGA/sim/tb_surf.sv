`timescale 1us/1ns
// ═══════════════════════════════════════════════════════════
//   tb_surf.sv - F1022A: SURFACE ידני על CH8 + פולס הכניסה מחדש ב-CH7
//
//   שני עותקים של drone_top מקבלים את אותו זרם בתים:
//     dut - SURF_ENABLE=1 (ברירת המחדל)
//     ref - SURF_ENABLE=0 (= ההתנהגות של F1021E: CH8 קבוע 1000)
//   כל חבילת MSP_SET_RAW_RC שיוצאת על fc_tx מפוענחת (כותרת, checksum, CH1-CH8).
//
//   T1  POSHOLD בלי SRF ⇒ זהה ל-ref בכל מחזור, CH8 = 1000
//   T2  SRF:1 ב-POSHOLD ⇒ CH8 = 2000, ובדיוק 10 חבילות CH7 = 1250 (ALTHOLD) ואז 1500
//   T3  SRF:0 ⇒ CH8 = 1000 + שוב 10 חבילות של 1250
//   T4  SRF:1 ב-ALTHOLD ⇒ CH8 נשאר 1000, אין פולס
//   T5  SRF:1 ב-POSHOLD ואז הטייס עובר ל-ANGLE ⇒ CH8 = 1000 מיד, CH7 = 1000, אין פולס
//   T6  שער המצפן (MGH:2 ⇒ failover של flight_fsm ל-ALTHOLD) ⇒ CH8 = 1000
//   T7  ref עם SRF:1 ⇒ CH8 תמיד 1000 ו-CH7 בלי פולס
//   T9  (E1022B) מסירת מצערת: TH2 = מרכז ואחרי 100ms SRF:0 ⇒ CH3 = המרכז בכל חבילות הכניסה מחדש
//   T10 (F1022B) יציאה עם THS: 5 חבילות הקדמה (CH3 = THS, CH8 = 2000), 10 חבילות ALTHOLD, ואז החזקה
//       ללא טיימר עד שהאפליקציה מתיישרת (±1%)
//   T11 (F1022B) MGH:2 בזמן SURFACE: הקדמה של 5 חבילות ואז ה-failover ל-ALTHOLD, עם החזקת המצערת
//   T12 (F1022B) פער של 700ms בשורות השליטה ברגע SRF:0 - ההקדמה וההחזקה לא תלויות בהן
//   T13 (F1022B) אובדן קשר מלא אחרי SRF:0 - LINK_LOSS בדיוק 1000ms (ה-ESP לא מייצר שורות שליטה)
//   T14 (E1022H) THS:-1 (מגע מאומת) לפני SRF:0, האפליקציה 0% ⇒ יציאה מיידית, CH3 = 1000, אין החזקת THS
//   T15 (E1022H2) הסדר הישן "SRF:0 THS:-1" באותה שורה ⇒ החזקה עם THS ישן (שחזור הבאג)
//   T16 (E1022H2) הסדר החדש "THS:-1 SRF:0" באותה שורה ⇒ יציאה מיידית, CH3 = 0% של האפליקציה
//   T17 (E1022H2) navTouch מגיע 300ms אחרי תחילת היציאה ⇒ ההחזקה משתחררת ב-app_match (ערך המסירה)
//   T8  פרוטוקול: SRF:2 = 0 (רק "1" מבקש); checksum תקין בכל החבילות
//
//   הרצה (Verilator 5, מתוך FPGA-Drone):
//     $ verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style --default-language 1364-2005
//       +1800-2017ext+sv --top-module tb_surf sim/tb_surf.sv drone_top.v uart_rx.v uart_tx.v
//       cmd_parser.v msp_builder.v flight_fsm.v mission_rx.v mission_fsm.v mission_tx.v -o tbs
// ═══════════════════════════════════════════════════════════
module tb_surf;
    localparam integer CLK  = 1_000_000;
    localparam integer BAUD = 100_000;
    localparam integer BIT  = CLK / BAUD;

    reg clk = 1'b0;
    always #0.5 clk = ~clk;

    reg  esp_rx = 1'b1;
    wire fc_rx  = 1'b1;
    wire d_esp_tx, d_fc_tx, r_esp_tx, r_fc_tx;
    wire [2:0] d_led, r_led;

    drone_top #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD)) u_dut (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(d_esp_tx), .fc_tx(d_fc_tx), .fc_rx(fc_rx), .led(d_led));
    drone_top #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD), .SURF_ENABLE(0)) u_ref (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(r_esp_tx), .fc_tx(r_fc_tx), .fc_rx(fc_rx), .led(r_led));

    // ── משדר UART אל esp_rx ──
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

    // ── מפענח חבילות RC על fc_tx ──
    typedef struct { real t; int ch[8]; } frame_t;
    frame_t dfr[$], rfr[$];
    int bad_csum = 0;
    task automatic rx_frames(input bit which);
        byte unsigned buf_[$];
        forever begin
            byte unsigned b;
            if (which == 0) @(negedge d_fc_tx); else @(negedge r_fc_tx);
            #(BIT * 1.5);
            for (int i = 0; i < 8; i++) begin
                b[i] = (which == 0) ? d_fc_tx : r_fc_tx;
                #(BIT);
            end
            buf_.push_back(b);
            while (buf_.size() > 0 && buf_[0] != 8'h24) void'(buf_.pop_front());
            if (buf_.size() >= 22) begin
                if (buf_[1] == 8'h4D && buf_[2] == 8'h3C && buf_[3] == 8'd16 && buf_[4] == 8'd200) begin
                    frame_t f; byte unsigned cs = 0;
                    for (int k = 3; k <= 20; k++) cs ^= buf_[k];
                    if (cs != buf_[21]) bad_csum++;
                    f.t = $realtime;
                    for (int c = 0; c < 8; c++) f.ch[c] = buf_[5 + 2*c] | (buf_[6 + 2*c] << 8);
                    if (which == 0) dfr.push_back(f); else rfr.push_back(f);
                end
                for (int k = 0; k < 22; k++) void'(buf_.pop_front());
            end
        end
    endtask
    initial rx_frames(0);
    initial rx_frames(1);

    // ── השוואה מחזור-מחזור (ב-T1) ──
    bit cmp_on = 1'b0;
    longint d_fc = 0;
    always @(posedge clk) if (cmp_on && (d_fc_tx != r_fc_tx || d_esp_tx != r_esp_tx || d_led != r_led)) d_fc++;

    int errors = 0;
    task automatic check(bit cond, string msg);
        if (cond) $display("[%8.1f ms] PASS  %s", $realtime / 1000.0, msg);
        else begin $display("[%8.1f ms] FAIL  %s", $realtime / 1000.0, msg); errors++; end
    endtask

    // ── זרם ה-ESP: שורת שליטה + שורת חיישן כל 50ms ──
    string mode_s = "ANGLE";
    int    srf_v  = 0;
    int    mgh_v  = 1;
    int    th2_v  = 0;
    int    dseq = 0, fseq = 0;
    bit    stream_on = 1'b1;
    bit    ctrl_on   = 1'b1;      // F1022B: שורות השליטה (האפליקציה) בלבד - פער WiFi
    int    ths_v     = -1;        // F1022B: THS בשורת החיישנים; ‎-1 = לא נשלח ערך תקין
    bit    ths_first = 1'b0;      // E1022H2: THS לפני SRF באותה שורה
    real   last_ctrl_t = 0;
    initial begin
        #2000;
        forever begin
            if (stream_on) begin
                if (ctrl_on) begin
                    line($sformatf("THR:0 YAW:0 PCH:0 RLL:0 MODE:%s DSC:0 PHA:0 TH2:%0d RPL:0", mode_s, th2_v));
                    last_ctrl_t = $realtime;
                end
                dseq = (dseq + 1) % 256; fseq = (fseq + 1) % 256;
                if (ths_first)   // E1022H2: סדר השדות של ה-ESP החדש - THS לפני SRF
                    line($sformatf("FWD:300 FOK:1 DWN:60 DOK:1 FSEQ:%0d DSEQ:%0d PHS:0 FWA:0 MGH:%0d FRC:1 DRC:1 FLM:1 DLM:0 THS:%0d SRF:%0d", fseq, dseq, mgh_v, ths_v, srf_v));
                else
                    line($sformatf("FWD:300 FOK:1 DWN:60 DOK:1 FSEQ:%0d DSEQ:%0d PHS:0 FWA:0 MGH:%0d FRC:1 DRC:1 FLM:1 DLM:0 SRF:%0d THS:%0d", fseq, dseq, mgh_v, srf_v, ths_v));
            end
            #50000;
        end
    end

    // פריימים אחרי זמן t0 (מילישניות)
    function automatic int count_ch(input real t0, input real t1, input int c, input int v);
        int n = 0;
        foreach (dfr[i]) if (dfr[i].t >= t0 * 1000 && dfr[i].t < t1 * 1000 && dfr[i].ch[c] == v) n++;
        return n;
    endfunction
    function automatic int first_idx_after(input real t0);
        foreach (dfr[i]) if (dfr[i].t >= t0 * 1000) return i;
        return -1;
    endfunction
    // רצף ערכי CH7 מחבילה i: כמה חבילות ברצף עם ערך v
    function automatic int run_len(input int i0, input int c, input int v);
        int n = 0;
        for (int i = i0; i < dfr.size(); i++) begin
            if (dfr[i].ch[c] == v) n++;
            else if (n > 0) break;
        end
        return n;
    endfunction

    real t;
    int  i0, n;
    initial begin
        // ── T1 ──
        mode_s = "POSHOLD"; srf_v = 0; cmp_on = 1'b1;
        #1500000;
        check(d_fc == 0, $sformatf("T1 POSHOLD בלי SRF: זהה ל-ref בכל מחזור (הבדלים=%0d)", d_fc));
        check(count_ch(500, 1500, 7, 2000) == 0 && count_ch(500, 1500, 6, 1500) > 40, "T1 CH8=1000, CH7=1500");
        cmp_on = 1'b0;

        // ── T2 ──
        t = $realtime / 1000.0; srf_v = 1;
        #1500000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[7] == 2000) i0 = i;
        check(i0 >= 0, "T2 CH8 עלה ל-2000");
        if (i0 >= 0) begin
            n = run_len(i0, 6, 1250);
            check(dfr[i0].ch[6] == 1250 && n == 10, $sformatf("T2 פולס ALTHOLD מתחיל באותה חבילה, 10 חבילות (%0d)", n));
            check(dfr[i0 + n].ch[6] == 1500 && dfr[i0 + n].ch[7] == 2000, "T2 חזרה ל-POSHOLD, CH8 נשאר 2000");
            check((dfr[i0 + n].t - dfr[i0].t) / 1000.0 > 190 && (dfr[i0 + n].t - dfr[i0].t) / 1000.0 < 230,
                  $sformatf("T2 משך הפולס %.0f ms", (dfr[i0 + n].t - dfr[i0].t) / 1000.0));
        end
        check(count_ch(t + 600, t + 1500, 6, 1250) == 0, "T2 אחרי הפולס - אין עוד ALTHOLD");
        check(rfr.size() > 0 && count_ch(t, t + 1500, 7, 2000) > 0, "T2 dut עם CH8");

        // ── T3 ──
        t = $realtime / 1000.0; srf_v = 0;
        #1500000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[7] == 1000) i0 = i;
        check(i0 >= 0, "T3 CH8 ירד ל-1000");
        if (i0 >= 0) begin
            n = run_len(i0, 6, 1250);
            check(dfr[i0].ch[6] == 1250 && n == 10, $sformatf("T3 פולס ALTHOLD ביציאה, 10 חבילות (%0d)", n));
        end

        // ── T4 ──
        t = $realtime / 1000.0; mode_s = "ALTHOLD"; srf_v = 1;
        #1500000;
        check(count_ch(t, t + 1500, 7, 2000) == 0, "T4 ALTHOLD + SRF:1 ⇒ CH8 נשאר 1000");
        check(count_ch(t + 200, t + 1500, 6, 1250) > 40 && count_ch(t + 200, t + 1500, 6, 1500) == 0, "T4 CH7=1250 בלי פולס");

        // ── T5 ──
        mode_s = "POSHOLD"; #1500000;
        t = $realtime / 1000.0; mode_s = "ANGLE";
        #1500000;
        i0 = first_idx_after(t + 150);
        check(count_ch(t + 150, t + 1500, 7, 2000) == 0, "T5 POSHOLD+SURFACE ⇒ ANGLE: CH8=1000");
        check(count_ch(t + 150, t + 1500, 6, 1250) == 0 && count_ch(t + 150, t + 1500, 6, 1000) > 20, "T5 CH7=1000, אין פולס");

        // ── T6 ──
        mode_s = "POSHOLD"; srf_v = 1; #1500000;
        check(count_ch($realtime / 1000.0 - 800, $realtime / 1000.0, 7, 2000) > 20, "T6 לפני: CH8=2000");
        t = $realtime / 1000.0; mgh_v = 2;
        #1500000;
        check(count_ch(t + 300, t + 1500, 7, 2000) == 0, "T6 MGH:2 (מצפן קפוא ⇒ failover ל-ALTHOLD) ⇒ CH8=1000");
        check(count_ch(t + 300, t + 1500, 6, 1250) > 20 && count_ch(t + 300, t + 1500, 6, 1500) == 0, "T6 CH7=1250 (ה-failover עצמו הוא הכניסה מחדש)");
        mgh_v = 1;

        // ── T7 ──
        check(rfr.size() > 200, $sformatf("T7 ref שידר %0d חבילות", rfr.size()));
        begin
            int c8 = 0, c7 = 0;
            foreach (rfr[i]) begin if (rfr[i].ch[7] != 1000) c8++; end
            check(c8 == 0, "T7 ref (SURF_ENABLE=0) - CH8 תמיד 1000");
        end

        // ── T8 ──
        #1500000;
        t = $realtime / 1000.0;
        check(count_ch(t - 700, t, 7, 2000) > 20, "T8 לפני: SURFACE פעיל");
        srf_v = 2; t = $realtime / 1000.0;
        #800000;
        check(count_ch(t + 200, t + 800, 7, 2000) == 0, "T8 SRF:2 ⇒ כבוי");
        check(bad_csum == 0, $sformatf("T8 checksum תקין בכל %0d החבילות", dfr.size()));

        // ── T9 (E1022B): מסירת המצערת - שורת שליטה עם TH2 = מרכז, ואחרי 100ms SRF:0 ──
        //   המרכז חייב להיות ב-CH3 בכל חבילות הכניסה מחדש, ולפחות 4 חבילות לפני הראשונה שבהן.
        srf_v = 0; mode_s = "POSHOLD"; #1000000;
        line("CMD:CHECK"); #200000; line("CMD:ARM"); #300000;   // חימוש: אחרת שער הבטיחות שולח CH3 = 1000
        check(u_dut.armed == 1'b1, "T9 armed (FPGA)");
        srf_v = 1; th2_v = 0; #1500000;                       // SURFACE פעיל, מקל למטה (0%)
        t = $realtime / 1000.0; th2_v = 110;                   // ה-ESP: שורת המסירה
        #100000; srf_v = 0;                                    // ה-ESP: SRF:0 אחרי 100ms
        #1000000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[7] == 1000) i0 = i;
        check(i0 >= 0 && run_len(i0, 6, 1250) == 10, "T9 SRF:0 ⇒ CH8 1000 + 10 חבילות ALTHOLD");
        if (i0 >= 0) begin
            int bad = 0, pre = 0;
            for (int k = i0; k < i0 + 10; k++) if (dfr[k].ch[2] != 1550) bad++;
            for (int k = i0 - 1; k >= 0 && dfr[k].ch[2] == 1550; k--) pre++;
            check(bad == 0, "T9 כל 10 חבילות הכניסה מחדש נושאות CH3 = 1550 (המרכז, 55%) ולא 1000");
            check(pre >= 4, $sformatf("T9 המרכז כבר ב-CH3 %0d חבילות לפני הכניסה מחדש (≥ 4)", pre));
        end
        th2_v = 0;
        line("CMD:DISARM"); #300000;

        // ═══ F1022B: מסירת המצערת ב-FPGA ═══
        line("CMD:CHECK"); #200000; line("CMD:ARM"); #300000;
        check(u_dut.armed == 1'b1, "T10 armed");
        // ── T10: יציאה עם THS תקין, האפליקציה עדיין 0% ──
        mode_s = "POSHOLD"; mgh_v = 1; th2_v = 0; ths_v = 110; srf_v = 1; #1500000;
        t = $realtime / 1000.0; srf_v = 0;
        #6000000;                                             // 6 שנ' בלי שהאפליקציה מתיישרת
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[2] == 1550) i0 = i;
        check(i0 >= 0, "T10 CH3 עובר ל-1550 (THS 110)");
        if (i0 >= 0) begin
            int lead = 0, pul = 0, k;
            for (k = i0; k < dfr.size() && dfr[k].ch[2] == 1550 && dfr[k].ch[7] == 2000 && dfr[k].ch[6] == 1500; k++) lead++;
            for (; k < dfr.size() && dfr[k].ch[2] == 1550 && dfr[k].ch[7] == 1000 && dfr[k].ch[6] == 1250; k++) pul++;
            check(lead == 5, $sformatf("T10 הקדמה: %0d חבילות עם CH3 = THS, ‏CH8 = 2000, ‏CH7 = POSHOLD (5)", lead));
            check(pul == 10, $sformatf("T10 אחריה הכניסה מחדש: %0d חבילות ALTHOLD, ‏CH8 = 1000, ‏CH3 = THS (10)", pul));
            check(dfr[i0 - 1].ch[2] == 1000, "T10 לפני ההקדמה CH3 = 0% (מקל למטה ב-SURFACE)");
        end
        check(count_ch(t + 5000, t + 6000, 2, 1550) > 40 && count_ch(t + 5000, t + 6000, 2, 1000) == 0,
              "T10 אחרי 5-6 שנ' האפליקציה עדיין 0% - ההחזקה נשארת (אין שחרור לפי טיימר)");
        t = $realtime / 1000.0; th2_v = 111;                  // האפליקציה מתיישרת (±1%)
        #400000;
        check(count_ch(t + 150, t + 400, 2, 1555) > 8, "T10 האפליקציה שולחת 111 ⇒ ההחזקה משתחררת, CH3 = 1555 (ערך האפליקציה)");
        t = $realtime / 1000.0; th2_v = 0; #400000;
        check(count_ch(t + 150, t + 400, 2, 1000) > 8, "T10 אחר כך האפליקציה שולטת (0% עובר)");

        // ── T11: מצפן קפוא (MGH:2) בזמן נחיתה ב-SURFACE, מקל למטה ──
        th2_v = 0; ths_v = 110; srf_v = 1; mgh_v = 1; #1500000;
        check(count_ch($realtime / 1000.0 - 700, $realtime / 1000.0, 7, 2000) > 20, "T11 לפני: SURFACE פעיל");
        t = $realtime / 1000.0; mgh_v = 2; srf_v = 0;          // ה-ESP: MGH:2 ו-SRF:0 באותה שורה
        #1500000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[2] == 1550) i0 = i;
        check(i0 >= 0, "T11 CH3 עובר ל-THS");
        if (i0 >= 0) begin
            int lead = 0, k;
            for (k = i0; k < dfr.size() && dfr[k].ch[2] == 1550 && dfr[k].ch[7] == 2000 && dfr[k].ch[6] == 1500; k++) lead++;
            check(lead == 5, $sformatf("T11 הקדמה של 5 חבילות לפני ה-failover (%0d): CH7 עדיין POSHOLD, ‏CH3 = THS", lead));
            check(k < dfr.size() && dfr[k].ch[6] == 1250 && dfr[k].ch[7] == 1000 && dfr[k].ch[2] == 1550,
                  "T11 אחריה ה-failover עצמו: CH7 = ALTHOLD, ‏CH8 = 1000, ‏CH3 = THS");
        end
        check(count_ch(t + 800, t + 1500, 6, 1250) > 25 && count_ch(t + 800, t + 1500, 2, 1550) > 25,
              "T11 נשאר ALTHOLD (ההגנה לא השתנתה) עם החזקת המצערת");
        th2_v = 110; #400000; mgh_v = 1; mode_s = "ANGLE"; th2_v = 0; #600000;

        // ── T12: פער של 700ms בשורות האפליקציה ברגע אובדן ה-DOWN ──
        mode_s = "POSHOLD"; th2_v = 0; ths_v = 110; srf_v = 1; #1500000;
        t = $realtime / 1000.0; ctrl_on = 0; srf_v = 0;        // השורות נעצרות, ה-ESP: SRF:0
        #700000; ctrl_on = 1; #1000000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[2] == 1550) i0 = i;
        check(i0 >= 0 && (dfr[i0].t / 1000.0 - t) < 120, "T12 ההקדמה מתחילה גם בלי שורות מהאפליקציה (THS בשורת החיישנים)");
        if (i0 >= 0) begin
            int lead = 0, k;
            for (k = i0; k < dfr.size() && dfr[k].ch[2] == 1550 && dfr[k].ch[7] == 2000; k++) lead++;
            check(lead == 5, $sformatf("T12 5 חבילות הקדמה (%0d)", lead));
        end
        check(count_ch(t + 400, t + 1700, 2, 1550) > 50 && u_dut.armed && !u_dut.rc_hold,
              "T12 אחרי שהשורות חוזרות (0%) ההחזקה נשארת; אין LINK_LOSS (700ms < 1000ms)");

        // ── T13: SRF:0 ואז אובדן קשר מלא - זמן LINK_LOSS לא משתנה ──
        th2_v = 110; #300000; th2_v = 0; ths_v = 110; srf_v = 1; #1500000;
        ctrl_on = 0; srf_v = 0; t = last_ctrl_t / 1000.0;
        #2000000;
        begin
            real last_f = 0;
            foreach (dfr[i]) if (dfr[i].t > last_f) last_f = dfr[i].t;
            check(u_dut.rc_hold == 1'b1, "T13 בלי שורות שליטה: זרם ה-RC נעצר (LINK_LOSS)");
            check((last_f / 1000.0 - t) >= 980 && (last_f / 1000.0 - t) <= 1030,
                  $sformatf("T13 החבילה האחרונה %.0fms אחרי שורת השליטה האחרונה (LINK_LOSS 1000ms, בלי הארכה)", last_f / 1000.0 - t));
        end

        // ── T14 (E1022H, אין שינוי FPGA): מגע מאומת ⇒ ה-ESP שולח THS:-1, האפליקציה נעולה על 0% ──
        //    THS תקין בזמן SURFACE ומתחלף ל-‎-1 (navTouch) 200ms לפני היציאה, כמו בטיסה.
        //    היציאה (אובדן DOWN / OFF יזום - בשניהם ה-ESP שולח SRF:0) חייבת להיות מיידית, בלי הקדמה ובלי החזקה.
        line("CMD:DISARM"); #300000; ctrl_on = 1; stream_on = 1;
        line("CMD:CHECK"); #200000; line("CMD:ARM"); #300000;
        check(u_dut.armed == 1'b1, "T14 armed");
        mode_s = "POSHOLD"; mgh_v = 1; th2_v = 0; ths_v = 110; srf_v = 1; #1500000;
        check(count_ch($realtime / 1000.0 - 700, $realtime / 1000.0, 7, 2000) > 20, "T14 לפני: SURFACE פעיל, THS 110");
        ths_v = -1; #200000;                                    // ה-ESP: navTouch ⇒ THS:-1
        t = $realtime / 1000.0; srf_v = 0;                      // ה-ESP: SRF:0
        #6000000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[7] == 1000) i0 = i;
        check(i0 >= 0 && (dfr[i0].t / 1000.0 - t) < 120, "T14 CH8 = 1000 מיד (בלי 5 חבילות הקדמה)");
        if (i0 >= 0) check(run_len(i0, 6, 1250) == 10, "T14 10 חבילות ALTHOLD (הכניסה מחדש, כמו F1022A)");
        check(count_ch(t, t + 6000, 2, 1550) == 0, "T14 CH3 אף פעם לא THS (1550) אחרי היציאה");
        check(count_ch(t, t + 6000, 2, 1000) > 250, "T14 CH3 = 1000 (0% של האפליקציה) לאורך 6 שנ' עד הניתוק");
        check(u_dut.g_surf.srf_hold == 1'b0, "T14 אין החזקה פעילה");

        // ═══ E1022H2: SRF:0 ו-THS:-1 מופיעים לראשונה **באותה שורה** ═══
        //   השורה עוברת דרך ה-UART וה-cmd_parser האמיתיים; כל שדה נקלט ברגע שהוא מפוענח.
        // ── T15: הסדר הישן (E1022H: SRF לפני THS) - משחזר את הממצא: החזקת THS נוצרת ──
        th2_v = 110; #300000; th2_v = 0;
        ths_first = 0; ths_v = 110; srf_v = 1; #1500000;
        check(count_ch($realtime / 1000.0 - 700, $realtime / 1000.0, 7, 2000) > 20, "T15 לפני: SURFACE פעיל, THS 110");
        t = $realtime / 1000.0; srf_v = 0; ths_v = -1;     // אותה שורה
        #3000000;
        check(count_ch(t + 1000, t + 3000, 2, 1550) > 80,
              "T15 E1022H (SRF:0 THS:-1): ה-FPGA נכנס להחזקה עם ה-THS הישן ונשאר בה (משחזר את הבאג)");
        th2_v = 110; #300000; th2_v = 0; #300000;
        // ── T16: הסדר החדש (E1022H2: THS לפני SRF), אותה שורה ⇒ יציאה מיידית ──
        ths_first = 1; ths_v = 110; srf_v = 1; #1500000;
        check(count_ch($realtime / 1000.0 - 700, $realtime / 1000.0, 7, 2000) > 20, "T16 לפני: SURFACE פעיל, THS 110");
        t = $realtime / 1000.0; srf_v = 0; ths_v = -1;     // אותה שורה: "THS:-1 SRF:0"
        #3000000;
        i0 = -1;
        foreach (dfr[i]) if (i0 < 0 && dfr[i].t >= t * 1000 && dfr[i].ch[7] == 1000) i0 = i;
        check(i0 >= 0 && (dfr[i0].t / 1000.0 - t) < 120, "T16 THS:-1 SRF:0 באותה שורה ⇒ CH8 = 1000 מיד, בלי הקדמה");
        if (i0 >= 0) check(run_len(i0, 6, 1250) == 10, "T16 10 חבילות ALTHOLD");
        check(count_ch(t, t + 3000, 2, 1550) == 0 && count_ch(t, t + 3000, 2, 1000) > 120 && u_dut.g_surf.srf_hold == 1'b0,
              "T16 CH3 = 1000 (0% של האפליקציה) כל הזמן, אין החזקה");
        // ── T17: היציאה התחילה עם THS תקין, ו-navTouch (THS:-1) מגיע 300ms אחר כך ──
        //   האפליקציה אינה יכולה לנעול אחרי היציאה (הנעילה דורשת SURFACE פעיל) ושולחת את ערך
        //   המסירה = THS בדיוק (center.coerceIn(20%..90%)), ולכן ההחזקה משתחררת ב-app_match.
        ths_v = 110; srf_v = 1; #1500000;
        t = $realtime / 1000.0; srf_v = 0;                  // יציאה עם THS 110
        th2_v = 110;                                                                // האפליקציה: ערך המסירה
        #300000; ths_v = -1;                                                        // navTouch מאוחר
        #2000000;
        check(count_ch(t, t + 300, 2, 1550) > 10, "T17 היציאה התחילה עם החזקה (THS 110)");
        check(u_dut.g_surf.srf_hold == 1'b0 && count_ch(t + 800, t + 2300, 2, 1550) > 60,
              "T17 ההחזקה משתחררת ב-app_match; CH3 = הערך של האפליקציה (110), לא נתקע");
        t = $realtime / 1000.0; th2_v = 0; #500000;
        check(count_ch(t + 150, t + 500, 2, 1000) > 15, "T17 אחר כך האפליקציה שולטת (0% עובר מיד)");

        $display("TOTAL errors=%0d frames dut=%0d ref=%0d", errors, dfr.size(), rfr.size());
        $finish;
    end
endmodule
