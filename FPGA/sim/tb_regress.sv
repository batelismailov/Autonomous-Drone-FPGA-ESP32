`timescale 1us/1ns
// ═══════════════════════════════════════════════════════════
//   tb_regress.sv - בדיקת רגרסיה: המשימה כבויה ⇒ אותם פלטים בדיוק
//
//   שלושה עותקים של drone_top מקבלים את אותו זרם בתים בדיוק על esp_rx:
//     old   - הגרסה שלפני ה-Mission FSM (sim/drone_top_old.v)
//     new0  - הגרסה החדשה עם MISSION_ENABLE=0
//     new1  - הגרסה החדשה עם MISSION_ENABLE=1, בלי START שמצליח
//   ובכל מחזור שעון משווים את fc_tx (זרם ה-RC/AUX), esp_tx ו-led.
//
//   שלב A: תעבורה אקראית - שורות שליטה, חיישנים, פקודות, זבל, שורות
//          משימה ו-MSTART / MCROSS / MSTOP שנדחים (BTS:2 ⇒ בדיקת הסוללה נכשלת).
//          old = new0 = new1 בכל שלושת הפלטים, בכל מחזור.
//   שלב B: אותו דבר + CMD:MSTAT. fc_tx ו-led זהים בכל מחזור; ב-esp_tx של
//          new1 מופיעות שורות MST, ושורות ה-STATUS שלו זהות לשל old ולא נפגעות.
//   שלב C: בקרה שלילית - START אמיתי ב-new1 (באוויר, אחרי שורת MSE). fc_tx חייב להשתנות, אחרת
//          ההשוואה לא בודקת כלום.
//
//   הרצה (Verilator 5, מתוך FPGA-Drone):
//     $ verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style
//         --default-language 1364-2005 +1800-2017ext+sv --top-module tb_regress
//         sim/tb_regress.sv sim/drone_top_old.v drone_top.v uart_rx.v uart_tx.v
//         cmd_parser.v msp_builder.v flight_fsm.v mission_rx.v mission_fsm.v
//         mission_tx.v -o tbr  &&  ./obj_dir/tbr
// ═══════════════════════════════════════════════════════════
module tb_regress;
    localparam integer CLK  = 1_000_000;
    localparam integer BAUD = 100_000;
    localparam integer BIT  = CLK / BAUD;          // 10 מחזורים = 10µs

    reg clk = 1'b0;
    always #0.5 clk = ~clk;

    reg  esp_rx = 1'b1;
    wire fc_rx  = 1'b1;
    wire o_esp_tx, o_fc_tx, n0_esp_tx, n0_fc_tx, n1_esp_tx, n1_fc_tx;
    wire [2:0] o_led, n0_led, n1_led;

    drone_top_old #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD)) u_old (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(o_esp_tx), .fc_tx(o_fc_tx), .fc_rx(fc_rx), .led(o_led));
    drone_top #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD), .MISSION_ENABLE(0)) u_new0 (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(n0_esp_tx), .fc_tx(n0_fc_tx), .fc_rx(fc_rx), .led(n0_led));
    drone_top #(.CLK_FREQ(CLK), .BAUD_RATE(BAUD), .MISSION_ENABLE(1)) u_new1 (
        .clk(clk), .esp_rx(esp_rx), .esp_tx(n1_esp_tx), .fc_tx(n1_fc_tx), .fc_rx(fc_rx), .led(n1_led));

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

    // ── מקלט UART לבדיקת השורות של esp_tx ──
    string o_lines[$], n1_lines[$];
    task automatic rx_line(input bit which);   // 0 = old, 1 = new1
        string cur = "";
        forever begin
            byte unsigned b;
            if (which == 0) @(negedge o_esp_tx); else @(negedge n1_esp_tx);
            #(BIT * 1.5);
            for (int i = 0; i < 8; i++) begin
                b[i] = (which == 0) ? o_esp_tx : n1_esp_tx;
                #(BIT);
            end
            if (b == 8'h0A) begin
                if (which == 0) o_lines.push_back(cur); else n1_lines.push_back(cur);
                cur = "";
            end else cur = {cur, $sformatf("%c", b)};
        end
    endtask
    initial rx_line(0);
    initial rx_line(1);

    // ── השוואה מחזור-מחזור ──
    bit cmp_esp = 1'b1;                 // בשלב B esp_tx של new1 משתנה בכוונה
    longint cyc = 0, d_n0 = 0, d_n1_fc = 0, d_n1_led = 0, d_n1_esp = 0;
    longint fc_edges = 0;
    always @(posedge clk) begin
        cyc++;
        if (o_fc_tx != n0_fc_tx || o_esp_tx != n0_esp_tx || o_led != n0_led) d_n0++;
        if (o_fc_tx != n1_fc_tx)  d_n1_fc++;
        if (o_led   != n1_led)    d_n1_led++;
        if (cmp_esp && o_esp_tx != n1_esp_tx) d_n1_esp++;
    end
    reg prev_fc = 1'b1;
    always @(posedge clk) begin prev_fc <= o_fc_tx; if (prev_fc != o_fc_tx) fc_edges++; end

    // האם ה-Mission FSM של new1 היה פעיל אי פעם
    bit seen_active = 1'b0;
    always @(posedge clk) if (u_new1.g_mission.u_mfsm.m_active) seen_active <= 1'b1;

    int errors = 0;
    task automatic check(bit cond, string msg);
        if (cond) $display("[%8.1f ms] PASS  %s", $realtime / 1000.0, msg);
        else begin $display("[%8.1f ms] FAIL  %s", $realtime / 1000.0, msg); errors++; end
    endtask

    // ── מחוללי תעבורה אקראית ──
    // ⚠️ בלי MODE:WP: ב-new1 בקשת WP היא "משימה נבחרה" (ANGLE על הקרקע / POSHOLD
    //    באוויר) ולא WP ידני - הבדל מכוון מול old, ונבדק ב-tb_mission (תרחיש 20, 25).
    string modes[5] = '{"ANGLE", "ALTHOLD", "POSHOLD", "RTH", "MAN"};
    string cmds[9]  = '{"CMD:CHECK", "CMD:ARM", "CMD:DISARM", "CMD:PING", "CMD:ABORT",
                        "CMD:LAND", "CMD:MSTART", "CMD:MCROSS", "CMD:MSTOP"};

    function automatic int rnd(int lo, int hi);
        return lo + int'($urandom % (hi - lo + 1));
    endfunction

    task automatic rnd_ctrl();
        string m = modes[rnd(0, 4)];
        int t = rnd(0, 100);
        // RPL:0 - הגבלת Roll/Pitch כבויה ב-new1, כדי שזרם ה-RC יושווה ל-old.
        //   (ההגבלה עצמה נבדקת ב-tb_mission, תרחיש 25.)
        line($sformatf("THR:%0d THH:%0d YAW:%0d PCH:%0d RLL:%0d MODE:%s DSC:%0d PHA:%0d TH2:%0d RPL:0",
                       t, 2*t, rnd(-120, 120), rnd(-120, 120), rnd(-120, 120), m,
                       rnd(0, 1), rnd(0, 3), rnd(0, 230)));
    endtask
    int fseq = 0, dseq = 0;
    task automatic rnd_sens(bit with_cls);
        fseq = (fseq + rnd(0, 1)) % 256;  dseq = (dseq + rnd(0, 1)) % 256;
        if (with_cls)
            line($sformatf("FWD:%0d FOK:%0d DWN:%0d DOK:%0d FSEQ:%0d DSEQ:%0d PHS:%0d FWA:%0d MGH:%0d FRC:%0d DRC:%0d",
                           rnd(0, 450), rnd(0, 6), rnd(0, 450), rnd(0, 6), fseq, dseq,
                           rnd(0, 3), rnd(0, 1), rnd(0, 3), rnd(0, 7), rnd(0, 7)));
        else
            line($sformatf("FWD:%0d FOK:%0d DWN:%0d DOK:%0d FSEQ:%0d DSEQ:%0d PHS:%0d FWA:%0d MGH:%0d",
                           rnd(0, 450), rnd(0, 6), rnd(0, 450), rnd(0, 6), fseq, dseq,
                           rnd(0, 3), rnd(0, 1), rnd(0, 3)));
    endtask
    int nq = 0;
    task automatic rnd_fast();
        nq++;
        line($sformatf("NPX:%0d NPY:%0d NPZ:%0d NVX:%0d NVY:%0d NVZ:%0d NPQ:%0d NMD:%0d NST:%0d NWP:%0d NER:%0d NNQ:%0d NFM:%0d NAR:%0d NFQ:%0d GFX:%0d GSA:%0d GEP:%0d GSQ:%0d",
                       rnd(-40000, 40000), rnd(-40000, 40000), rnd(-500, 500), rnd(-300, 300), rnd(-300, 300),
                       rnd(-300, 300), nq % 256, rnd(0, 15), rnd(0, 15), rnd(0, 9), rnd(0, 11), nq % 256,
                       rnd(0, 2047), rnd(0, 1), nq % 256, rnd(0, 2), rnd(0, 20), rnd(0, 1200), nq % 256));
    endtask
    task automatic rnd_slow();
        nq++;
        // BTS:2 תמיד: בדיקת הסוללה נכשלת ⇒ אף START / MCROSS לא מתקבל בשלבים A-B
        line($sformatf("MIS:%0d CAT:%0d CUC:%0d CRS:%0d CAS:%0d BTS:2 SLQ:%0d",
                       rnd(0, 9), rnd(0, 3), rnd(0, 3), rnd(0, 3), rnd(0, 2500), nq % 256));
    endtask
    task automatic rnd_garbage();
        string g = "";
        int n = rnd(1, 30);
        for (int i = 0; i < n; i++) g = {g, $sformatf("%c", rnd(32, 126))};
        line(g);
    endtask

    // תעבורה אקראית למשך ms אלפיות שנייה. stat=1 מוסיף CMD:MSTAT
    task automatic traffic(int ms, bit stat);
        int t = 0;
        while (t < ms) begin
            int k = rnd(0, 99);
            if      (k < 30) rnd_ctrl();
            else if (k < 55) rnd_sens(rnd(0, 1));
            else if (k < 70) rnd_fast();
            else if (k < 75) rnd_slow();
            else if (k < 85) line(cmds[rnd(0, 8)]);
            else if (k < 90) rnd_garbage();
            else if (stat)   line("CMD:MSTAT");
            else             line("CMD:PING");
            #(rnd(2, 25) * 1000);
            t += 15;
            while (txq.size() > 400) #1000;     // לא להציף את התור
        end
    endtask

    // ── תרחיש שבו START אמיתי עובר ב-new1 (שלב C) ──
    task automatic start_ok_world(int ms);
        int t = 0;
        while (t < ms) begin
            line("THR:65 THH:131 YAW:0 PCH:0 RLL:0 MODE:POSHOLD DSC:0 PHA:3 TH2:131 RPL:0");
            fseq++; dseq++;
            line($sformatf("FWD:130 FOK:2 DWN:60 DOK:1 FSEQ:%0d DSEQ:%0d PHS:3 FWA:1 MGH:1 FRC:2 DRC:1", fseq % 256, dseq % 256));
            nq++;
            line($sformatf("NPX:0 NPY:0 NPZ:60 NVX:0 NVY:0 NVZ:0 NPQ:%0d NMD:1 NST:3 NWP:1 NER:0 NNQ:%0d NFM:49 NAR:1 NFQ:%0d GFX:2 GSA:14 GEP:120 GSQ:%0d",
                           nq % 256, nq % 256, nq % 256, nq % 256));
            line($sformatf("MIS:1 CAT:0 CUC:1 CRS:1 CAS:50 BTS:0 SLQ:%0d CTM:65 MAL:100", nq % 256));
            #50000;
            t += 50;
        end
    endtask

    initial begin
        void'($urandom(32'd20261005));
        #100000;   // POR של שלושת העותקים

        // ═══ שלב A ═══
        line("CMD:CHECK"); line("CMD:ARM");
        traffic(60000, 1'b0);
        while (txq.size() > 0) #1000;
        #20000;
        check(d_n0 == 0, $sformatf("A: old == new(MISSION_ENABLE=0) on fc_tx/esp_tx/led in every cycle (%0d cycles, %0d diffs)", cyc, d_n0));
        check(d_n1_fc == 0 && d_n1_led == 0 && d_n1_esp == 0,
              $sformatf("A: old == new(MISSION_ENABLE=1, never started) on fc_tx/esp_tx/led (%0d/%0d/%0d diffs)",
                        d_n1_fc, d_n1_led, d_n1_esp));
        check(fc_edges > 100000, $sformatf("A: the RC stream really moved (%0d edges on fc_tx)", fc_edges));
        check(!seen_active, $sformatf("A: Mission FSM was never active (st=%0d, reason=%0h)",
                                      u_new1.g_mission.u_mfsm.st, u_new1.g_mission.u_mfsm.reason));

        // ═══ שלב B ═══
        cmp_esp = 1'b0;
        o_lines.delete(); n1_lines.delete();
        traffic(40000, 1'b1);
        while (txq.size() > 0) #1000;
        #20000;
        check(d_n0 == 0, $sformatf("B: old == new0 still identical (%0d diffs)", d_n0));
        check(d_n1_fc == 0 && d_n1_led == 0,
              $sformatf("B: with CMD:MSTAT the RC stream and LEDs stay identical (%0d/%0d diffs)", d_n1_fc, d_n1_led));
        begin
            string o_st[$], n_st[$];
            int n_mst = 0, bad = 0;
            foreach (o_lines[i])  if (o_lines[i].substr(0, 6) == "STATUS:") o_st.push_back(o_lines[i]);
            foreach (n1_lines[i]) begin
                if (n1_lines[i].substr(0, 6) == "STATUS:") n_st.push_back(n1_lines[i]);
                else if (n1_lines[i].substr(0, 3) == "MST:" && n1_lines[i].len() == 55) n_mst++;
                else bad++;
            end
            check(!seen_active, "B: Mission FSM was never active");
            check(n_mst > 50, $sformatf("B: new1 answered CMD:MSTAT with well-formed MST lines (%0d)", n_mst));
            check(bad == 0, $sformatf("B: no garbled line on new1 esp_tx (%0d bad)", bad));
            check(o_st.size() > 20 && o_st.size() == n_st.size(),
                  $sformatf("B: same number of STATUS replies (old %0d, new1 %0d)", o_st.size(), n_st.size()));
            begin
                int same = 1;
                foreach (o_st[i]) if (i < n_st.size() && o_st[i] != n_st[i]) same = 0;
                check(same == 1, "B: every STATUS reply of new1 is identical to old (no interleaving with MST)");
            end
        end

        // ═══ שלב C - בקרה שלילית ═══
        d_n1_fc = 0;
        line("CMD:DISARM"); line("CMD:DISARM");
        // סט ההגדרות - רק כשה-FPGA מנוטרל. 8·100+7·150+6·70+5·70+4·400+3·50+2·2+0 = 4374
        line("MSE:100,150,70,70,400,50,2,0,4374");
        start_ok_world(300);
        line("CMD:CHECK"); line("CMD:ARM");
        start_ok_world(2000);
        line("CMD:MSTART");
        start_ok_world(3000);
        check(seen_active, $sformatf("C: a valid START is accepted by new1 (now st=%0d, reason=%0h)",
                                     u_new1.g_mission.u_mfsm.st, u_new1.g_mission.u_mfsm.reason));
        check(d_n1_fc > 0, $sformatf("C: ...and then fc_tx differs from old (%0d cycles) - the comparison has teeth", d_n1_fc));
        check(d_n0 == 0, $sformatf("C: new0 still identical to old (%0d diffs)", d_n0));

        $display("=== DONE  errors=%0d ===", errors);
        $finish;
    end
endmodule
