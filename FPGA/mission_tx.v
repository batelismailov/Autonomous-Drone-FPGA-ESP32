// ═══════════════════════════════════════════════════════════
//   mission_tx.v - תשובת מצב המשימה ל-ESP32
//
//   משיב רק על CMD:MSTAT - בלי בקשה הוא לא משדר כלום.
//
//   "MST:SS:RR:CCCC:DDD:TTT:WW:G:VVV:AA:PPP:LLL:BB:KKKK:F:NN\n"  (56 בתים, hex)
//     SS    מצב ה-Mission FSM
//     RR    סיבת העצירה / הדחייה האחרונה (00 = אין)
//     CCCC  16 ביטי SELF_CHECK, חיים
//     DDD   FWD ברגע זיהוי המכשול, cm (3FF = אין)
//     TTT   FWD אחרי העצירה, cm (3FF = אין)
//     WW    ה-WP שלפני העקיפה
//     G     שלב הבדיקה (הגדרה בזמן ריצה)
//     VVV   מהירות אופקית ברגע הזיהוי, cm/s
//     AA    זמן עד ש-INAV אישר POSHOLD אחרי הזיהוי, 10ms (FF = לא אושר)
//     PPP   זמן עד עצירה יציבה, 10ms
//     LLL   תזוזה מנקודת הזיהוי עד העצירה, cm
//     BB    חזרה לאחור אחרי העצירה, cm
//     KKKK  ה-SCK של סט ההגדרות שהוחל
//     F     bit0 סט תקף · bit1 DOWN ראה קרקע · bit2 המראה מהקרקע · bit3 מצערת מסונכרנת
//     NN    נייטרל המצערת, חצאי אחוז
//
//   ═══ שיתוף ה-UART עם flight_fsm ═══
//   השורה מתחילה רק כשהמשדר פנוי וכש-flight_fsm אינו באמצע תשובה. בזמן
//   שהשורה יוצאת, drone_top מציג ל-flight_fsm את המשדר כתפוס - כך שתי
//   השורות לעולם לא מתערבבות.
// ═══════════════════════════════════════════════════════════
module mission_tx (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        req,          // CMD:MSTAT
    input  wire [4:0]  st,
    input  wire [7:0]  rsn,
    input  wire [15:0] chk,
    input  wire [9:0]  det,
    input  wire [9:0]  stp,
    input  wire [5:0]  wp,
    input  wire [3:0]  stage,
    input  wire [9:0]  dv,
    input  wire [7:0]  ta,
    input  wire [9:0]  tsp,
    input  wire [9:0]  trv,
    input  wire [7:0]  rbk,
    input  wire [15:0] sck,
    input  wire [3:0]  flg,
    input  wire [7:0]  ntr,

    input  wire        fsm_cmd,      // cmd_check | cmd_ping - flight_fsm עומד לענות
    input  wire [7:0]  fsm_tx_data,
    input  wire        fsm_tx_send,
    input  wire        uart_busy,

    output reg  [7:0]  tx_data,
    output reg         tx_send,
    output reg         active        // 1 מתחילת השורה ועד סוף ה-LF שלה
);

    localparam [5:0] LAST = 6'd55;

    // flight_fsm באמצע תשובה: מהפקודה ועד שה-LF שלה נשלח
    reg fsm_busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                     fsm_busy <= 1'b0;
        else if (fsm_cmd)                               fsm_busy <= 1'b1;
        else if (fsm_tx_send && fsm_tx_data == 8'h0A)   fsm_busy <= 1'b0;
    end

    // הספרות נקראות ישירות מהקלטים, ספרה אחר ספרה (בלי עותק רשום של כל השורה).
    // הערכים משתנים לכל היותר ב-10-20Hz, ושורה יוצאת ב-5ms - ולכן שורה מעורבת
    // נדירה, וגם אז כל שדה בה הוא ערך אמיתי (ה-ESP דוגם את השורה ב-5Hz).
    wire [147:0] bus = {{3'd0, st}, rsn, chk, {2'd0, det}, {2'd0, stp}, {2'd0, wp}, stage, {2'd0, dv}, ta, {2'd0, tsp}, {2'd0, trv}, rbk, sck, flg, ntr};
    reg  [5:0]  idx;            // תו בשורה
    reg  [5:0]  dig;            // ספרת hex הבאה
    reg         pend;

    reg        is_fix;
    reg [7:0]  fixed_ch;
    always @* begin
        is_fix   = 1'b1;
        fixed_ch = 8'h0A;
        case (idx)
            6'd0 : fixed_ch = "M";
            6'd1 : fixed_ch = "S";
            6'd2 : fixed_ch = "T";
            6'd3, 6'd6, 6'd9, 6'd14, 6'd18, 6'd22, 6'd25, 6'd27, 6'd31, 6'd34, 6'd38, 6'd42, 6'd45, 6'd50, 6'd52: fixed_ch = ":";
            6'd55: fixed_ch = 8'h0A;
            default: is_fix = 1'b0;
        endcase
    end

    reg [3:0] nib;
    always @* begin
        case (dig)
            6'd0: nib = bus[147:144];
            6'd1: nib = bus[143:140];
            6'd2: nib = bus[139:136];
            6'd3: nib = bus[135:132];
            6'd4: nib = bus[131:128];
            6'd5: nib = bus[127:124];
            6'd6: nib = bus[123:120];
            6'd7: nib = bus[119:116];
            6'd8: nib = bus[115:112];
            6'd9: nib = bus[111:108];
            6'd10: nib = bus[107:104];
            6'd11: nib = bus[103:100];
            6'd12: nib = bus[99:96];
            6'd13: nib = bus[95:92];
            6'd14: nib = bus[91:88];
            6'd15: nib = bus[87:84];
            6'd16: nib = bus[83:80];
            6'd17: nib = bus[79:76];
            6'd18: nib = bus[75:72];
            6'd19: nib = bus[71:68];
            6'd20: nib = bus[67:64];
            6'd21: nib = bus[63:60];
            6'd22: nib = bus[59:56];
            6'd23: nib = bus[55:52];
            6'd24: nib = bus[51:48];
            6'd25: nib = bus[47:44];
            6'd26: nib = bus[43:40];
            6'd27: nib = bus[39:36];
            6'd28: nib = bus[35:32];
            6'd29: nib = bus[31:28];
            6'd30: nib = bus[27:24];
            6'd31: nib = bus[23:20];
            6'd32: nib = bus[19:16];
            6'd33: nib = bus[15:12];
            6'd34: nib = bus[11:8];
            6'd35: nib = bus[7:4];
            6'd36: nib = bus[3:0];
            default: nib = 4'd0;
        endcase
    end

    wire [7:0] hex = (nib < 4'd10) ? (8'h30 + {4'd0, nib}) : (8'h37 + {4'd0, nib});
    wire [7:0] cur = is_fix ? fixed_ch : hex;

    localparam T_IDLE = 2'd0, T_LOAD = 2'd1, T_WAIT = 2'd2, T_HOLD = 2'd3;
    reg [1:0] t_st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_st    <= T_IDLE;
            idx     <= 6'd0;
            pend    <= 1'b0;
            active  <= 1'b0;
            tx_data <= 8'd0;
            tx_send <= 1'b0;
            dig     <= 6'd0;
        end else begin
            if (req) pend <= 1'b1;

            case (t_st)
            T_IDLE: begin
                tx_send <= 1'b0;
                if (pend && !fsm_busy && !fsm_cmd && !uart_busy) begin
                    idx    <= 6'd0;
                    dig    <= 6'd0;
                    pend   <= 1'b0;
                    active <= 1'b1;
                    t_st   <= T_LOAD;
                end
            end
            T_LOAD: if (!uart_busy) begin
                    tx_data <= cur;
                    tx_send <= 1'b1;
                    t_st    <= T_WAIT;
                end
            T_WAIT: begin
                    tx_send <= 1'b0;
                    if (uart_busy) t_st <= T_HOLD;
                end
            T_HOLD: if (!uart_busy) begin
                    if (!is_fix) dig <= dig + 6'd1;
                    if (idx == LAST) begin
                        active <= 1'b0;
                        t_st   <= T_IDLE;
                    end else begin
                        idx  <= idx + 6'd1;
                        t_st <= T_LOAD;
                    end
                end
            default: t_st <= T_IDLE;
            endcase
        end
    end

endmodule
