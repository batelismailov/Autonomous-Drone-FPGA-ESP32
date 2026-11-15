#pragma once
#include <HardwareSerial.h>
#include "config.h"

HardwareSerial SerialFPGA(1);   // UART1 של הESP → GPIO17/18

// מצב ה-FSM כפי שדווח אחרון:  0..5, או -1 כשאין קשר
int8_t fpgaState = -1;

// אבחון מתשובת ה-STATUS:
//   fpgaFlags  ספרת hex: bit3 זרם ה-RC עצור · bit2 דגימת FWD טרייה
//                        bit1 דגימת DWN טרייה · bit0 פקודות שליטה טריות
//   fpgaEnd    סיבת העצירה האחרונה: '-' אין · 'L' אין פקודות שליטה · 'E' ה-ESP מת
//
// ⚠️ הדגלים האלה הם של הארכיטקטורה הנוכחית. אין יותר DOWN_ACTIVE, אין נעילת
//    "DWN היה נמוך", אין גובה ואין זיהוי נגיעה או גיבוי - הנחיתה היא באחריות INAV.
char fpgaFlags = '?';
char fpgaEnd   = '?';
// גרסת ה-FPGA מסוף תשובת STATUS (":F1018A"). "-" = לא דווחה (FPGA ישן או אין תשובה)
String fpgaVer = "-";

void initFPGA() {
#if MISSION_ENABLE
    // שורות המשימה מכפילות בערך את התעבורה אל ה-FPGA (עדיין פחות מחצי מהקיבולת).
    // מאגר שידור, כדי שכתיבה של שורה ארוכה לא תעצור את הלולאה עד שה-FIFO יתרוקן.
    SerialFPGA.setTxBufferSize(512);
#endif
    SerialFPGA.begin(BAUD_FPGA, SERIAL_8N1, UART_FPGA_RX, UART_FPGA_TX);
    DBG_PRINTLN("[FPGA] UART1 ready");
}

void fpgaSend(const String& msg) {
    SerialFPGA.println(msg);       // מוסיף \r\n - cmd_parser מטפל בשניהם
}

#if MISSION_ENABLE
// ═══ שורות MST של ה-Mission FSM ═══
//   ה-FPGA עונה על CMD:MSTAT בשורה "MST:..." שמגיעה כשה-ESP לא ממתין לשום
//   תשובה. לכן הקליטה כאן אינה חוסמת: fpgaPollRx נקרא בכל סבב של הלולאה,
//   ושורה שהתחילה להגיע נשמרת ב-fpgaRxLine עד שה-LF שלה מגיע.
//   missionTakeLine (mission.h) מחזיר true כשהשורה היא MST, אחרי שפוענחה.
//   ⚠️ ה-FPGA לעולם לא משלב את שורת ה-MST בתוך תשובת STATUS - השורות שלמות.
bool missionTakeLine(const String& line);
static String fpgaRxLine;

void fpgaPollRx() {
    while (SerialFPGA.available()) {
        char c = SerialFPGA.read();
        if (c == '\n') {
            missionTakeLine(fpgaRxLine);       // שורה שאינה MST (תשובה מאוחרת) נזרקת
            fpgaRxLine = "";
        } else if (fpgaRxLine.length() < 80) {
            fpgaRxLine += c;
        }
    }
}
#endif

String fpgaRead(int timeoutMs = FPGA_TIMEOUT_MS) {
    unsigned long t0 = millis();
#if MISSION_ENABLE
    String resp = fpgaRxLine;          // שורה שכבר התחילה להגיע
    fpgaRxLine = "";
#else
    String resp = "";
#endif
    // ממשיכים גם אחרי פסק הזמן כל עוד יש בתים - תשובה שהגיעה בזמן הטיפול בפקודות לא הולכת לאיבוד
    while (millis() - t0 < timeoutMs || SerialFPGA.available()) {
        if (!SerialFPGA.available()) ioIdleHook();     // שליטה / חירום מהאפליקציה בזמן ההמתנה
        while (SerialFPGA.available()) {
            char c = SerialFPGA.read();
#if MISSION_ENABLE
            // שורת MST אינה התשובה שמחכים לה: נקלטת כאן, וממשיכים לחכות
            if (c == '\n' && missionTakeLine(resp)) { resp = ""; continue; }
#endif
            if (c == '\n') { resp.trim(); return resp; }
            resp += c;
        }
    }
    resp.trim();
    return resp;
}

// ─── פענוח "STATUS:OK", "STATUS:OK:<n>" או "STATUS:OK:<n>:<F><R>" ─────
// תואם לאחור: גם תשובה בלי ספרה או בלי אבחון נחשבת תקינה
bool parseFpgaResp(const String& r) {
    if (!r.startsWith("STATUS:OK")) return false;
    if (r.length() >= 11 && r.charAt(9) == ':') {
        char c = r.charAt(10);
        if (c >= '0' && c <= '9') fpgaState = c - '0';
    }
    if (r.length() >= 14 && r.charAt(11) == ':') {
        fpgaFlags = r.charAt(12);
        fpgaEnd   = r.charAt(13);
    }
    // ":<גרסה>" אחרי האבחון - אותיות וספרות בלבד, עד 8 תווים
    if (r.length() >= 16 && r.charAt(14) == ':') {
        String v = "";
        for (unsigned int i = 15; i < r.length() && v.length() < 8; i++) {
            char c = r.charAt(i);
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) v += c;
            else break;
        }
        if (v.length() > 0) fpgaVer = v;
    }
    return true;
}

// ─── בדיקת חיות - לא משנה את מצב ה-FSM ────────
bool pingFPGA() {
    fpgaSend("CMD:PING");
    return parseFpgaResp(fpgaRead(FPGA_TIMEOUT_MS));
}

// ─── פתיחת READY - משנה מצב! ──────────────────
bool startFPGA() {
    fpgaSend("CMD:CHECK");
    return parseFpgaResp(fpgaRead(FPGA_TIMEOUT_MS));
}
