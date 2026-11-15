#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════
//   שלב 0א - תיעוד פסיבי של חיישן ה-DOWN (אבחון נחיתה בלבד)
//
//   ⚠️ פסיבי לגמרי: הקובץ רק **רושם**. אף ערך כאן לא נקרא בחזרה ע"י לוגיקת
//      הלידאר, מכונת השלבים, המשימה, ה-FPGA או INAV. אין שאילתות MSP חדשות,
//      אין שינוי בסינון ואין שינוי בפקודות.
//
//   כל ניסיון קריאה של DOWN נרשם, לפני כל עיבוד:
//     U - דגימה שימושית: המרחק הגולמי (mm) וה-status המקורי של VL53L1X,
//         ואחריהם הסיווג והמרחק המסונן שהלוגיקה הקיימת קבעה ממנה
//     I - status 10 (פסיקת סנכרון, אין מדידה) - נזרקת כמו היום
//     R / D / C - כשל I2C: DATA_READY / RESULT / CLEAR_IRQ
//
//   המאגר מתרוקן לתוך חבילת הטלמטריה (10Hz) בשדה DLG, עם עוגן זמן EMS
//   (millis של ה-ESP ברגע השליחה). כך אף מדידה לא הולכת לאיבוד בין החבילות.
//   מאגר מלא (למשל כשאין אפליקציה מחוברת) ⇒ הישנה נדרסת ונספרת ב-DLN.
//   האפליקציה לא מפרשת את השדות - מקליט הטיסה שומר את החבילה כמו שהיא,
//   והניתוח נעשה אחרי הטיסה.
// ═══════════════════════════════════════════════════════════
#define DWNLOG_N  32          // ~1.7s בקצב המדידה (55ms) - מספיק בהרבה ל-10Hz
// ⚠️ לכל היותר 8 רשומות בחבילה (~180 תווים): החבילה כולה נשארת מתחת ל-1500 בתים
//    (מאגר הקליטה של האפליקציה). השאר נשלחות בחבילה הבאה - 80 רשומות בשנייה
//    מול ~18 שנוצרות, כך שהמאגר מתרוקן מהר גם אחרי הפסקה.
#define DWNLOG_MAX_PER_PKT  8

struct DwnLogEntry {
    uint32_t ms;              // millis() של ה-ESP ברגע הקריאה
    uint16_t raw;             // mm, כפי שהחיישן החזיר (0 בכשל I2C)
    uint8_t  st;              // status המקורי של VL53L1X (255 בכשל I2C)
    char     ev;              // U / I / R / D / C
    uint8_t  cls;             // המצב שהלוגיקה הקיימת קבעה (LID_*) אחרי הקריאה
    int16_t  cm;              // המרחק המסונן שהלוגיקה הקיימת קבעה
};

static DwnLogEntry dwnLog[DWNLOG_N];
static uint8_t     dwnLogHead    = 0;     // המקום הבא לכתיבה
static uint8_t     dwnLogCount   = 0;
static uint32_t    dwnLogTotal   = 0;     // כל הרשומות מאז ההדלקה
static uint32_t    dwnLogDropped = 0;     // רשומות שנדרסו לפני שנשלחו

static inline void dwnLogPush(uint32_t ms, uint16_t raw, uint8_t st, char ev,
                              uint8_t cls, int cm) {
    DwnLogEntry& e = dwnLog[dwnLogHead];
    e.ms  = ms;
    e.raw = raw;
    e.st  = st;
    e.ev  = ev;
    e.cls = cls;
    e.cm  = (int16_t)cm;
    dwnLogHead = (uint8_t)((dwnLogHead + 1) % DWNLOG_N);
    if (dwnLogCount < DWNLOG_N) dwnLogCount++;
    else                        dwnLogDropped++;    // הישנה ביותר נדרסה
    dwnLogTotal++;
}

// שדות הטלמטריה: " EMS:<ms> DLN:<total>,<dropped> DLG:<רשומות>"
// כל רשומה: "<ms לפני EMS>,<raw mm>,<status>,<ev>,<cls>,<cm>", מופרדות ב-';'.
// ⚠️ בלי רווחים בתוך השדה - האפליקציה מפצלת חבילה לפי רווח.
String dwnLogTake(uint32_t baseMs) {
    String s = " EMS:" + String(baseMs) +
               " DLN:" + String(dwnLogTotal) + "," + String(dwnLogDropped);
    if (dwnLogCount == 0) return s;
    s += " DLG:";
    uint8_t idx = (uint8_t)((dwnLogHead + DWNLOG_N - dwnLogCount) % DWNLOG_N);
    uint8_t n   = (dwnLogCount < DWNLOG_MAX_PER_PKT) ? dwnLogCount : DWNLOG_MAX_PER_PKT;
    for (uint8_t k = 0; k < n; k++) {
        const DwnLogEntry& e = dwnLog[idx];
        if (k) s += ';';
        s += String((long)(baseMs - e.ms)) + ',' + String(e.raw) + ',' + String(e.st) + ',' +
             e.ev + ',' + String(e.cls) + ',' + String(e.cm);
        idx = (uint8_t)((idx + 1) % DWNLOG_N);
    }
    dwnLogCount -= n;                     // הוותיקות נשלחו; החדשות נשארות לחבילה הבאה
    return s;
}
