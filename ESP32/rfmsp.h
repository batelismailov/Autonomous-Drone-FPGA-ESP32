#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "surface.h"     // E1022B: כל מסגרת שנכתבה בפועל מעדכנת את תקינות ה-DOWN ל-SURFACE

// ═══════════════════════════════════════════════════════════
//   שלב 0ב - מדידת DOWN ל-INAV כ-MSP Rangefinder
//
//   MSP2_SENSOR_RANGEFINDER (0x1F01), MSP v2, בלי תשובה מ-INAV:
//     מטען 5 בתים: quality (uint8) · distanceMm (int32 LE). מרחק שלילי = מחוץ לטווח.
//     מסגרת: "$X<" flag fn(2) size(2) payload crc8-dvb-s2 = 14 בתים.
//
//   ⚠️ INAV משתמש בזה רק כש-rangefinder_hardware = MSP. ב-NONE ההודעה נקלטת ונזרקת.
//      שלב 0ב: בדיקה על הקרקע בלבד, בלי פרופלורים, ובסוף חזרה ל-NONE.
//
//   ═══ מה נשלח ═══
//     LID_VALID                          → המרחק המסונן (אותו ערך שהמערכת משתמשת בו), mm
//     LID_NO_RETURN (status 4 / 2)       → -1 (מחוץ לטווח)
//     LID_TOO_CLOSE + דגימת קרקע גולמית טרייה → 10mm = 1cm (E1021E, ראו למטה)
//     LID_TOO_CLOSE בלי דגימה טרייה (נעילה) → -1. נתון חסר לעולם אינו "גובה 0".
//     UNCERTAIN / WEAK / wrapped / FAULT,
//     status 10, כשל I2C, נתון ישן       → לא נשלח כלום (INAV יראה שאין נתון חדש)
//
//   ═══ למה זה לא מעכב שום דבר ═══
//     · כל דגימה חדשה נרשמת פעם אחת (rfMspNote, מתוך updateLidars). דגימה שלא נשלחה
//       עד שהגיעה חדשה - נדרסת ונספרת. ערך ישן לא נשלח פעמיים.
//     · השליחה (rfMspService) רק מה-loop, אחרי שאילתות ה-MSP - אף פעם לא באמצע המתנה
//       לתשובה, ולא מתוך ioIdleHook. בלי המתנה, בלי תשובה ובלי שאילתה חדשה.
//     · availableForWrite < 14 ⇒ לא כותבים (כתיבה לא תחסום את הלולאה). נספר, ומנסים
//       בסבב הבא עם הדגימה העדכנית.
//     · שורות הפקודות ל-INAV עוברות מה-FPGA בקו נפרד - הקו הזה הוא רק קו הסקר של ה-ESP.
//
//   אבחון בטלמטריה: " RFM:<sent>,<overwritten>,<no room>,<not sent: unreliable>"
// ═══════════════════════════════════════════════════════════
#define MSP2_SENSOR_RANGEFINDER  0x1F01
#define RF_MSP_FRAME_LEN         14

// ═══ הפרדה מהסקר (E1021B) ═══
//   INAV מעבד פקודת MSP אחת בכל סבב של משימת ה-Serial (100Hz, msp_serial.c: "process one
//   command at a time so as not to block"). שאילתה שנשלחה מיד אחרי מסגרת המרחק חיכתה לסבב
//   נוסף - נמדד: סבב הלולאה של ה-ESP עלה מ-19 ל-25ms (חציון), גם בלי Configurator.
//   לכן אחרי כל מסגרת: אין שאילתת MSP חדשה RF_MSP_POLL_HOLD_MS (סבב Serial מלא + זמן השידור),
//   כך ש-INAV כבר עיבד את המסגרת כשהשאילתה מגיעה.
#define RF_MSP_POLL_HOLD_MS      12

extern HardwareSerial SerialFC;            // msp.h

struct RfPending {
    bool    has;
    int32_t mm;
    uint8_t quality;
};
static RfPending rfPend      = { false, 0, 0 };
static uint32_t  rfSent      = 0;
static uint32_t  rfOverwrite = 0;
static uint32_t  rfNoRoom    = 0;
static uint32_t  rfUnsent    = 0;
static uint32_t      rfHoldUntilMs = 0;
static bool          rfHoldOn      = false;

// true = נשלחה מסגרת מרחק לפני פחות מ-RF_MSP_POLL_HOLD_MS - לא שולחים עכשיו שאילתת MSP
static inline bool rfMspHold() {
#if RF_MSP_ENABLE
    if (rfHoldOn && (int32_t)((uint32_t)millis() - rfHoldUntilMs) < 0) return true;
    rfHoldOn = false;
#endif
    return false;
}

// ═══ E1021E: "קרקע = 1cm" ═══
//   החיישן כ-1cm מעל הרצפה כשהרחפן עומד, מתחת לטווח המינימלי (~4cm): על הקרקע הוא מחזיר
//   0-3mm ⇒ TOO_CLOSE. עד עכשיו נשלח -1, כלומר ל-INAV אין מדידה על הקרקע (ב-SURFACE ⇒ ירידה).
//   הדרייבר של INAV 9.1 (rangefinder_msp.c) מחלק ב-10 ודוחה 0cm ⇒ הערך הקטן ביותר שמתקבל: 10mm.
//   נשלח רק כשה-TOO_CLOSE נתמך בדגימה גולמית של קרקע בתוך RF_MSP_GROUND_FRESH_MS (אותו אות
//   כמו במגע ב-phase.h: tcRawMs, שנקבע רק אחרי שה-TOO_CLOSE התקבל לפי הכללים של E1021C).
//   TOO_CLOSE שנשמר רק מנעילה (NO_RETURN אחרי הקרקע) ⇒ -1 כמו קודם.
//   ⚠️ לניסוי קרקע בלבד (rangefinder_hardware = MSP, בלי חימוש). ב-NONE אין לזה שום השפעה.
#define RF_MSP_GROUND_MM          10
#define RF_MSP_GROUND_FRESH_MS    600

// ═══ E1022A: TOO_CLOSE באוויר בלי הקשר של התקרבות ⇒ לא "קרקע" ═══
//   ב-SITL של INAV 9.1, ב-SURFACE ב-60cm: 3 דגימות שגויות של 10mm ⇒ קפיצה ל-79cm,
//   11 דגימות ⇒ ל-149cm (INAV חושב שהוא על הקרקע ומטפס ליעד).
//   TOO_CLOSE שהתקבל רק בכלל "3 ברצף" (בלי VALID ≤ 40cm לפניו, lidar.h: tcNoCtx) נשלח
//   כ-‎-1 כשהשלב הוא AIRBORNE. ‎-1 ⇒ SURFACE יוצא מיד (surface.h) ⇒ POSHOLD רגיל.
//   על הקרקע (אחרי הדלקה, בלי התקרבות) הכלל לא משתנה - שם 10mm הוא הנכון.
static bool rfAirborne = false;            // מה-.ino: flight.phase == PH_AIRBORNE

// נקרא מ-updateLidars על כל דגימת DOWN שעברה עיבוד. state = LID_*, status = של VL53L1X
//   groundFresh: TOO_CLOSE עם דגימת קרקע גולמית בתוך RF_MSP_GROUND_FRESH_MS
static inline void rfMspNote(uint8_t state, uint8_t status, int distCm, bool groundFresh) {
#if RF_MSP_ENABLE
    int32_t mm;
    uint8_t q;
    if (state == LID_VALID && distCm > 0) {
        mm = (int32_t)distCm * 10;
        q  = 255;
    } else if (state == LID_TOO_CLOSE && groundFresh) {
        mm = RF_MSP_GROUND_MM;
        q  = 255;
    } else if (state == LID_TOO_CLOSE ||
               (state == LID_NO_RETURN &&
                (status == 2 || (status < 32 && (LIDAR_OOR_STATUS_MASK & (1u << status)))))) {
        mm = -1;
        q  = 0;
    } else {
        rfUnsent++;                       // לא אמין - לא שולחים כלום
        return;
    }
    if (rfPend.has) rfOverwrite++;        // הקודמת לא נשלחה - החדשה מחליפה אותה
    rfPend.has     = true;
    rfPend.mm      = mm;
    rfPend.quality = q;
#else
    (void)state; (void)status; (void)distCm; (void)groundFresh;
#endif
}

static inline uint8_t rfCrc8(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

// נקרא מה-loop בלבד, כשאין שאילתת MSP פתוחה
static void rfMspService() {
#if RF_MSP_ENABLE
    if (!rfPend.has) return;
    if (SerialFC.availableForWrite() < RF_MSP_FRAME_LEN) { rfNoRoom++; return; }
    uint8_t f[RF_MSP_FRAME_LEN];
    const uint32_t d = (uint32_t)rfPend.mm;
    f[0]  = '$'; f[1] = 'X'; f[2] = '<';
    f[3]  = 0;                                              // flag
    f[4]  = (uint8_t)(MSP2_SENSOR_RANGEFINDER & 0xFF);
    f[5]  = (uint8_t)(MSP2_SENSOR_RANGEFINDER >> 8);
    f[6]  = 5;  f[7] = 0;                                   // size
    f[8]  = rfPend.quality;
    f[9]  = (uint8_t)(d);
    f[10] = (uint8_t)(d >> 8);
    f[11] = (uint8_t)(d >> 16);
    f[12] = (uint8_t)(d >> 24);
    uint8_t crc = 0;
    for (uint8_t i = 3; i < 13; i++) crc = rfCrc8(crc, f[i]);
    f[13] = crc;
    SerialFC.write(f, RF_MSP_FRAME_LEN);
    surfNoteRf(rfPend.mm);                // E1022B: תקינות ל-SURFACE רק ממה שנכתב בפועל ל-UART
    rfPend.has    = false;
    rfSent++;
    rfHoldUntilMs = (uint32_t)millis() + RF_MSP_POLL_HOLD_MS;
    rfHoldOn      = true;
#endif
}

String rfMspTelem() {
#if RF_MSP_ENABLE
    return " RFM:" + String(rfSent) + "," + String(rfOverwrite) + "," + String(rfNoRoom) + "," + String(rfUnsent);
#else
    return "";
#endif
}
