#pragma once
#include <HardwareSerial.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════
//   MSP - טלמטריה מ-MATEK (UART1)
//   ⚠️ שום פקודת שליטה לא נשלחת מכאן. הערוצים נבנים ב-FPGA בלבד.
//      הכתיבה היחידה בקובץ היא MSP_SET_RTC - כיוון השעון של בקר הטיסה,
//      שאינו נוגע בערוצים, במצבי הטיסה או בחימוש.
//
//   הסקר מתוזמן לפי חותמת זמן לכל סוג שאילתה (config.h), ולא בתור אחיד:
//   סטטוס ~8Hz, סוללה ~3Hz, גובה ~4Hz, ונתוני התצוגה בקצב חבילת התצוגה
//   (5Hz): ATTITUDE, GPS, GPSSTATISTICS ו-RAW_IMU (MAG).
//   MSP_BOXIDS ו-nav_mc_althold_throttle נקראים רק כשהם עוד לא ידועים -
//   כלומר בהתחברות ואחרי אתחול של הקישור.
//
//   ⚠️ MSP_ALTITUDE נשאר לשימוש פנימי בלבד (בדיקת סבירות של חיישן DWN).
//      הגובה והמהירות האנכית לא נשלחים ל-FPGA ולא לאפליקציה.
// ═══════════════════════════════════════════════════════════
#define MSP_RAW_IMU          102       // 9 × int16: ACC XYZ · GYRO XYZ · MAG XYZ - נקרא רק בשביל MAG
#define MSP_RAW_GPS          106
#define MSP_ATTITUDE         108
#define MSP_ALTITUDE         109
#define MSP_GPSSTATISTICS    166       // HDOP / EPH / EPV - תצוגה ואבחון בלבד
#define MSP_BOXIDS           119       // permanentId של כל מצב, לפי סדר הביטים במסכת המצבים
#define MSP_RTC              246       // קריאת השעון: int32 שניות UTC + uint16 מילישניות
#define MSP_SET_RTC          247       // כתיבת השעון, מטען 6 בתים באותו מבנה

#define MSP2_COMMON_SETTING  0x1003    // קריאת הגדרה לפי שם (מטען: שם + '\0')
#define MSP2_INAV_STATUS     0x2000    // חימוש + מצבי טיסה בפועל (במקום MSP_STATUS)

#if MISSION_ENABLE
// ── Mission FSM בלבד - קריאה בלבד, שום כתיבה ל-INAV (msp_protocol.h של INAV 9.1) ──
#define MSP_WP_GETINFO       20        // reserved · max · valid · count
#define MSP_WP               118       // WP אחד לפי מספר (1..count), 21 בתים
#define MSP_NAV_STATUS       121       // mode · state · WP action · WP number · error · heading
#define MSP2_INAV_FULL_LOCAL_POSE 0x2220   // attitude + מיקום ומהירות NEU, cm / cm/s
#endif

#define INAV_ARMING_ARMED    (1UL << 2)   // runtime_config.h: ARMED = (1 << 2)

#define FC_CONFIG_RETRY_MS   1000      // ניסיון חוזר כשהמיפוי / ההגדרה עוד לא ידועים
#define FC_CONFIG_SLOW_MS    10000     // ...ואחרי כמה כישלונות רצופים, כדי לא להציף את ה-UART
#define FC_CONFIG_FAST_TRIES 5

#define RTC_MIN_EPOCH_S      1700000000UL   // נובמבר 2023 - ערך קטן מזה אינו זמן אמיתי
#define RTC_MIN_SET_GAP_MS   2000           // הגבלת קצב, גם אם האפליקציה תשלח שעה בכל חבילה

// ── סוגי שאילתה ──
#define POLL_STATUS  0    // MSP2_INAV_STATUS
#define POLL_ANALOG  1    // בפועל: MSP2_INAV_ANALOG דרך batt.h
#define POLL_GPS     2
#define POLL_ATT     3
#define POLL_ALT     4
#define POLL_GPSSTAT 5    // MSP_GPSSTATISTICS - 5Hz, תצוגה בלבד
#define POLL_IMU     6    // MSP_RAW_IMU (MAG)  - 5Hz, תצוגה בלבד
#if MISSION_ENABLE
#define POLL_POSE    7    // MSP2_INAV_FULL_LOCAL_POSE - 10Hz, Mission FSM בלבד
#define POLL_NAV     8    // MSP_NAV_STATUS            - 10Hz, Mission FSM בלבד
#define POLL_COUNT   9
#else
#define POLL_COUNT   7
#endif

// ═══════════════════════════════════════════════
//   מצבי הטיסה בפועל - ביטים שלנו, נשלחים לאפליקציה ב-FMD.
//   מיקום הביטים אצל INAV תלוי בתצורה, ולכן הוא לא מקובע:
//   MSP_BOXIDS נותן את ה-permanentId של כל ביט, וממנו בונים מיפוי.
//
//   ⚠️ אלה המצבים שמופעלים **בפועל**, לא אלה שביקשנו. הבקשה יוצאת כערך
//      AUX מה-FPGA, ו-INAV מחליט אם להפעיל אותה (למשל POSHOLD בלי GPS).
//      האפליקציה מחליטה לפי השדה הזה ולא לפי המצב שנבחר בממשק.
// ═══════════════════════════════════════════════
#define FM_ARM          (1u << 0)
#define FM_ANGLE        (1u << 1)
#define FM_HORIZON      (1u << 2)
#define FM_MANUAL       (1u << 3)
#define FM_NAV_ALTHOLD  (1u << 4)
#define FM_NAV_POSHOLD  (1u << 5)
#define FM_NAV_RTH      (1u << 6)
#define FM_NAV_WP       (1u << 7)
#define FM_NAV_COURSE   (1u << 8)
#define FM_NAV_CRUISE   (1u << 9)
#define FM_FAILSAFE     (1u << 10)
#define FM_SURFACE      (1u << 11)     // E1022A: INAV מדווח את מצב SURFACE **בפועל** (הדגל הפנימי)
#define FM_COUNT        12

// permanentId ב-INAV (fc_msp_box.c) - זהה ב-9.0.1 וב-9.1.0, באותו סדר כמו הביטים למעלה
static const uint8_t FM_PERMANENT_ID[FM_COUNT] = {
    0,    // ARM
    1,    // ANGLE
    2,    // HORIZON
    12,   // MANUAL
    3,    // NAV ALTHOLD
    11,   // NAV POSHOLD
    10,   // NAV RTH
    28,   // NAV WP
    45,   // NAV COURSE HOLD
    53,   // NAV CRUISE
    27,   // FAILSAFE
    33    // SURFACE (fc_msp_box.c: BOXSURFACE, permanentId 33)
};

HardwareSerial SerialFC(2);    // UART2 של הESP → Matek UART1

struct FCStatus {
    bool     armed      = false;
    bool     gpsFix     = false;
    int      gpsSats    = 0;
    double   lat        = 0.0;
    double   lon        = 0.0;

    // HDOP × 100 מ-MSP_RAW_GPS, -1 = לא ידוע. תצוגה בלבד.
    int      gpsHdop    = -1;
    // מ-MSP_GPSSTATISTICS, בסנטימטרים כמו ב-INAV, -1 = לא ידוע. תצוגה בלבד.
    int      gpsEph     = -1;
    int      gpsEpv     = -1;

    // מעלות, בדיוק של עשירית: MSP_ATTITUDE מגיע בדצי-מעלות, ושמירה כ-int
    // הייתה מאבדת את העשירית עוד לפני הטלמטריה
    float    attRoll    = 0.0f;
    float    attPitch   = 0.0f;
    int      heading    = 0;      // 0..359

    // מגנטומטר - הקריאות החיות מ-MSP_RAW_IMU (לא magzero / maggain). תצוגה בלבד.
    int16_t  magX       = 0;
    int16_t  magY       = 0;
    int16_t  magZ       = 0;
    // ג'יירו מאותה תשובת MSP_RAW_IMU, deg/s - הוכחת התנועה לבדיקת המצפן בלבד
    int16_t  gyroX      = 0;
    int16_t  gyroY      = 0;
    int16_t  gyroZ      = 0;
    // בריאות המצפן: MAG_H_UNVERIFIED / MAG_H_OK / MAG_H_FROZEN. ראו magHealthSample.
    uint8_t  magHealth  = 0;

    int32_t  altCm      = 0;      // גובה משוערך של INAV, cm - שימוש פנימי בלבד
    // מהירות אנכית משוערכת, cm/s, חיובי = טיפוס. שימוש פנימי בלבד.
    // ⚠️ זו נגזרת ולא ערך מוחלט, ולכן היא **אינה** נודדת עם הלחץ הברומטרי.
    //    בדיוק בגלל זה היא העדות שמבדילה בין המראה אמיתית לבין סחיפת ברומטר.
    int16_t  vspdCms    = 0;
    // מונה דגימות גובה/מהירות אנכית (0..255, מתגלגל). עולה רק כשהגיעה תשובה
    // חדשה מ-MSP_ALTITUDE. ⚠️ הסקר הוא 4Hz ומכונת השלבים רצה ב-20Hz, ולכן
    // ספירת "דגימות רצופות" חייבת להישען על המונה הזה ולא על זמן - אחרת
    // אותה דגימה שמורה נספרת חמש פעמים. אותו עיקרון כמו FSEQ/DSEQ בלידאר.
    uint8_t  altSeq     = 0;

    uint16_t modeFlags   = 0;     // FM_* - מצבי הטיסה הפעילים בפועל
    bool     rfPresent   = false; // E1022A: INAV מדווח חיישן rangefinder (sensors bit 4, MSP2_INAV_STATUS)
    bool     modesFresh  = false; // המיפוי ידוע והסטטוס טרי
    bool     boxMapValid = false;
    int8_t   altHoldThr  = -1;    // nav_mc_althold_throttle: 0 STICK · 1 MID_STICK · 2 HOVER · -1 לא ידוע

    bool     linkOK     = false;
    bool     armFresh   = false;
    bool     gpsFresh   = false;
    bool     attFresh   = false;
    bool     altFresh   = false;
    // טריות נתוני התצוגה האיטיים - **אינם** נכנסים ל-linkOK
    bool     gpsStatFresh = false;
    bool     magFresh     = false;

    bool     disarmSuspect = false;

#if MISSION_ENABLE
    // ═══ Mission FSM בלבד - עוברים ל-FPGA בשורת ה-INAV, לא לאפליקציה ═══
    //   כל מונה *Seq עולה רק כשהגיעה תשובת MSP חדשה באמת. ה-FPGA מודד טריות
    //   לפיהם בלבד, כמו FSEQ / DSEQ של הלידאר.
    int32_t  posX = 0, posY = 0, posZ = 0;   // cm, NEU: X צפון · Y מזרח · Z למעלה חיובי
    int16_t  velX = 0, velY = 0, velZ = 0;   // cm/s, אותם צירים
    uint8_t  poseSeq = 0;
    uint8_t  navMode = 0, navState = 0, navWpAction = 0, navWp = 0, navErr = 0;
    uint8_t  navSeq  = 0;
    uint8_t  flgSeq  = 0;                    // MSP2_INAV_STATUS חדש, עם מיפוי מצבים ידוע
    uint8_t  gpsSeq  = 0;                    // MSP_RAW_GPS חדש
#endif
};

static unsigned long lastStatusMs = 0;
static unsigned long lastGpsMs    = 0;
static unsigned long lastAttMs    = 0;
static unsigned long lastAltMs    = 0;
static unsigned long lastGpsStatMs = 0;
static unsigned long lastImuMs    = 0;
static unsigned long lastModesMs  = 0;
static unsigned long lastConfigMs = 0;
static uint8_t       configFails  = 0;
static bool          cfgRecheck   = false;   // הקישור נפל כשההגדרה הייתה ידועה - לקרוא שוב כשמנוטרל

static uint8_t fmBit[FM_COUNT];          // אינדקס הביט במסכה לכל FM_*, 0xFF = לא קיים בתצורה

static bool          prevArmed       = false;
static unsigned long lastDisarmCmdMs = 0;

// ── תזמון הסקר ──
static const uint16_t POLL_INTERVAL_MS[POLL_COUNT] = {
    MSP_STATUS_MS,   // POLL_STATUS
    MSP_BATT_MS,     // POLL_ANALOG
    MSP_GPS_MS,      // POLL_GPS
    MSP_ATT_MS,      // POLL_ATT
    MSP_ALT_MS,      // POLL_ALT
    MSP_GPSSTAT_MS,  // POLL_GPSSTAT
    MSP_IMU_MS       // POLL_IMU
#if MISSION_ENABLE
    , MISSION_POSE_MS  // POLL_POSE
    , MISSION_NAV_MS   // POLL_NAV
#endif
};
static unsigned long lastPollMs[POLL_COUNT] = { 0, 0, 0, 0, 0, 0, 0 };
// ═══ שאילתות המשימה (POSE / NAV) רק כשמשימה נבחרה ═══
//   ה-.ino מעדכן את זה בכל סבב לפי missionEngaged(). בלי משימה נבחרת העומס על
//   בקר הטיסה זהה לזה שלפני שהוספנו את המשימה (~35 שאילתות בשנייה ולא ~55).
static bool missionPollsOn = false;
static unsigned long lastAnyPollMs = 0;
// אבחון קריאת הגדרות: הבתים הראשונים שהגיעו ומתי (רק ל-MSP2_COMMON_SETTING)
static uint8_t       dgSetRaw[16];
static uint8_t       dgSetRawN    = 0;
static unsigned long dgSetFirstMs = 0, dgSetTotalMs = 0;


// ═══════════════════════════════════════════════
//   אבחון מצב INAV (FC_DEBUG) - מודפס ל-Serial Monitor רק כשמשהו משתנה
// ═══════════════════════════════════════════════
#if FC_DEBUG
static const char* const FM_NAME[FM_COUNT] = {
    "ARM", "ANGLE", "HORIZON", "MANUAL", "ALTHOLD", "POSHOLD",
    "RTH", "WP", "COURSE", "CRUISE", "FAILSAFE", "SURFACE"
};

static int fmBitOrNone(int k) { return fmBit[k] == 0xFF ? -1 : (int)fmBit[k]; }

// מיפוי הביטים מ-MSP_BOXIDS. -1 = המצב לא קיים בתצורה של INAV
static void printBoxMap(int boxCount) {
    if (!debugEnabled) return;
    static int     prevCount = -1;
    static uint8_t prevBits[FM_COUNT];
    if (boxCount == prevCount && memcmp(prevBits, fmBit, FM_COUNT) == 0) return;
    prevCount = boxCount;
    memcpy(prevBits, fmBit, FM_COUNT);
    DBG_PRINTF("[FC] BOXIDS: %d modes · bit ARM=%d ANGLE=%d ALTHOLD=%d POSHOLD=%d RTH=%d FAILSAFE=%d\n",
                  boxCount, fmBitOrNone(0), fmBitOrNone(1), fmBitOrNone(4),
                  fmBitOrNone(5), fmBitOrNone(6), fmBitOrNone(10));
}

// תוצאת הקריאה של nav_mc_althold_throttle
static void printAltHoldSetting(int8_t v, int len, int byte0) {
    if (!debugEnabled) return;
    static const char* const ATM_NAME[3] = { "STICK", "MID_STICK", "HOVER" };
    static int prev = -2;
    if (v >= 0) {
        if (v != prev) DBG_PRINTF("[FC] nav_mc_althold_throttle = %s\n", ATM_NAME[v]);
        prev = v;
        return;
    }
    prev = v;
    // כל כישלון מודפס (רק ב-Debug, ולכל היותר פעם בשנייה) עם הבתים שהגיעו
    DBG_PRINTF("[FC] nav_mc_althold_throttle: no valid answer (len=%d byte0=%d) rx=%u first=%lums total=%lums:",
               len, byte0, (unsigned)dgSetRawN, dgSetFirstMs, dgSetTotalMs);
    for (uint8_t i = 0; i < dgSetRawN && i < sizeof(dgSetRaw); i++) DBG_PRINTF(" %02X", dgSetRaw[i]);
    DBG_PRINTLN();
}
#endif

// מצבי הטיסה הפעילים - נקרא מה-.ino כשמשהו משתנה
void printInavModes(const FCStatus& s) {
#if FC_DEBUG
    if (!debugEnabled) return;
    if (!s.modesFresh) {
        DBG_PRINTF("[FC] INAV modes: ? (BOXIDS map %s, STATUS %s)\n",
                      s.boxMapValid ? "ok" : "missing", s.armFresh ? "ok" : "no answer");
        return;
    }
    DBG_PRINTF("[FC] INAV modes 0x%03X:", (unsigned)s.modeFlags);
    if (s.modeFlags == 0) DBG_PRINT(" none (ACRO)");
    for (int k = 0; k < FM_COUNT; k++) {
        if (s.modeFlags & (1u << k)) DBG_PRINTF(" %s", FM_NAME[k]);
    }
    DBG_PRINTLN();
#else
    (void)s;
#endif
}

// נקרא מה-.ino בכל DISARM / ABORT שאנחנו יוזמים
void noteDisarmCommanded() { lastDisarmCmdMs = millis(); }

// ═══ MSP v1 ═══
void mspRequest(uint8_t cmd) {
    dgMspReq++;
    uint8_t cs = 0 ^ cmd;
    SerialFC.write('$'); SerialFC.write('M'); SerialFC.write('<');
    SerialFC.write((uint8_t)0); SerialFC.write(cmd);
    SerialFC.write(cs);
}

// MSP v1 עם מטען - בשימוש MSP_SET_RTC בלבד
void mspSend(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    dgMspReq++;
    uint8_t cs = len ^ cmd;
    SerialFC.write('$'); SerialFC.write('M'); SerialFC.write('<');
    SerialFC.write(len); SerialFC.write(cmd);
    for (uint8_t i = 0; i < len; i++) {
        SerialFC.write(payload[i]);
        cs ^= payload[i];
    }
    SerialFC.write(cs);
}

bool mspResponse(uint8_t expectCmd, uint8_t* buf, uint8_t bufSize, int& len) {
    unsigned long t0 = millis();
    int      state = 0;
    uint8_t size = 0, cmd = 0, cs = 0;
    int      idx = 0;

    // ⚠️ ממשיכים לקרוא גם אחרי פסק הזמן כל עוד יש בתים בתור: הטיפול בפקודות מהאפליקציה
    //    בזמן ההמתנה (ioIdleHook) לוקח זמן, ותשובה שכבר הגיעה לא תיחשב ככישלון.
    while (millis() - t0 < MSP_TIMEOUT_MS || SerialFC.available()) {
        if (!SerialFC.available()) { ioIdleHook(); continue; }
        uint8_t c = SerialFC.read();
        switch (state) {
            case 0: if (c == '$') state = 1;                     break;
            case 1: if (c == 'M') state = 2; else state = 0;   break;
            case 2: if (c == '>') state = 3; else state = 0;   break;
            case 3:
                size = c; cs = c; idx = 0;
                if (size > bufSize) { state = 0; break; }      // מסגרת פגומה
                state = 4;
                break;
            case 4: cmd = c; cs ^= c; state = size ? 5 : 6;    break;
            case 5:
                buf[idx++] = c; cs ^= c;
                if (idx >= size) state = 6;
                break;
            case 6:
                if (c == cs && cmd == expectCmd) { len = size; return true; }
                state = 0; idx = 0;
                break;
        }
    }
    dgMspFail++;
    return false;
}

// ═══ MSP v2 - "$X<" flag fn(2) size(2) payload crc8-dvb-s2 ═══
// (שמות נפרדים מהפונקציות של batt.h, שנכלל אחרי הקובץ הזה)
static uint8_t mspCrc8(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

static void msp2Send(uint16_t fn, const uint8_t* payload, uint16_t len) {
    dgMspReq++;
    uint8_t h[8];
    h[0] = '$'; h[1] = 'X'; h[2] = '<';
    h[3] = 0;                                  // flag
    h[4] = (uint8_t)(fn & 0xFF);
    h[5] = (uint8_t)(fn >> 8);
    h[6] = (uint8_t)(len & 0xFF);
    h[7] = (uint8_t)(len >> 8);
    uint8_t crc = 0;
    for (uint8_t i = 3; i < 8; i++)  crc = mspCrc8(crc, h[i]);
    for (uint16_t i = 0; i < len; i++) crc = mspCrc8(crc, payload[i]);
    SerialFC.write(h, 8);
    if (len) SerialFC.write(payload, len);
    SerialFC.write(crc);
}

// מחזיר את אורך המטען, או -1 (פסק זמן / CRC / מטען גדול מהמאגר / תשובת שגיאה)
static int msp2Recv(uint16_t wantFn, uint8_t* out, int maxLen, uint16_t timeoutMs) {
    unsigned long t0 = millis();
    uint8_t  st = 0, crc = 0;
    uint16_t fn = 0, size = 0, idx = 0;
    const bool trace = (wantFn == MSP2_COMMON_SETTING);
    if (trace) { dgSetRawN = 0; dgSetFirstMs = 0; }

    // ⚠️ ממשיכים לקרוא גם אחרי פסק הזמן כל עוד יש בתים בתור: הטיפול בפקודות מהאפליקציה
    //    בזמן ההמתנה (ioIdleHook) לוקח זמן, ותשובה שכבר הגיעה לא תיחשב ככישלון.
    while (millis() - t0 < timeoutMs || SerialFC.available()) {
        if (!SerialFC.available()) { ioIdleHook(); continue; }
        uint8_t b = SerialFC.read();
        if (trace) {
            if (dgSetRawN == 0) dgSetFirstMs = millis() - t0;
            if (dgSetRawN < sizeof(dgSetRaw)) dgSetRaw[dgSetRawN] = b;
            if (dgSetRawN < 255) dgSetRawN++;
            dgSetTotalMs = millis() - t0;
        }
        switch (st) {
            case 0: if (b == '$') st = 1; break;
            case 1: st = (b == 'X') ? 2 : 0; break;
            case 2: st = (b == '>') ? 3 : 0; break;     // '!' = הפקודה לא נתמכת ⇒ מתעלמים
            case 3: crc = mspCrc8(0, b); st = 4; break;
            case 4: fn  = b;  crc = mspCrc8(crc, b); st = 5; break;
            case 5: fn |= (uint16_t)b << 8;  crc = mspCrc8(crc, b); st = 6; break;
            case 6: size = b; crc = mspCrc8(crc, b); st = 7; break;
            case 7: size |= (uint16_t)b << 8; crc = mspCrc8(crc, b);
                    if (size > maxLen) { st = 0; break; }
                    idx = 0; st = (size == 0) ? 9 : 8; break;
            case 8: out[idx++] = b; crc = mspCrc8(crc, b);
                    if (idx >= size) st = 9;
                    break;
            case 9: if (b == crc && fn == wantFn) return (int)size;
                    st = 0; break;
        }
    }
    if (trace) dgSetTotalMs = millis() - t0;
    dgMspFail++;
    return -1;
}

// ═══════════════════════════════════════════════
//   שעון בקר הטיסה
//
//   בלי GPS אין ל-Matek שעת אמת, ולכן קובצי ה-Blackbox וההודעות שלו נושאים
//   תאריך חסר משמעות. האפליקציה שולחת את זמן ה-UTC של הטאבלט, וה-ESP כותב
//   אותו לבקר הטיסה. זו פעולת תצורה: היא לא נוגעת בערוצים, לא עוברת דרך
//   ה-FPGA, ולא משנה דבר בהתנהגות הטיסה.
//
//   מטען MSP_SET_RTC: int32 שניות UTC (little-endian) + uint16 מילישניות.
//   INAV עונה במסגרת ריקה; חוסר תשובה מדווח ב-Serial ולא נחשב תקלה.
// ═══════════════════════════════════════════════
bool setFcRtc(uint32_t epochS, uint16_t ms) {
    uint8_t p[6] = {
        (uint8_t)(epochS        & 0xFF), (uint8_t)((epochS >>  8) & 0xFF),
        (uint8_t)((epochS >> 16) & 0xFF), (uint8_t)((epochS >> 24) & 0xFF),
        (uint8_t)(ms & 0xFF),             (uint8_t)(ms >> 8)
    };
    uint8_t buf[8];
    int     len = 0;

    while (SerialFC.available()) SerialFC.read();      // ניקוי שאריות של תשובה ישנה
    mspSend(MSP_SET_RTC, p, sizeof(p));
    bool ack      = mspResponse(MSP_SET_RTC, buf, sizeof(buf), len);
    lastAnyPollMs = millis();                          // שומר על המרווח מול שאילתות הסקר
    return ack;
}

// קריאה חוזרת לאבחון - השעה שבקר הטיסה מחזיק כרגע, 0 אם אין תשובה
uint32_t readFcRtc() {
    uint8_t buf[8];
    int     len = 0;

    while (SerialFC.available()) SerialFC.read();
    mspRequest(MSP_RTC);
    lastAnyPollMs = millis();
    if (mspResponse(MSP_RTC, buf, sizeof(buf), len) && len >= 4) {
        return (uint32_t)buf[0]         | ((uint32_t)buf[1] << 8) |
              ((uint32_t)buf[2] << 16)  | ((uint32_t)buf[3] << 24);
    }
    return 0;
}

// ═══════════════════════════════════════════════
//   בריאות המצפן - זיהוי מצפן קפוא
//
//   MAG_UNVERIFIED  אחרי הדלקה, ואחרי כל הפסקה בנתוני RAW_IMU (למשל אתחול של
//                   הבקר): עוד לא נראתה תגובה של המצפן לתנועה אמיתית.
//   MAG_OK          בחלון שבו הרחפן באמת זז, X/Y/Z השתנו.
//   MAG_FROZEN      בחלון שבו הרחפן באמת זז, X/Y/Z לא השתנו כלל.
//
//   ⚠️ ערכים קבועים כשהרחפן עומד אינם תקלה - לכן אין החלטה בלי הוכחת תנועה:
//      סיבוב מצטבר לפי הג'יירו (מאותה תשובה, בלי שאילתה נוספת) או שינוי
//      Roll/Pitch. Heading לא משמש - הוא עצמו נגזר מהמצפן.
//   ⚠️ FROZEN ננעל כל עוד מחומשים: מעבר בטיחות באוויר לא "מהבהב" חזרה.
//      על הקרקע, מנוטרל, מצפן שחזר להגיב לתנועה מאומת מחדש.
//   ⚠️ UNVERIFIED לבדו אינו מפעיל שום מעבר באוויר - הוא רק חוסם כניסה חדשה
//      למצבי ניווט אופקי (ב-FPGA ובאפליקציה).
// ═══════════════════════════════════════════════
#define MAG_H_UNVERIFIED 0
#define MAG_H_OK         1
#define MAG_H_FROZEN     2
#define MAG_HIST_N       20

struct MagSample {
    unsigned long ms;
    uint16_t      rate;          // |ω| deg/s
    int16_t       mx, my, mz;
    int16_t       roll, pitch;   // דצי-מעלות
    bool          attOk;
};
static MagSample magHist[MAG_HIST_N];
static uint8_t   magHistLen = 0;

const char* magHealthName(uint8_t h) {
    switch (h) {
        case MAG_H_OK:     return "MAG_OK";
        case MAG_H_FROZEN: return "MAG_FROZEN";
        default:           return "MAG_UNVERIFIED";
    }
}

static void magHealthSet(FCStatus& s, uint8_t h, int rot, int peak, int att, int span) {
    if (s.magHealth == h) return;
    s.magHealth = h;
#if FC_DEBUG
    if (debugEnabled)
        DBG_PRINTF("[MAG] %s  (rot=%d deg, peak=%d deg/s, roll/pitch range=%d deg, mag span=%d)\n",
                   magHealthName(h), rot, peak, att, span);
#else
    (void)rot; (void)peak; (void)att; (void)span;
#endif
}

// נקרא אחרי כל תשובת MSP_RAW_IMU תקינה
void magHealthSample(FCStatus& s, unsigned long now) {
    // דגימות שיצאו מהחלון נושרות מההתחלה
    uint8_t drop = 0;
    while (drop < magHistLen && now - magHist[drop].ms > MAG_CHECK_WINDOW_MS) drop++;
    if (magHistLen == MAG_HIST_N && drop == 0) drop = 1;
    if (drop) {
        for (uint8_t i = drop; i < magHistLen; i++) magHist[i - drop] = magHist[i];
        magHistLen -= drop;
    }

    float w = sqrtf((float)s.gyroX * s.gyroX + (float)s.gyroY * s.gyroY + (float)s.gyroZ * s.gyroZ);
    MagSample& m = magHist[magHistLen++];
    m.ms    = now;
    m.rate  = (uint16_t)(w > 2000.0f ? 2000.0f : w);
    m.mx    = s.magX;  m.my = s.magY;  m.mz = s.magZ;
    m.roll  = (int16_t)lrintf(s.attRoll  * 10.0f);
    m.pitch = (int16_t)lrintf(s.attPitch * 10.0f);
    m.attOk = s.attFresh;

    if (magHistLen < MAG_CHECK_MIN_SAMPLES) return;

    // ── הוכחת תנועה ──
    float    rot  = 0.0f;
    uint16_t peak = 0;
    int16_t  rMin = 32767, rMax = -32768, pMin = 32767, pMax = -32768;
    uint8_t  attN = 0;
    int16_t  xMin = 32767, xMax = -32768, yMin = 32767, yMax = -32768, zMin = 32767, zMax = -32768;
    for (uint8_t i = 0; i < magHistLen; i++) {
        const MagSample& q = magHist[i];
        if (q.rate > peak) peak = q.rate;
        if (i > 0) {
            unsigned long dt = q.ms - magHist[i - 1].ms;
            if (dt > 300) dt = 300;                  // פער בסקר אינו סיבוב
            rot += (float)q.rate * (float)dt / 1000.0f;
        }
        if (q.attOk) {
            attN++;
            rMin = min(rMin, q.roll);   rMax = max(rMax, q.roll);
            pMin = min(pMin, q.pitch);  pMax = max(pMax, q.pitch);
        }
        xMin = min(xMin, q.mx);  xMax = max(xMax, q.mx);
        yMin = min(yMin, q.my);  yMax = max(yMax, q.my);
        zMin = min(zMin, q.mz);  zMax = max(zMax, q.mz);
    }
    int attRange = 0;
    if (attN >= 2) {
        attRange = rMax - rMin;
        if (pMax - pMin > attRange) attRange = pMax - pMin;
    }
    bool moved = (rot >= MAG_MOVE_ROT_DEG && peak >= MAG_MOVE_PEAK_DPS) ||
                 (attRange >= MAG_MOVE_ATT_DEG * 10);
    if (!moved) return;                              // בלי תנועה אין החלטה לשום כיוון

    int span = xMax - xMin;
    if (yMax - yMin > span) span = yMax - yMin;
    if (zMax - zMin > span) span = zMax - zMin;

    if (span <= MAG_FROZEN_SPAN) {
        magHealthSet(s, MAG_H_FROZEN, (int)rot, peak, attRange / 10, span);
    } else if (span >= MAG_ALIVE_SPAN) {
        bool disarmedKnown = s.armFresh && !s.armed;
        if (s.magHealth == MAG_H_UNVERIFIED ||
            (s.magHealth == MAG_H_FROZEN && disarmedKnown))
            magHealthSet(s, MAG_H_OK, (int)rot, peak, attRange / 10, span);
    }
    // בין הספים: לא מספיק ברור - אין שינוי
}

// אין נתוני RAW_IMU טריים. ההיסטוריה מתאפסת. אימות OK מבוטל רק כשהקישור ל-Matek
// נפל (ייתכן שהבקר אותחל) - לא בגלל עיכוב רגעי בסקר, שאינו אומר דבר על המצפן.
// FROZEN נשאר תמיד.
static void magHealthStale(FCStatus& s) {
    magHistLen = 0;
    if (!s.linkOK && s.magHealth == MAG_H_OK) magHealthSet(s, MAG_H_UNVERIFIED, 0, 0, 0, 0);
}

// ═══════════════════════════════════════════════
//   טריות השדות - מחושב מחדש בכל סבב של הלולאה,
//   כדי שערך ישן לא ייראה תקף רק כי לא סקרנו אותו כרגע.
// ═══════════════════════════════════════════════
void updateFCFlags(FCStatus& s) {
    unsigned long now = millis();

    s.armFresh   = (lastStatusMs != 0) && (now - lastStatusMs < FC_STALE_MS);
    s.gpsFresh   = (lastGpsMs    != 0) && (now - lastGpsMs    < GPS_STALE_MS);
    s.attFresh   = (lastAttMs    != 0) && (now - lastAttMs    < ATT_STALE_MS);
    s.altFresh   = (lastAltMs    != 0) && (now - lastAltMs    < ALT_STALE_MS);
    s.linkOK     = s.armFresh || s.gpsFresh || s.attFresh || s.altFresh;
    s.modesFresh = s.boxMapValid && (lastModesMs != 0) && (now - lastModesMs < FC_STALE_MS);
    s.gpsStatFresh = (lastGpsStatMs != 0) && (now - lastGpsStatMs < GPSSTAT_STALE_MS);
    s.magFresh     = (lastImuMs     != 0) && (now - lastImuMs     < MAG_STALE_MS);

    // כל שדה ישן מתאפס בנפרד
    if (!s.armFresh) s.armed = false;
    if (!s.gpsFresh) { s.gpsFix = false; s.gpsSats = 0; s.lat = 0.0; s.lon = 0.0; s.gpsHdop = -1; }
    if (!s.attFresh) { s.attRoll = 0; s.attPitch = 0; s.heading = 0; }
    if (!s.gpsStatFresh) { s.gpsEph = -1; s.gpsEpv = -1; }
    if (!s.magFresh)     { s.magX = 0; s.magY = 0; s.magZ = 0;
                           s.gyroX = 0; s.gyroY = 0; s.gyroZ = 0;
                           magHealthStale(s); }
    if (!s.altFresh) { s.altCm = 0; s.vspdCms = 0; }

    // הקישור ל-FC נפל ⇒ אחרי אתחול שלו המיפוי וההגדרות עלולים להשתנות
    if (!s.linkOK) {
        s.boxMapValid = false;
        // ההגדרה לא משתנה באוויר: הערך הידוע נשמר, ונקרא שוב רק כשמנוטרל
        if (s.altHoldThr >= 0) cfgRecheck = true;
        configFails   = 0;
    }
}

// ═══════════════════════════════════════════════
//   מי השאילתה הבאה שצריך לבצע, או -1 אם אף אחת לא בשלה.
//   נבחרת זו שהכי מאחרת, כדי שאף סוג לא יורעב.
// ═══════════════════════════════════════════════
int mspDue(const FCStatus& s) {
    unsigned long now = millis();
    if (now - lastAnyPollMs < MSP_MIN_GAP_MS) return -1;
    // ה-FC לא עונה: סקר איטי, כדי שהלולאה תמשיך לשרת לידארים ואפליקציה
    if (!s.linkOK && (lastAnyPollMs != 0) && (now - lastAnyPollMs < MSP_BACKOFF_MS)) return -1;

    int           best     = -1;
    unsigned long bestLate = 0;
    for (int k = 0; k < POLL_COUNT; k++) {
#if MISSION_ENABLE
        if (!missionPollsOn && (k == POLL_POSE || k == POLL_NAV)) continue;
#endif
        unsigned long age = now - lastPollMs[k];
        if (age < POLL_INTERVAL_MS[k]) continue;
        unsigned long late = age - POLL_INTERVAL_MS[k];
        if (best < 0 || late > bestLate) { best = k; bestLate = late; }
    }
    return best;
}

// מסמנים שהשאילתה בוצעה - גם כשלא הגיעה תשובה, כדי לא להיתקע על FC מת
void mspNotePoll(uint8_t what) {
    if (what >= POLL_COUNT) return;
    lastPollMs[what] = millis();
    lastAnyPollMs    = millis();
}

// ═══════════════════════════════════════════════
//   שאילתה אחת בכל קריאה
//   what: POLL_STATUS / POLL_GPS / POLL_ATT / POLL_ALT / POLL_GPSSTAT / POLL_IMU
//   (POLL_ANALOG מטופל ב-batt.h ונקרא ישירות מה-.ino)
// ═══════════════════════════════════════════════
void pollFCOnce(FCStatus& s, uint8_t what) {
    uint8_t buf[32];
    int len = 0;

    while (SerialFC.available()) SerialFC.read();       // ניקוי שאריות של תשובה ישנה

    switch (what) {
        // ═══ MSP2_INAV_STATUS (INAV 9.0.1 / 9.1.0, fc_msp.c):
        //     [0..1] cycleTime · [2..3] i2c errors · [4..5] sensors · [6..7] load · [8] profiles
        //     [9..12] armingFlags · [13..n-2] מסכת מצבים (boxBitmask_t) · [n-1] mixer profile ═══
        case POLL_STATUS: {
            uint8_t p[40];
            msp2Send(MSP2_INAV_STATUS, nullptr, 0);
            int n = msp2Recv(MSP2_INAV_STATUS, p, sizeof(p), MSP_TIMEOUT_MS);
            if (n >= 18) {
                uint32_t arming = (uint32_t)p[9]         | ((uint32_t)p[10] << 8) |
                                  ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24);
                s.armed      = (arming & INAV_ARMING_ARMED) != 0;
                s.rfPresent  = (p[4] & (1u << 4)) != 0;      // E1022A: packSensorStatus, SENSOR_RANGEFINDER
                lastStatusMs = millis();

                if (s.boxMapValid) {
                    int maskLen = n - 14;
                    uint16_t f = 0;
                    for (int k = 0; k < FM_COUNT; k++) {
                        uint8_t bit = fmBit[k];
                        if (bit == 0xFF || (int)(bit >> 3) >= maskLen) continue;
                        if (p[13 + (bit >> 3)] & (1u << (bit & 7))) f |= (1u << k);
                    }
                    s.modeFlags = f;
                    lastModesMs = millis();
#if MISSION_ENABLE
                    s.flgSeq++;
#endif
                }

                // ניתוק חימוש שלא ביקשנו = חשד ל-failsafe
                if (prevArmed && !s.armed &&
                    (millis() - lastDisarmCmdMs > DISARM_GRACE_MS)) {
                    s.disarmSuspect = true;
                }
                if (s.armed) s.disarmSuspect = false;
                prevArmed = s.armed;
            }
            break;
        }

        case POLL_GPS:
            mspRequest(MSP_RAW_GPS);
            if (mspResponse(MSP_RAW_GPS, buf, sizeof(buf), len) && len >= 14) {
                s.gpsFix  = (buf[0] == 2);        // GPS_FIX_3D
                s.gpsSats = buf[1];
                int32_t latRaw = buf[2] | (buf[3]<<8) | (buf[4]<<16) | (buf[5]<<24);
                int32_t lonRaw = buf[6] | (buf[7]<<8) | (buf[8]<<16) | (buf[9]<<24);
                s.lat = latRaw / 1e7;
                s.lon = lonRaw / 1e7;
                // [16..17] HDOP × 100 (INAV fc_msp.c). תשובה קצרה ⇒ לא ידוע
                s.gpsHdop = (len >= 18) ? (int)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8)) : -1;
                lastGpsMs = millis();
#if MISSION_ENABLE
                s.gpsSeq++;
#endif
            }
            break;

        case POLL_ATT:      // ATTITUDE - זה גם מדד ה-IMU החי
            mspRequest(MSP_ATTITUDE);
            if (mspResponse(MSP_ATTITUDE, buf, sizeof(buf), len) && len >= 6) {
                int16_t r = (int16_t)(buf[0] | (buf[1] << 8));   // דצי-מעלות
                int16_t p = (int16_t)(buf[2] | (buf[3] << 8));   // דצי-מעלות
                int16_t y = (int16_t)(buf[4] | (buf[5] << 8));   // מעלות

                s.attRoll  = r / 10.0f;
                s.attPitch = p / 10.0f;
                s.heading  = y;
                lastAttMs  = millis();
            }
            break;

        // ═══ MSP_ALTITUDE (INAV 9): גובה משוערך int32 cm · מהירות אנכית int16 cm/s · ברומטר int32 cm
        //     נקרא לשימוש פנימי בלבד - בדיקת סבירות של DWN. לא נשלח לאף אחד. ═══
        case POLL_ALT:
            mspRequest(MSP_ALTITUDE);
            if (mspResponse(MSP_ALTITUDE, buf, sizeof(buf), len) && len >= 6) {
                s.altCm = (int32_t)((uint32_t)buf[0]         | ((uint32_t)buf[1] << 8) |
                                    ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
                // בתים 4-5: המהירות האנכית המשוערכת, int16 cm/s. הגיעה עד היום
                // בכל תשובה ופשוט לא נקראה - היא הבסיס לזיהוי המראה ומגע קרקע.
                s.vspdCms = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
                s.altSeq++;                    // דגימה חדשה באמת - מתגלגל אחרי 255
                lastAltMs = millis();
            }
            break;

        // ═══ MSP_GPSSTATISTICS (INAV 9, fc_msp.c): [0..1] lastMessageDt · [2..5] errors ·
        //     [6..9] timeouts · [10..13] packetCount · [14..15] HDOP×100 · [16..17] EPH cm ·
        //     [18..19] EPV cm (גרסאות חדשות מוסיפות בית hwVersion בסוף).
        //     תצוגה ואבחון בלבד - לא נכנס לשום החלטה. ═══
        case POLL_GPSSTAT:
            mspRequest(MSP_GPSSTATISTICS);
            if (mspResponse(MSP_GPSSTATISTICS, buf, sizeof(buf), len) && len >= 20) {
                s.gpsEph = (int)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8));
                s.gpsEpv = (int)((uint16_t)buf[18] | ((uint16_t)buf[19] << 8));
                lastGpsStatMs = millis();
            }
            break;

        // ═══ MSP_RAW_IMU (INAV 9, fc_msp.c): 9 × int16 - ACC XYZ · GYRO XYZ · MAG XYZ.
        //     MAG בבתים 12..17, ו-GYRO (deg/s) בבתים 6..11 - הוכחת התנועה לבדיקת
        //     המצפן, מאותה תשובה ובלי שאילתה נוספת. ⚠️ int16 **עם סימן**. ═══
        case POLL_IMU:
            mspRequest(MSP_RAW_IMU);
            if (mspResponse(MSP_RAW_IMU, buf, sizeof(buf), len) && len >= 18) {
                s.gyroX = (int16_t)((uint16_t)buf[6]  | ((uint16_t)buf[7]  << 8));
                s.gyroY = (int16_t)((uint16_t)buf[8]  | ((uint16_t)buf[9]  << 8));
                s.gyroZ = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
                s.magX = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
                s.magY = (int16_t)((uint16_t)buf[14] | ((uint16_t)buf[15] << 8));
                s.magZ = (int16_t)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8));
                lastImuMs = millis();
                magHealthSample(s, lastImuMs);
            }
            break;

#if MISSION_ENABLE
        // ═══ MSP2_INAV_FULL_LOCAL_POSE (INAV 9.1, fc_msp.c) - 24 בתים:
        //     [0..5] roll · pitch · yaw (int16), ואז לכל ציר X, Y, Z: מיקום int32 cm
        //     ואחריו מהירות int16 cm/s - [6..9] X [10..11] VX [12..15] Y [16..17] VY
        //     [18..21] Z [22..23] VZ. posControl.actualState.abs: NEU, Z למעלה חיובי. ═══
        case POLL_POSE: {
            uint8_t p[32];
            msp2Send(MSP2_INAV_FULL_LOCAL_POSE, nullptr, 0);
            int n = msp2Recv(MSP2_INAV_FULL_LOCAL_POSE, p, sizeof(p), MSP_TIMEOUT_MS);
            if (n >= 24) {
                s.posX = (int32_t)((uint32_t)p[6]  | ((uint32_t)p[7]  << 8) | ((uint32_t)p[8]  << 16) | ((uint32_t)p[9]  << 24));
                s.velX = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));
                s.posY = (int32_t)((uint32_t)p[12] | ((uint32_t)p[13] << 8) | ((uint32_t)p[14] << 16) | ((uint32_t)p[15] << 24));
                s.velY = (int16_t)((uint16_t)p[16] | ((uint16_t)p[17] << 8));
                s.posZ = (int32_t)((uint32_t)p[18] | ((uint32_t)p[19] << 8) | ((uint32_t)p[20] << 16) | ((uint32_t)p[21] << 24));
                s.velZ = (int16_t)((uint16_t)p[22] | ((uint16_t)p[23] << 8));
                s.poseSeq++;
            }
            break;
        }

        // ═══ MSP_NAV_STATUS (fc_msp.c): mode · state · activeWpAction ·
        //     activeWpNumber (מ-1) · error · heading target (int16) ═══
        case POLL_NAV:
            mspRequest(MSP_NAV_STATUS);
            if (mspResponse(MSP_NAV_STATUS, buf, sizeof(buf), len) && len >= 5) {
                s.navMode     = buf[0];
                s.navState    = buf[1];
                s.navWpAction = buf[2];
                s.navWp       = buf[3];
                s.navErr      = buf[4];
                s.navSeq++;
            }
            break;
#endif
    }

    updateFCFlags(s);
}

// ═══ אבחון (פקודת S ב-Serial Monitor, רק כשמנוטרל): בקשת הגדרה והדפסת הבתים הגולמיים ═══
//   לא משנה דבר בבקר - קריאה בלבד. מדפיס כל מה שחזר תוך 300ms בהקסה.
static void fcSettingProbe(const char* name) {
    while (SerialFC.available()) SerialFC.read();
    msp2Send(MSP2_COMMON_SETTING, (const uint8_t*)name, (uint16_t)(strlen(name) + 1));
    uint8_t raw[48]; int n = 0;
    unsigned long t0 = millis(), tFirst = 0, tLast = 0;
    while (millis() - t0 < 300) {
        while (SerialFC.available()) {
            int b = SerialFC.read();
            if (n == 0) tFirst = millis() - t0;
            tLast = millis() - t0;
            if (n < (int)sizeof(raw)) raw[n++] = (uint8_t)b;
        }
        delay(1);
    }
    Serial.printf("[PROBE] %s -> %d bytes (first %lums, last %lums):", name, n, tFirst, tLast);
    for (int i = 0; i < n; i++) Serial.printf(" %02X", raw[i]);
    Serial.println();
    lastAnyPollMs = millis();
}

// ═══════════════════════════════════════════════
//   הגדרות שמשתנות לעיתים רחוקות: MSP_BOXIDS ו-nav_mc_althold_throttle.
//   נקראות רק כשהן עוד לא ידועות - כלומר בהתחברות ואחרי אתחול של הקישור
//   (updateFCFlags מאפס אותן כשהקישור נופל).
//
//   ⚠️ כישלון בקריאה לא מוסתר: altHoldThr נשאר -1, הוא נשלח ככה לאפליקציה
//      (ATM:-1) ומודפס ל-Serial כשמופעל FC_DEBUG.
// ═══════════════════════════════════════════════
// ⚠️ קריאת הגדרה (MSP2_COMMON_SETTING) כבדה לבקר הטיסה - רק כשידוע שהוא מנוטרל.
//    MSP_BOXIDS (מיפוי המצבים) קל, ומותר תמיד.
static bool fcSettingReadAllowed(const FCStatus& s) { return s.armFresh && !s.armed; }

bool fcConfigDue(const FCStatus& s) {
    if (!s.linkOK)                            return false;
    bool needBox = !s.boxMapValid;
    bool needSet = (s.altHoldThr < 0 || cfgRecheck) && fcSettingReadAllowed(s);
    if (!needBox && !needSet)                 return false;
    unsigned long gap = (configFails < FC_CONFIG_FAST_TRIES) ? FC_CONFIG_RETRY_MS : FC_CONFIG_SLOW_MS;
    return millis() - lastConfigMs >= gap;
}

void pollFCConfig(FCStatus& s) {
    while (SerialFC.available()) SerialFC.read();
    lastConfigMs = millis();
    bool got = false;

    if (!s.boxMapValid) {
        uint8_t ids[96];
        int len = 0;
        mspRequest(MSP_BOXIDS);
        if (mspResponse(MSP_BOXIDS, ids, sizeof(ids), len) && len > 0) {
            for (int k = 0; k < FM_COUNT; k++) fmBit[k] = 0xFF;
            for (int i = 0; i < len; i++) {
                for (int k = 0; k < FM_COUNT; k++) {
                    if (ids[i] == FM_PERMANENT_ID[k]) fmBit[k] = (uint8_t)i;
                }
            }
            s.boxMapValid = true;
            got = true;
#if FC_DEBUG
            printBoxMap(len);
#endif
        }
    } else if (fcSettingReadAllowed(s)) {
        // nav_mc_althold_throttle - enum של בית אחד (settings.yaml: STICK, MID_STICK, HOVER)
        static const char name[] = "nav_mc_althold_throttle";
        uint8_t p[8];
        msp2Send(MSP2_COMMON_SETTING, (const uint8_t*)name, sizeof(name));   // כולל '\0'
        int n = msp2Recv(MSP2_COMMON_SETTING, p, sizeof(p), MSP_SETTING_TIMEOUT_MS);
        int8_t v = (n == 1 && p[0] <= 2) ? (int8_t)p[0] : -1;
        if (v >= 0)           { s.altHoldThr = v; cfgRecheck = false; }
        else if (!cfgRecheck)   s.altHoldThr = -1;     // בבדיקה חוזרת נשאר הערך הידוע
        got = (v >= 0);
#if FC_DEBUG
        printAltHoldSetting(s.altHoldThr, n, n > 0 ? (int)p[0] : -1);
#endif
    }

    if (got) configFails = 0;
    else if (configFails < 255) configFails++;
}
