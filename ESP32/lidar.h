#pragma once
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <Preferences.h>
#include "config.h"
#include "dwnlog.h"      // שלב 0א - תיעוד פסיבי של DOWN, לא משפיע על הלוגיקה

SFEVL53L1X lidarFwd;    // קדמי - מכשולים
SFEVL53L1X lidarDwn;    // מטה  - קרבה לקרקע

// ═══════════════════════════════════════════════
//   מצבי חיישן - נשלחים ל-FPGA ב-FOK / DOK
// ═══════════════════════════════════════════════
#define LID_FAULT       0   // לא עונה / אין דגימה שימושית 300ms / תקלת חומרה
#define LID_VALID       1   // מרחק אמין - עבר גם בדיקת סבירות
#define LID_NO_RETURN   2   // אין מטרה בטווח
#define LID_UNCERTAIN   3   // יש החזר אבל לא אמין, או קפיצה שעוד לא אושרה
#define LID_TOO_CLOSE   4   // מתחת לטווח המינימלי

#include "rfmsp.h"       // שלב 0ב - DOWN ל-INAV (MSP Rangefinder). צריך את LID_* שלמעלה

#if MISSION_ENABLE
// ═══════════════════════════════════════════════
//   סיווג הדגימה ל-Mission FSM - נשלח ל-FPGA ב-FRC / DRC
//   מפורט יותר מ-FOK / DOK, שלא שונו. אותה דגימה של FSEQ / DSEQ.
//   ⚠️ OUT_OF_RANGE = חיישן תקין בלי מטרה בטווח: אינו תקלה, ואינו הוכחה
//      לנתיב פנוי. FAULT (כולל דגימה ישנה מ-LIDAR_STALE_MS) הוא כשל אמיתי.
// ═══════════════════════════════════════════════
#define MC_FAULT        0
#define MC_VALID        1
#define MC_OOR          2   // OUT_OF_RANGE - status ב-LIDAR_OOR_STATUS_MASK (4 בלבד)
#define MC_WEAK         3   // WEAK_TARGET  - status 12: יש מטרה, אין מספיק אות
#define MC_UNCERTAIN    4   // כולל status 7 (wrapped target)
#define MC_TOO_CLOSE    5
#define MC_NO_SIGNAL    6   // signal fail  - status 2
#endif

#define LIDAR_SMOOTH            0.35f   // מקדם החלקה - נמוך = חלק יותר, איטי יותר
#define LIDAR_SILENCE_CLEAR_MS  1000    // בלי דגימה שימושית זמן כזה ⇒ מוחקים היסטוריה

// ═══════════════════════════════════════════════
//   RangeStatus - SparkFun VL53L1X 1.2.12
//   (st_src/vl53l1x_class.cpp → VL53L1X_GetRangeStatus, טבלת ה-ULD של ST)
//
//     0    תקין                          → בדיקת סבירות → VALID
//     1    sigma fail                     → UNCERTAIN
//     2    signal fail                    → NO_RETURN
//     3    מטרה מתחת לסף הזיהוי המינימלי → TOO_CLOSE
//     4    out of bounds                  → NO_RETURN  (מעבר לטווח המצב: SHORT 1.3m · LONG 4m)
//     5    תקלת חומרה                     → FAULT
//     6    תקין בלי בדיקת wraparound      → UNCERTAIN
//     7    wrapped target                 → NO_RETURN  (ב-SHORT: מעבר ל-1.3m)
//                                             ל-Mission (FRC/DRC): UNCERTAIN, לא OUT_OF_RANGE
//     9    xtalk signal fail              → UNCERTAIN
//     10   פסיקה ראשונה אחרי התחלה       → נזרקת. לא מעדכנת טריות, מצב או מונה דגימות
//     11   merged pulse                   → UNCERTAIN
//     12   target present, lack of signal → NO_RETURN
//     13   ROI לא תקין                    → FAULT
//     255  לא מוכר                        → UNCERTAIN
//
//   ⚠️ לאמת בבדיקה (LIDAR_DEBUG): איזה status החיישן התחתון מחזיר
//      כשהרחפן על הרצפה וכשמורידים אותו לאט אליה.
// ═══════════════════════════════════════════════

struct LidarData {
    int      fwd    = 0,         dwn    = 0;          // cm מנקודת המגע - ערך טוב אחרון, מסונן
    uint8_t fwdSt  = LID_FAULT, dwnSt  = LID_FAULT;
    uint8_t fwdSeq = 0,         dwnSeq = 0;          // מונה דגימות שימושיות (0..255, מתגלגל)
    bool    fwdLong = false,    dwnLong = false;     // המצב שהוחל ונקרא בחזרה מהחיישן
    int     fwdMax = LIDAR_MAX_CM_SHORT, dwnMax = LIDAR_MAX_CM_SHORT;   // הטווח המרבי במצב הנוכחי, cm
    // E1021D: מתי נקראה לאחרונה דגימה גולמית שמראה קרקע בפועל (0 = אף פעם). ראו tcRawMs
    unsigned long dwnTcRawMs = 0;
    // E1022J/K: הקריאה הגולמית האחרונה **המאומתת** של ה-DOWN בטווח המדידה (cm; מתקבלת, או קפיצה שנדחתה בשרשרת ≥ LIDAR_PROX_CHAIN_N), ‎-1 = אין.
    //   dwn מסונן (חציון + EMA) ומפגר ~10cm בירידה; לבדיקת "הייתה ממש ליד הקרקע" צריך את הגולמי.
    int     dwnRawCm = -1;
#if MISSION_ENABLE
    uint8_t fwdCls = MC_FAULT,  dwnCls = MC_FAULT;   // סיווג ה-Mission FSM (FRC / DRC)
#endif
};

struct LidarChannel {
    const char*   name;
    SFEVL53L1X*   sensor;
    int           mountCm;
    int           biasCm;
    bool          ok;               // האתחול הצליח
    bool          longMode;         // המצב שהוחל ואומת בחיישן (קריאה חוזרת)
    int           maxCm;            // הטווח המרבי במצב הנוכחי

    int           dist;             // cm, מסונן
    uint8_t       state;

    // שני זמנים נפרדים:
    //   lastRawMs  - כל דגימה שנקראה מהחיישן, כולל status 10 שנזרק (לאבחון בלבד)
    //   lastUsedMs - דגימה שעברה עיבוד וקבעה מצב. הטריות נמדדת רק ממנו
    unsigned long lastRawMs;
    unsigned long lastUsedMs;
    uint16_t      ignoredRun;       // כמה דגימות status 10 ברצף
    bool          staleReported;    // הודעת STALE כבר הודפסה

    // מונה דגימות שימושיות - עולה ב-1 בכל דגימה שעברה עיבוד.
    // השורה ל-FPGA נשלחת כל 50ms גם בלי דגימה חדשה, וה-FPGA סופר
    // דגימה רק כשהמונה השתנה (FSEQ / DSEQ)
    uint8_t       seq;

    // מסנן לתצוגה ולבקרה: חציון-3 ואז ממוצע נע מעריכי
    int           hist[3];
    uint8_t       hn;
    float         ema;
    bool          emaInit;

    // קריאות גולמיות שהתקבלו כתקינות (mm), [0] = האחרונה
    uint16_t      accRaw[3];
    unsigned long accMs[3];
    uint8_t       accN;

    // קפיצה שממתינה לקריאה שנייה
    bool          pending;
    uint16_t      pendRaw;
    unsigned long pendMs;

    bool          tooCloseLatched;
    uint8_t       tcRun;            // DOWN: דגימות TOO_CLOSE ברצף שעוד לא התקבלו (E1021C)
    // E1021D: הזמן של הדגימה הגולמית האחרונה שתומכת ב-TOO_CLOSE בעצמה: status 0 + raw < 40mm
    //   או status 3 שהתקבלה, או VALID ≤ LIDAR_NEAR_MM בזמן נעילה. **לא** מתעדכן כשהנעילה
    //   נשמרת רק בגלל NO_RETURN / UNCERTAIN (אז TOO_CLOSE הוא היסק, לא מדידה).
    unsigned long tcRawMs;
    bool          tcNoCtx;          // E1022A: הנעילה הנוכחית התקבלה רק בכלל "3 ברצף" (בלי התקרבות)
    // E1022J/K: הקריאה הגולמית האחרונה בטווח (status 0, ‏≥ LIDAR_MIN_VALID_MM) **שאומתה**, 0 = אין.
    //   אומתה = התקבלה (contiguous / confirmed), או שהיא קפיצה שנדחתה אבל חלק משרשרת של לפחות
    //   LIDAR_PROX_CHAIN_N קפיצות יורדות ברצף (ירידה מהירה: 75, 40, 21cm). קפיצה בודדת שנדחתה
    //   (90mm ביניים של 250mm) **אינה** הוכחת קרבה לקרקע (E1022K).
    //   קפיצה **עולה** שנדחתה (קריאה גבוהה מהמאומתת האחרונה) כן מעדכנת: היא עדות להתרחקות (E1022K2).
    uint16_t      lastInRangeMm;
    uint8_t       jumpChain;        // E1022K: כמה קפיצות יורדות שנדחו ברצף (קריאות status 0 בטווח, כל אחת נמוכה מקודמתה)
    uint8_t       sf2Run;           // E1022N: דגימות status 2 (signal fail) ברצף - למיפוי DOWN ל-OOR במשימה (רק כשנמשך)
#if MISSION_ENABLE
    uint8_t       lastStatus;       // ה-status של הדגימה השימושית האחרונה - לסיווג FRC / DRC
#endif
};

static LidarChannel chFwd = { "FWD", &lidarFwd, LIDAR_FWD_MOUNT_CM, LIDAR_FWD_BIAS_CM };
static LidarChannel chDwn = { "DWN", &lidarDwn, LIDAR_DWN_MOUNT_CM, LIDAR_DWN_BIAS_CM };

static unsigned long lastLidarPollMs = 0;
// E1022F: ה-Mission FSM מעל המכשול (CROSS / REAR_EDGE). DOWN שרואה את ראש המכשול קרוב
//   הוא בדיוק מה שמחפשים, גם כש-INAV מראה גובה מעל טווח החיישן - ולכן כאן VALID (status 0,
//   אחרי אישור הקפיצה הרגיל) לא נחסם לפי הגובה. TOO_CLOSE נשאר חסום כמו קודם.
//   נמצא ב-25.10: 3 מעברים בגובה 3.7m, כל VALID של ראש הקרטון סומן UNCERTAIN ⇒ 0x31.
//   רק עד LIDAR_DWN_OBST_MAX_MM: באותן טיסות, בגובה 2.5m מעל קרקע, SHORT החזיר status 0 עקבי
//   של 1361-1386mm (קיפול - הקרקע מחוץ לטווח). קריאה כזו לא תיחשב ראש מכשול.
static bool lidarDwnObstMode = false;
static bool lidarDwnRelaxAlias = false;   // E1022N: נחיתה (כוונת ירידה) - אליאס DOWN ≤ LIDAR_DWN_OBST_MAX_MM לא נחסם

// ═══════════════════════════════════════════════
//   מצב המדידה - נשמר ב-NVS ("lidar": fl / dl), נקרא בכל אתחול
// ═══════════════════════════════════════════════
static bool lidarWantLong(const LidarChannel& c) {
    Preferences pr;
    pr.begin("lidar", true);
    bool v = (&c == &chFwd) ? (pr.getUChar("fl", LIDAR_FWD_LONG_DEFAULT) != 0)
                            : (pr.getUChar("dl", LIDAR_DWN_LONG_DEFAULT) != 0);
    pr.end();
    return v;
}

static bool lidarSaveLong(const LidarChannel& c, bool lng) {
    Preferences pr;
    pr.begin("lidar", false);
    size_t w = pr.putUChar((&c == &chFwd) ? "fl" : "dl", lng ? 1 : 0);
    pr.end();
    return w == 1;
}

// מחיל מצב בחיישן וקורא בחזרה. true רק אם המצב וה-timing budget אומתו.
static bool lidarApplyMode(LidarChannel& c, bool lng) {
    if (lng) c.sensor->setDistanceModeLong();
    else     c.sensor->setDistanceModeShort();
    c.sensor->setTimingBudgetInMs(LIDAR_TIMING_BUDGET_MS);
    c.sensor->setIntermeasurementPeriod(LIDAR_IMP_MS);
    uint8_t  mode = c.sensor->getDistanceMode();           // 1 SHORT · 2 LONG
    uint16_t tb   = c.sensor->getTimingBudgetInMs();
    bool ok = (mode == (lng ? 2 : 1)) && (tb == LIDAR_TIMING_BUDGET_MS);
    if (ok) {
        c.longMode = lng;
        c.maxCm    = lng ? LIDAR_MAX_CM_LONG : LIDAR_MAX_CM_SHORT;
    }
    DBG_PRINTF("[LiDAR] %s mode %s -> read back mode=%u tb=%ums %s\n", c.name, lng ? "LONG" : "SHORT",
               (unsigned)mode, (unsigned)tb, ok ? "OK" : "MISMATCH");
    return ok;
}

// ═══════════════════════════════════════════════
//   קריאת I2C קשיחה ל-VL53L1X
//
//   ספריית SparkFun נוחה לאתחול, אבל פונקציות הקריאה הגבוהות שלה אינן
//   מחזירות לקוד שלנו את שגיאת ה-I2C. אם קריאת רגיסטר קצרה נכשלת, אפשר
//   לקבל 0mm/status255 שנראים כמו דגימה אמיתית. כאן אנחנו מאמתים במפורש
//   שכל הטרנזקציה וכל הבתים התקבלו. אין שינוי בספים/פילטרים/מצבים - רק
//   בשכבת רכישת הדגימה.
// ═══════════════════════════════════════════════
#define VL53_REG_GPIO_HV_MUX_CTRL       0x0030
#define VL53_REG_GPIO_TIO_HV_STATUS     0x0031
#define VL53_REG_SYSTEM_INTERRUPT_CLEAR 0x0086
#define VL53_REG_RESULT_RANGE_STATUS    0x0089
#define VL53_REG_RESULT_DISTANCE_MM     0x0096

static uint8_t lidarAddr7(const LidarChannel& c) {
    return (&c == &chFwd) ? ADDR_FWD_7B : ADDR_DWN_7B;
}

static bool vl53ReadRegsOnce(uint8_t addr7, uint16_t reg, uint8_t* dst, size_t len) {
    Wire.beginTransmission(addr7);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;

    size_t got = Wire.requestFrom(addr7, len, true);
    if (got != len) {
        while (Wire.available()) (void)Wire.read();
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        dst[i] = (uint8_t)Wire.read();
    }
    return true;
}

static bool vl53ReadRegs(uint8_t addr7, uint16_t reg, uint8_t* dst, size_t len) {
    if (vl53ReadRegsOnce(addr7, reg, dst, len)) return true;
    delayMicroseconds(100);
    return vl53ReadRegsOnce(addr7, reg, dst, len);
}

static bool vl53WriteByteOnce(uint8_t addr7, uint16_t reg, uint8_t value) {
    Wire.beginTransmission(addr7);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

static bool vl53WriteByte(uint8_t addr7, uint16_t reg, uint8_t value) {
    if (vl53WriteByteOnce(addr7, reg, value)) return true;
    delayMicroseconds(100);
    return vl53WriteByteOnce(addr7, reg, value);
}

// אותה לוגיקת Data Ready של ST, אבל עם בדיקת הצלחת ה-I2C.
static bool vl53DataReadySafe(uint8_t addr7, bool& ready) {
    uint8_t r[2];
    if (!vl53ReadRegs(addr7, VL53_REG_GPIO_HV_MUX_CTRL, r, sizeof(r))) return false;
    uint8_t intPol = (uint8_t)!((r[0] >> 4) & 0x01);
    ready = ((r[1] & 0x01) == intPol);
    return true;
}

// ממפה את ה-RangeStatus הגולמי בדיוק כמו ULD של ST, כדי שכל הלוגיקה
// הקיימת בהמשך תמשיך לקבל את אותם ערכי status שה-SparkFun API החזיר.
static uint8_t vl53MapRangeStatus(uint8_t rawStatus) {
    switch (rawStatus & 0x1F) {
        case 9:  return 0;
        case 6:  return 1;
        case 4:  return 2;
        case 8:  return 3;
        case 5:  return 4;
        case 3:  return 5;
        case 19: return 6;
        case 7:  return 7;
        case 12: return 9;
        case 18: return 10;
        case 22: return 11;
        case 23: return 12;
        case 13: return 13;
        default: return 255;
    }
}

// 0x0089..0x0097 הוא בלוק תוצאה אחד. קריאה אחת מבטיחה שה-status והמרחק
// מגיעים מאותה מדידה וגם מצמצמת את מספר טרנזקציות ה-I2C.
static bool vl53ReadResultSafe(uint8_t addr7, uint16_t& distanceMm, uint8_t& status) {
    uint8_t r[15];
    if (!vl53ReadRegs(addr7, VL53_REG_RESULT_RANGE_STATUS, r, sizeof(r))) return false;
    status = vl53MapRangeStatus(r[0]);
    const size_t off = (size_t)(VL53_REG_RESULT_DISTANCE_MM - VL53_REG_RESULT_RANGE_STATUS); // 13
    distanceMm = (uint16_t)(((uint16_t)r[off] << 8) | r[off + 1]);
    return true;
}

static bool vl53ClearInterruptSafe(uint8_t addr7) {
    return vl53WriteByte(addr7, VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static void lidarI2cFail(const LidarChannel& c, const char* where) {
#if LIDAR_DEBUG
    if (!debugEnabled) return;
    static unsigned long lastFwdPrint = 0;
    static unsigned long lastDwnPrint = 0;
    unsigned long now = millis();
    unsigned long& last = (&c == &chFwd) ? lastFwdPrint : lastDwnPrint;
    if (now - last >= 500) {
        last = now;
        DBG_PRINTF("[LID] %s I2C read failed at %s - sample discarded\n", c.name, where);
    }
#else
    (void)c; (void)where;
#endif
}

const char* lidarStateName(uint8_t st) {
    switch (st) {
        case LID_VALID:     return "VALID";
        case LID_NO_RETURN: return "NO_RETURN";
        case LID_UNCERTAIN: return "UNCERTAIN";
        case LID_TOO_CLOSE: return "TOO_CLOSE";
        default:            return "FAULT";
    }
}

static void resetChannel(LidarChannel& c) {
    c.dist = 0;
    c.state = LID_FAULT;
    c.lastRawMs = 0;
    c.lastUsedMs = 0;
    c.ignoredRun = 0;
    c.staleReported = false;
    c.seq = 0;
    c.hn = 0;
    c.ema = 0;
    c.emaInit = false;
    c.accN = 0;
    c.pending = false;
    c.tooCloseLatched = false;
    c.tcRun = 0;
    c.tcRawMs = 0;
    c.tcNoCtx = false;
    c.lastInRangeMm = 0;
    c.jumpChain = 0;
    c.sf2Run = 0;
#if MISSION_ENABLE
    c.lastStatus = 255;
#endif
}

static int median3(int a, int b, int c) {
    int t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

static int filterCm(LidarChannel& c, int cm) {
    c.hist[2] = c.hist[1];
    c.hist[1] = c.hist[0];
    c.hist[0] = cm;
    if (c.hn < 3) c.hn++;

    int med = (c.hn < 3) ? cm : median3(c.hist[0], c.hist[1], c.hist[2]);

    if (!c.emaInit) { c.ema = (float)med; c.emaInit = true; }
    else            { c.ema += LIDAR_SMOOTH * ((float)med - c.ema); }

    return (int)(c.ema + 0.5f);
}

// הקיזוז של מיקום החיישן (FRONT: 18cm עד קצה הפרופלור) מוחל כאן, פעם אחת בלבד.
// אף רכיב אחר (FPGA / אפליקציה) לא מקזז שוב.
static int rawToCm(const LidarChannel& c, uint16_t rawMm) {
    int cm = (int)(rawMm / 10) + c.biasCm - c.mountCm;
    if (cm < 0) cm = 0;
    if (cm > c.maxCm) cm = c.maxCm;
    return cm;
}

// שתי קריאות "קרובות" - ההפרש קטן מ-LIDAR_JUMP_MM או מ-LIDAR_JUMP_PCT של הקודמת
static bool nearMm(uint16_t a, uint16_t b) {
    int d = (int)a - (int)b;
    if (d < 0) d = -d;
    int lim = (int)b * LIDAR_JUMP_PCT / 100;
    if (lim < LIDAR_JUMP_MM) lim = LIDAR_JUMP_MM;
    return d <= lim;
}

static void pushAccepted(LidarChannel& c, uint16_t raw, unsigned long ms) {
    c.accRaw[2] = c.accRaw[1];  c.accMs[2] = c.accMs[1];
    c.accRaw[1] = c.accRaw[0];  c.accMs[1] = c.accMs[0];
    c.accRaw[0] = raw;          c.accMs[0] = ms;
    if (c.accN < 3) c.accN++;
}

// היסטוריה גולמית: הקריאה התקינה האחרונה טרייה וקרובה, והמרחק הלך וקטן
static bool approachingMinRange(const LidarChannel& c, unsigned long now) {
    if (c.accN < 2) return false;
    if (now - c.accMs[0] > LIDAR_HISTORY_MS) return false;
    if (now - c.accMs[c.accN - 1] > 2 * LIDAR_HISTORY_MS) return false;
    if (c.accRaw[0] > LIDAR_NEAR_MM) return false;
    return (int)c.accRaw[c.accN - 1] - (int)c.accRaw[0] >= LIDAR_TREND_MM;
}

// DOWN: מדידה תקינה קרובה וטרייה - הרחפן באמת היה ליד הקרקע / המכשול רגע לפני
//   (גם קריאה שעוד ממתינה לאישור קפיצה נחשבת: בירידה מהירה הקריאות האחרונות לפני
//    המגע הן לרוב "קפיצות" שלא אושרו - 75cm, 40cm, 21cm - והחיישן עצמו ראה את הקרקע מתקרבת)
static bool dwnNearContext(const LidarChannel& c, unsigned long now) {
    if (c.accN > 0 && now - c.accMs[0] <= LIDAR_TC_CTX_MS && c.accRaw[0] <= LIDAR_TC_CTX_MM) return true;
    return c.pending && now - c.pendMs <= LIDAR_TC_CTX_MS && c.pendRaw <= LIDAR_TC_CTX_MM;
}

// E1022J: הקריאה הגולמית האחרונה בטווח המדידה (cm), ‎-1 = אין
static int lastRawCm(const LidarChannel& c) {
    return c.lastInRangeMm ? rawToCm(c, c.lastInRangeMm) : -1;
}

// ═══════════════════════════════════════════════
//   קביעת מצב מדגימה שימושית אחת
// ═══════════════════════════════════════════════
static void applySample(LidarChannel& c, uint16_t raw, uint8_t status,
                        unsigned long now, bool altImplausible) {
    uint8_t cls;
    switch (status) {
        case 0:  cls = (raw < LIDAR_MIN_VALID_MM) ? LID_TOO_CLOSE : LID_VALID; break;
        case 3:  cls = LID_TOO_CLOSE; break;
        case 2:
        case 4:
        case 7:
        case 12: cls = LID_NO_RETURN; break;
        case 5:
        case 13: cls = LID_FAULT;     break;
        default: cls = LID_UNCERTAIN; break;      // 1, 6, 9, 11, 255
    }
    if (cls == LID_VALID && altImplausible &&
        !(&c == &chDwn && (lidarDwnObstMode || lidarDwnRelaxAlias) && raw <= LIDAR_DWN_OBST_MAX_MM)) cls = LID_UNCERTAIN;
    if (status == 2) { if (c.sf2Run < 255) c.sf2Run++; } else c.sf2Run = 0;   // E1022N
    if (cls != LID_VALID) c.jumpChain = 0;        // E1022K: שרשרת קפיצות נשברת בכל דגימה שאינה קריאה בטווח

    // ── DOWN: TOO_CLOSE חדש מתקבל רק עם הקשר (E1021C, ראו config.h) ──
    //   דגימה שלא התקבלה = UNCERTAIN: לא נעילה, לא "קרקע", המרחק נשאר האחרון.
    if (cls == LID_TOO_CLOSE && &c == &chDwn && !c.tooCloseLatched) {
        if (altImplausible) {
            c.tcRun   = 0;
            c.pending = false;
            c.state   = LID_UNCERTAIN;
            return;
        }
        if (!approachingMinRange(c, now) && !dwnNearContext(c, now)) {
            if (c.tcRun < 255) c.tcRun++;
            if (c.tcRun < LIDAR_TC_CONFIRM_N) {
                c.pending = false;
                c.state   = LID_UNCERTAIN;
                return;
            }
            c.tcNoCtx = true;                     // E1022A: התקבל רק בכלל הרצף
        } else {
            c.tcNoCtx = false;                    // E1022A: התקבל עם התקרבות
        }
    }
    if (cls != LID_TOO_CLOSE) c.tcRun = 0;

    // ── VALID: בדיקת סבירות בזמן ──
    if (cls == LID_VALID) {
        bool recent     = c.accN > 0 && (now - c.accMs[0]) <= LIDAR_HISTORY_MS;
        bool contiguous = recent && nearMm(raw, c.accRaw[0]);
        bool confirmed  = c.pending && (now - c.pendMs) <= LIDAR_HISTORY_MS &&
                          nearMm(raw, c.pendRaw);

        if (!contiguous && !confirmed) {          // קפיצה - מחכים לקריאה שנייה
            // E1022K: קפיצה שנדחתה נספרת כקרבה רק כחלק משרשרת קפיצות יורדות (קריאה בודדת = ייתכן שגויה)
            bool descending = c.pending && (now - c.pendMs) <= LIDAR_HISTORY_MS && raw < c.pendRaw;
            c.jumpChain = descending ? (uint8_t)(c.jumpChain + (c.jumpChain < 255)) : 1;
            if (c.jumpChain >= LIDAR_PROX_CHAIN_N) c.lastInRangeMm = raw;
            // E1022K2: קפיצה **עולה** שנדחתה (קריאה גבוהה מהקריאה המאומתת האחרונה) מעידה על התרחקות מהקרקע:
            //   היא מבטלת את הוכחת הקרבה הישנה (12cm ⇒ 25cm), גם אם עוד לא אושרה. כיוון בטוח: אין אישור נחיתה מהיר.
            else if (c.lastInRangeMm && raw > c.lastInRangeMm) c.lastInRangeMm = raw;
            c.pending = true;
            c.pendRaw = raw;
            c.pendMs  = now;
            c.state   = c.tooCloseLatched ? LID_TOO_CLOSE : LID_UNCERTAIN;
            return;
        }

        if (!contiguous) {                        // קפיצה אמיתית שאושרה
            pushAccepted(c, c.pendRaw, c.pendMs);
            c.hn = 0;
            c.emaInit = false;                    // המסנן מתחיל מהערך החדש
        }
        c.pending = false;
        pushAccepted(c, raw, now);
        c.lastInRangeMm = raw;                    // E1022K: קריאה שהתקבלה
        c.jumpChain = 0;

        if (c.tooCloseLatched && raw <= LIDAR_NEAR_MM) {
            c.tcRawMs = now;                      // מדידה אמיתית ליד הקרקע
            c.dist  = 0;                          // עדיין באזור הקרקע / המכשול
            c.state = LID_TOO_CLOSE;
            return;
        }
        c.tooCloseLatched = false;
        c.tcNoCtx = false;
        c.dist  = filterCm(c, rawToCm(c, raw));
        c.state = LID_VALID;
        return;
    }

    c.pending = false;

    if (cls == LID_TOO_CLOSE) {
        c.tcRun = 0;
        c.tcRawMs = now;                          // הדגימה עצמה מראה קרקע
        c.tooCloseLatched = true;
        c.dist  = 0;
        c.state = LID_TOO_CLOSE;
        return;
    }

    if (cls == LID_FAULT) {
        c.tooCloseLatched = false;
        c.tcNoCtx = false;
        c.accN  = 0;
        c.state = LID_FAULT;
        return;
    }

    // ── NO_RETURN / UNCERTAIN: האם נכנסנו מתחת לטווח המינימלי? ──
    if (c.tooCloseLatched || approachingMinRange(c, now)) {
        c.tooCloseLatched = true;
        c.dist  = 0;
        c.state = LID_TOO_CLOSE;
        return;
    }
    c.state = cls;                                // המרחק נשאר הערך הטוב האחרון
}

// ═══════════════════════════════════════════════
//   עיבוד דגימה אחת שנקראה מהחיישן
//   מחזיר false כשהדגימה נזרקה (status 10) - אז גם הטריות והמונה לא מתעדכנים
// ═══════════════════════════════════════════════
static bool processSample(LidarChannel& c, uint16_t raw, uint8_t status,
                          unsigned long now, bool altImplausible) {
    c.lastRawMs = now;                            // החיישן ענה - לאבחון בלבד

    if (status == 10) {                           // פסיקת סנכרון - אין בה מדידה
        if (c.ignoredRun < 0xFFFF) c.ignoredRun++;
        return false;
    }

    c.lastUsedMs = now;                           // רק דגימה שימושית שומרת על טריות
    c.ignoredRun = 0;
    c.seq++;                                      // דגימה חדשה באמת (מתגלגל אחרי 255)
#if MISSION_ENABLE
    c.lastStatus = status;
#endif
    applySample(c, raw, status, now, altImplausible);
    return true;
}

// ─── סורק I2C - אימות אמפירי של הכתובות ───────
void i2cScan() {
    if (!debugEnabled) return;
    DBG_PRINT("[I2C] Scan:");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { DBG_PRINTF(" 0x%02X", a); found++; }
    }
    DBG_PRINTF("    (%d devices)\n", found);
}

// מחזיר true רק כששני החיישנים עלו - אחרת ה-ino ינסה שוב
bool initLidars() {
    pinMode(XSHUT_FWD, OUTPUT); digitalWrite(XSHUT_FWD, LOW);
    pinMode(XSHUT_DWN, OUTPUT); digitalWrite(XSHUT_DWN, LOW);
    delay(20);

    struct SensorInit {
        int           xshut;
        LidarChannel* ch;
        uint8_t       addr8;
    } sensors[] = {
        { XSHUT_FWD, &chFwd, ADDR_FWD_8B },
        { XSHUT_DWN, &chDwn, ADDR_DWN_8B },
    };

    int okCount = 0;

    for (auto& s : sensors) {
        resetChannel(*s.ch);
        s.ch->ok = false;

        digitalWrite(s.xshut, HIGH);
        delay(15);

        uint8_t rc = s.ch->sensor->begin(Wire);
        DBG_PRINTF("[LiDAR] %s begin() rc=%u  (0=OK)\n", s.ch->name, rc);

        if (rc != 0) {
            DBG_PRINTF("[LiDAR] %s INIT FAILED\n", s.ch->name);
            digitalWrite(s.xshut, LOW);
            continue;
        }

        s.ch->sensor->setI2CAddress(s.addr8);
        delay(5);
        // המצב השמור ב-NVS. כישלון באימות ⇒ החיישן לא נחשב מוכן, וננסה שוב
        bool lng = lidarWantLong(*s.ch);
        if (!lidarApplyMode(*s.ch, lng)) {
            DBG_PRINTF("[LiDAR] %s mode verify FAILED\n", s.ch->name);
            digitalWrite(s.xshut, LOW);
            continue;
        }
        s.ch->sensor->startRanging();
        s.ch->ok = true;
        okCount++;
        DBG_PRINTF("[LiDAR] %s → 8b 0x%02X / 7b 0x%02X  %s %dms\n",
                      s.ch->name, s.addr8, s.addr8 >> 1, lng ? "LONG" : "SHORT", LIDAR_TIMING_BUDGET_MS);
    }

    i2cScan();
    return (okCount == 2);
}

// ═══════════════════════════════════════════════
//   שינוי מצב מהאפליקציה (LMODE) - ה-.ino מאפשר רק כשמנוטרל ועל הקרקע.
//   מחזיר nullptr בהצלחה, אחרת סיבה. הצלחה = המצב אומת בקריאה חוזרת ונשמר.
// ═══════════════════════════════════════════════
const char* lidarSetMode(bool front, bool lng) {
    LidarChannel& c = front ? chFwd : chDwn;
    if (!c.ok) return "NO_SENSOR";
    // כבר במצב המבוקש (למשל לחיצה כפולה מהירה) - לא נוגעים בחיישן בכלל
    if (c.longMode == lng) return nullptr;
    // החלפות רצופות מהירות מפילות את החיישן שוב ושוב - מרווח מינימלי בין החלפות
    static unsigned long lastChangeMs[2] = { 0, 0 };
    unsigned long& last = lastChangeMs[front ? 0 : 1];
    if (last != 0 && millis() - last < LIDAR_MODE_MIN_GAP_MS) return "BUSY";

    unsigned long t0 = millis();
    bool prev = c.longMode;
    c.sensor->stopRanging();
    bool ok = lidarApplyMode(c, lng);
    if (!ok) lidarApplyMode(c, prev);             // חזרה למצב הקודם, כדי לא להישאר במצב לא ידוע
    c.sensor->clearInterrupt();
    c.sensor->startRanging();

    // ⚠️ לא resetChannel: הוא מסמן FAULT עד הדגימה הראשונה, וזה הפיל את בדיקת
    //    המערכת לאדום והקפיץ את כל המסך בכל החלפה. ההחלפה היא פעולה מכוונת
    //    על הקרקע, לא תקלה. מאפסים רק את המסננים וההיסטוריה (הם של המצב הקודם),
    //    והמצב האחרון נשאר עד הדגימה הבאה - בדיוק כמו בין שתי דגימות רגילות.
    //    אם החיישן לא חוזר לדגום תוך LIDAR_STALE_MS - FAULT אמיתי, כרגיל.
    unsigned long now = millis();
    c.lastUsedMs = now;
    c.lastRawMs  = now;
    c.ignoredRun = 0;
    c.staleReported = false;
    c.hn = 0;
    c.ema = 0;
    c.emaInit = false;
    c.accN = 0;
    c.pending = false;
    last = now;
    DBG_PRINTF("[LiDAR] %s mode switch took %lums\n", c.name, now - t0);

    if (!ok) return "VERIFY";
    if (!lidarSaveLong(c, lng)) return "NVS";
    return nullptr;
}

bool lidarFwdLong() { return chFwd.longMode; }
int  lidarFwdMaxCm() { return chFwd.maxCm > 0 ? chFwd.maxCm : LIDAR_MAX_CM_SHORT; }
int  lidarDwnAliasCm() { return chDwn.longMode ? DWN_ALIAS_ALT_LONG_CM : DWN_ALIAS_ALT_SHORT_CM; }

bool checkLidarsOK() {
    uint8_t addrs7[] = { ADDR_FWD_7B, ADDR_DWN_7B };
    for (uint8_t a : addrs7) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) return false;
    }
    return true;
}

// ═══════════════════════════════════════════════
//   קריאה בקצב של החיישן - לקרוא מה-loop בכל סבב
//   dwnAltImplausible: ה-Matek מראה גובה שבו DWN לא יכול לראות קרקע
// ═══════════════════════════════════════════════
//   FRONT: שחרור נעילת TOO_CLOSE בהמראה
//
//   הנעילה (tooCloseLatched) נועדה למקרה שמטרה נכנסת מתחת לטווח המינימלי:
//   החיישן מפסיק לתת החזר תקין, ובלי הנעילה "אין החזר" היה נראה כמו נתיב פנוי.
//   היא משתחררת רק בקריאה תקינה רחוקה מ-LIDAR_NEAR_MM.
//
//   ⚠️ נמצא בטיסה (17/10): על הקרקע FRONT רואה את הדשא / הקרקע מקרוב ⇒ נעילה.
//      באוויר, בשטח פתוח, אין מטרה בטווח ולכן אין קריאה תקינה שתשחרר אותה -
//      FRONT נשאר TOO_CLOSE 0cm כל הטיסה, וה-Mission FSM עצר מיד אחרי START
//      כאילו יש מכשול צמוד.
//   נעילה שנוצרה על הקרקע לא מתארת שום דבר באוויר, ולכן היא משוחררת פעם אחת
//   במעבר ל-AIRBORNE. נעילה שנוצרת באוויר (התקרבות אמיתית) לא נפגעת.
//   הדגימה הבאה קובעת את המצב מחדש (NO_RETURN / VALID / TOO_CLOSE אמיתי).
// ═══════════════════════════════════════════════
void lidarFwdReleaseGroundLatch() {
    if (!chFwd.tooCloseLatched) return;
    chFwd.tooCloseLatched = false;
    chFwd.accN    = 0;          // ההיסטוריה מהקרקע לא רלוונטית להתקרבות באוויר
    chFwd.pending = false;
    DBG_PRINTF("[LID] FWD: ground TOO_CLOSE latch released at takeoff\n");
}

// ═══════════════════════════════════════════════
void updateLidars(bool dwnAltImplausible) {
    unsigned long now = millis();
    if (now - lastLidarPollMs < LIDAR_POLL_MS) return;
    lastLidarPollMs = now;

    LidarChannel* chans[2] = { &chFwd, &chDwn };
    for (LidarChannel* c : chans) {
        if (!c->ok) continue;

        // בלי דגימה שימושית זמן ארוך - ההיסטוריה כבר לא מתארת את המצב.
        // נבדק בכל סבב, גם כשהחיישן ממשיך לשלוח status 10.
        if (c->lastUsedMs != 0 && now - c->lastUsedMs > LIDAR_SILENCE_CLEAR_MS) {
            c->accN = 0;
            c->pending = false;
            c->tooCloseLatched = false;
        }

#if LIDAR_DEBUG
        // הודעה אחת במעבר מטרי לישן - רק כש-Debug פעיל.
        // כשהוא כבוי לא מתבצע גם bookkeeping אבחוני מיותר.
        if (debugEnabled) {
            bool stale = (c->lastUsedMs != 0) && (now - c->lastUsedMs > LIDAR_STALE_MS);
            if (stale && !c->staleReported) {
                DBG_PRINTF("[LID] %s STALE: no usable sample for %lums (last raw %lums ago, %u ignored)\n",
                              c->name, now - c->lastUsedMs,
                              c->lastRawMs ? now - c->lastRawMs : 0UL, (unsigned)c->ignoredRun);
            }
            c->staleReported = stale;
        }
#endif

        const uint8_t addr7 = lidarAddr7(*c);

        bool ready = false;
        if (!vl53DataReadySafe(addr7, ready)) {
            lidarI2cFail(*c, "DATA_READY");
            if (c == &chDwn) dwnLogPush(now, 0, 255, 'R', c->state, c->dist);   // תיעוד בלבד
            continue;                                   // אין דגימה - STALE הקיים יטפל בזה
        }
        if (!ready) continue;

        uint16_t raw = 0;
        uint8_t status = 255;
        if (!vl53ReadResultSafe(addr7, raw, status)) {
            lidarI2cFail(*c, "RESULT");
            if (c == &chDwn) dwnLogPush(now, 0, 255, 'D', c->state, c->dist);   // תיעוד בלבד
            continue;                                   // לא מעבירים 0/255 מזויפים ללוגיקה
        }
        if (!vl53ClearInterruptSafe(addr7)) {
            lidarI2cFail(*c, "CLEAR_IRQ");
            if (c == &chDwn) dwnLogPush(now, raw, status, 'C', c->state, c->dist);   // תיעוד בלבד
            continue;                                   // לא סופרים אותה דגימה פעמיים
        }

        bool used = processSample(*c, raw, status, now, (c == &chDwn) && dwnAltImplausible);

        // שלב 0א: הקריאה הגולמית (raw + status מקורי) ומה שהלוגיקה הקיימת קבעה ממנה.
        // נרשם **אחרי** העיבוד ולא משנה אותו - רק קורא את התוצאה.
        if (c == &chDwn) dwnLogPush(now, raw, status, used ? 'U' : 'I', c->state, c->dist);
        // שלב 0ב: רק דגימה שעברה עיבוד - פעם אחת, אחרי שהלוגיקה הקיימת קבעה את המצב
        if (c == &chDwn && used)
            rfMspNote(c->state, status, c->dist,
                      c->tcRawMs != 0 && (now - c->tcRawMs) <= RF_MSP_GROUND_FRESH_MS &&
                      !(c->tcNoCtx && rfAirborne));   // E1022A

#if LIDAR_DEBUG
        if (debugEnabled) {
            if (used) {
                DBG_PRINTF("[LID] %s raw=%4umm st=%3u -> %-9s %3dcm\n",
                              c->name, (unsigned)raw, (unsigned)status,
                              lidarStateName(c->state), c->dist);
            } else {
                DBG_PRINTF("[LID] %s raw=%4umm st=%3u -> ignored (%u in a row)\n",
                              c->name, (unsigned)raw, (unsigned)status, (unsigned)c->ignoredRun);
            }
        }
#else
        (void)used;
#endif
    }
}

// טריות לפי הדגימה השימושית האחרונה בלבד
static uint8_t freshState(const LidarChannel& c, unsigned long now) {
    if (!c.ok || c.lastUsedMs == 0 || now - c.lastUsedMs > LIDAR_STALE_MS) return LID_FAULT;
    return c.state;
}

#if MISSION_ENABLE
// סיווג ה-Mission FSM מתוך המצב הטרי ומה-status הגולמי של אותה דגימה.
// NO_RETURN מתפצל לפי הסיבה: מחוץ לטווח / wrapped / מטרה חלשה / אין אות.
static uint8_t missionLidarClass(const LidarChannel& c, uint8_t st) {
    switch (st) {
        case LID_VALID:     return MC_VALID;
        case LID_TOO_CLOSE: return MC_TOO_CLOSE;
        case LID_UNCERTAIN: return MC_UNCERTAIN;
        case LID_NO_RETURN:
            if (c.lastStatus < 32 && (LIDAR_OOR_STATUS_MASK & (1u << c.lastStatus))) return MC_OOR;
            if (c.lastStatus == 7)  return MC_UNCERTAIN;     // wrapped target - לא OUT_OF_RANGE
            if (c.lastStatus == 12) return MC_WEAK;
            // E1022N: DOWN בלי אות (status 2) שנמשך = אין מטרה בטווח - כמו OOR. אחרת חציית המכשול נתקעת ב-REAR_EDGE
            //   (28.10: ‏clear_smp=d_oor בלבד כש-gnd_vis=0). FWD נשאר NO_SIGNAL (ה-FSM מטפל בו כ-NR).
            //   E1022N: signal fail אינו בהכרח "אין משטח" (ראש כהה/חלש מחזיר אותו) - לכן רק כשהוא נמשך
            //   LIDAR_SIGFAIL_OOR_N דגימות ברצף. עם REAR_N=4 ב-FSM: לפחות ~0.5 שנ' של אין-אות רצוף עד אישור קצה.
            if (&c == &chDwn && c.lastStatus == 2 && c.sf2Run >= LIDAR_SIGFAIL_OOR_N) return MC_OOR;
            return MC_NO_SIGNAL;
        default:            return MC_FAULT;            // כולל דגימה ישנה
    }
}
#endif

// תמונת מצב נוכחית - טריות נבדקת כאן, המונה נלקח מאותה נקודת זמן
LidarData readLidars() {
    unsigned long now = millis();
    LidarData d;
    d.fwd    = chFwd.dist;
    d.fwdSt  = freshState(chFwd, now);
    d.fwdSeq = chFwd.seq;
    d.dwn    = chDwn.dist;
    d.dwnSt  = freshState(chDwn, now);
    d.dwnSeq = chDwn.seq;
    d.dwnTcRawMs = chDwn.tcRawMs;
    d.dwnRawCm   = lastRawCm(chDwn);
    d.fwdLong = chFwd.ok && chFwd.longMode;
    d.dwnLong = chDwn.ok && chDwn.longMode;
    d.fwdMax  = chFwd.longMode ? LIDAR_MAX_CM_LONG : LIDAR_MAX_CM_SHORT;
    d.dwnMax  = chDwn.longMode ? LIDAR_MAX_CM_LONG : LIDAR_MAX_CM_SHORT;
#if MISSION_ENABLE
    d.fwdCls = missionLidarClass(chFwd, d.fwdSt);
    d.dwnCls = missionLidarClass(chDwn, d.dwnSt);
#endif
    return d;
}
