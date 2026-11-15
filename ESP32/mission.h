#pragma once
#include "config.h"

// ═══════════════════════════════════════════════════════════
//   mission.h - צד ה-ESP32 של ה-Mission FSM (המשימה האוטונומית)
//
//   נכלל תמיד (MISSION_ENABLE=1 ב-config.h). המשימה עצמה רדומה: היא מתחילה
//   רק ב-CMD:MSTART מפורש מהאפליקציה, וה-FPGA מחליט אם START מותר.
//
//   ═══ חלוקת האחריות ═══
//   ה-Mission FSM רץ ב-FPGA (mission_fsm.v), ושם כל ההחלטות. ה-ESP כאן:
//     1. קורא מ-INAV (MSP, קריאה בלבד) ומעביר ל-FPGA:
//          שורת INAV מהירה, 10Hz:
//            "NPX:x NPY:y NPZ:z NVX:vx NVY:vy NVZ:vz NPQ:s NMD:m NST:st NWP:wp
//             NER:e NNQ:s NFM:f NAR:a NFQ:s GFX:f GSA:n GEP:eph GSQ:s"
//          שורה איטית, 2Hz:
//            "MIS:c CAT:a CUC:u CRS:r CAS:v BTS:b SLQ:s CTM:m MAL:a"
//          שורת ההגדרות, 1Hz ובכל שינוי:  "MSE:d,c,v,f,a,o,s,l,k"
//        (הפירוט המלא של כל שדה: mission_rx.v)
//     2. הכנת המשימה (CMD:MPREP, רק כשמנוטרל): קורא את משימת ה-WP שב-INAV
//        (זו שנטענה מה-Configurator - האפליקציה לא שומרת עותק), בודק אותה,
//        ושומר "תמונה" שלה: מספר ה-WP ו-CRC-32. מאז ועד CMD:MCANCEL, MIS=1 רק
//        אם המשימה ב-INAV עדיין זהה לתמונה.
//        ⚠️ התמונה נשמרת ב-RAM בלבד: אחרי אתחול של ה-ESP המשימה לא מוכנה,
//           ולכן אין שום דרך שמשימה תתחיל מעצמה אחרי אתחול.
//     3. הגדרות העקיפה (CMD:MSET): בדיקת טווח, שמירה ב-NVS, שליחה ל-FPGA כסט
//        אחד עם checksum, ואישור לאפליקציה רק אחרי שה-FPGA החזיר את אותו SCK.
//     4. שואל את ה-FPGA על מצב המשימה (CMD:MSTAT, 5Hz) ומעביר לאפליקציה.
//   ⚠️ שום דבר כאן לא כותב ל-INAV - לא הגדרות, לא משימה ולא מצבי טיסה.
//   ⚠️ CMD:MSTART / MSTOP מהאפליקציה עוברים ל-FPGA כמו כל פקודה שה-ESP אינו מכיר.
//
//   ═══ טריות ═══
//   כל מונה *Q עולה רק כשהגיעה תשובת MSP חדשה באמת (msp.h). השורות נשלחות
//   בקצב קבוע, וה-FPGA מודד את גיל כל מקור לפי המונה בלבד.
//
//   ═══ בדיקת המשימה - MIS ═══
//   0 NOT_READ · 1 OK · 2 NOT_PREPARED · 3 INVALID · 4 CHANGED · 5 END_ACTION ·
//   7 SPEED_BAD · 8 READ_ERROR · 9 UNSUPPORTED_ACTION · 10 ALT_ABSOLUTE · 11 ALT_BAD
//   OK רק אם: INAV מסמן את המשימה כתקינה · כל פעולה WAYPOINT או HOLD_TIME
//   (LAND / RTH במשימה ⇒ END_ACTION: ההדגמה מסתיימת ב-POSHOLD בנקודה B) ·
//   FLAG_LAST רק באחרון · גובה WP1 יחסי לבית ולפחות MISSION_ALT_MIN_CM ·
//   המהירות האפקטיבית המרבית (p1/p2, nav_auto_speed, nav_max_auto_speed) עד
//   MISSION_SPEED_MAX_CMS · והמשימה זהה לתמונה שנשמרה ב-MPREP.
//   באותו מחזור נקראות 6 הגדרות INAV (CAT / CUC / CRS / מהירויות / thr_mid).
//   המחזור חוזר כל MISSION_VERIFY_MS כשה-FSM לא פעיל.
//   ⚠️ בזמן שה-Mission FSM פעיל לא קוראים: ה-FPGA בודק את MIS רק ב-START.
// ═══════════════════════════════════════════════════════════

#if MISSION_ENABLE

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ═══════════════════════════════════════════════
//   חלק טהור - בלי Arduino, נבדק גם במחשב (MISSION_HOST_TEST)
// ═══════════════════════════════════════════════
#define MIS_NOT_READ            0
#define MIS_OK                  1
#define MIS_NOT_PREPARED        2
#define MIS_INVALID             3
#define MIS_CHANGED             4
#define MIS_END_ACTION          5
#define MIS_SPEED_BAD           7
#define MIS_READ_ERROR          8
#define MIS_UNSUPPORTED_ACTION  9
#define MIS_ALT_ABSOLUTE        10
#define MIS_ALT_BAD             11

// פעולות WP ודגל הסיום (navigation.h של INAV)
#define WPA_WAYPOINT    1
#define WPA_HOLD_TIME   3
#define WPA_RTH         4
#define WPA_LAND        8
#define WP_FLAG_LAST    0xA5
#define MISSION_WP_LEN  21     // תשובת MSP_WP: no · action · lat · lon · alt · p1 · p2 · p3 · flag
#define MISSION_WP_MAX  63     // ה-FPGA מחזיק את מספר ה-WP ב-6 ביטים

// מצבי ה-Mission FSM שבהם הוא אינו פעיל (mission_fsm.v)
#define MST_IDLE        0
#define MST_COMPLETE    17
#define MST_SUSPENDED   18
#define MST_LAST        25

// ── CRC-32 (IEEE, כמו zlib) - ניתן לשרשור ──
static uint32_t missionCrc32(uint32_t crc, const uint8_t* p, size_t n) {
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (uint8_t k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
    return ~crc;
}

// ── בדיקת המשימה: מצטברת, WP אחרי WP ──
struct MissionEval {
    uint32_t crc;          // CRC-32 של התשובות שנקלטו
    uint8_t  n;            // כמה WP נקלטו
    bool     endAction;    // LAND / RTH
    bool     badAction;    // פעולה אחרת שאינה WAYPOINT / HOLD_TIME
    bool     badFlag;      // FLAG_LAST לפני הסוף, או דגל לא מוכר
    uint8_t  lastFlag;
    int32_t  alt1;         // גובה WP1, cm
    bool     abs1;         // WP1 בגובה מוחלט (AMSL)
    uint16_t vMax;         // המהירות המפורשת הגבוהה ביותר (p1/p2 ≥ 50)
    bool     anyDefault;   // WP בלי מהירות מפורשת ⇒ nav_auto_speed
};

static void missionEvalStart(MissionEval& e) { memset(&e, 0, sizeof(e)); }

// rec = תשובת MSP_WP אחת, MISSION_WP_LEN בתים
static void missionEvalAdd(MissionEval& e, const uint8_t* rec) {
    uint8_t  action = rec[1];
    int32_t  alt    = (int32_t)((uint32_t)rec[10] | ((uint32_t)rec[11] << 8) |
                                ((uint32_t)rec[12] << 16) | ((uint32_t)rec[13] << 24));
    uint16_t p1     = (uint16_t)rec[14] | ((uint16_t)rec[15] << 8);
    uint16_t p2     = (uint16_t)rec[16] | ((uint16_t)rec[17] << 8);
    uint16_t p3     = (uint16_t)rec[18] | ((uint16_t)rec[19] << 8);
    uint8_t  flag   = rec[20];

    if (action == WPA_LAND || action == WPA_RTH) e.endAction = true;
    else if (action != WPA_WAYPOINT && action != WPA_HOLD_TIME) e.badAction = true;
    else {
        // INAV (getActiveSpeed): HOLD_TIME - המהירות ב-p2, אחרת ב-p1. מתחת ל-50 ⇒ nav_auto_speed
        uint16_t v = (action == WPA_HOLD_TIME) ? p2 : p1;
        if (v >= 50) { if (v > e.vMax) e.vMax = v; }
        else e.anyDefault = true;
    }
    if (e.n == 0) {
        e.alt1 = alt;
        e.abs1 = (p3 & 1u) != 0;            // p3 bit0: 1 = גובה מוחלט
    }
    if (e.n > 0 && e.lastFlag == WP_FLAG_LAST) e.badFlag = true;   // FLAG_LAST רק באחרון
    if (flag != 0 && flag != WP_FLAG_LAST)       e.badFlag = true;

    e.crc      = missionCrc32(e.crc, rec, MISSION_WP_LEN);
    e.lastFlag = flag;
    if (e.n < 255) e.n++;
}

// המהירות האפקטיבית המרבית במשימה, cm/s. -1 = לא ידוע (הגדרה שלא נקראה)
static int32_t missionEffSpeed(const MissionEval& e, int32_t autoSpd, int32_t maxSpd) {
    int32_t eff = 0;
    if (e.vMax > 0) eff = (maxSpd >= 0 && (int32_t)e.vMax > maxSpd) ? maxSpd : (int32_t)e.vMax;
    if (e.anyDefault) {
        if (autoSpd < 0) return -1;
        int32_t d = (maxSpd >= 0 && autoSpd > maxSpd) ? maxSpd : autoSpd;
        if (d > eff) eff = d;
    }
    return eff;
}

// הסדר קובע איזו סיבה מדווחת: קודם מבנה המשימה, ורק אחר כך ההשוואה לתמונה
static uint8_t missionEvalFinish(const MissionEval& e, bool readErr, bool listValid, uint8_t count,
                                 int32_t effSpeed, bool prepared, uint8_t prepCount, uint32_t prepCrc) {
    if (readErr)                                              return MIS_READ_ERROR;
    if (!listValid || count == 0 || count > MISSION_WP_MAX)   return MIS_INVALID;
    if (e.n != count)                                         return MIS_READ_ERROR;
    if (e.endAction)                                          return MIS_END_ACTION;
    if (e.badAction)                                          return MIS_UNSUPPORTED_ACTION;
    if (e.badFlag || e.lastFlag != WP_FLAG_LAST)              return MIS_INVALID;
    if (e.abs1)                                               return MIS_ALT_ABSOLUTE;
    if (e.alt1 < MISSION_ALT_MIN_CM || e.alt1 > MISSION_ALT_MAX_CM) return MIS_ALT_BAD;
    if (effSpeed < 0 || effSpeed > MISSION_SPEED_MAX_CMS)    return MIS_SPEED_BAD;
    if (!prepared)                                            return MIS_NOT_PREPARED;
    if (count != prepCount || e.crc != prepCrc)               return MIS_CHANGED;
    return MIS_OK;
}

static const char* missionMisName(uint8_t m) {
    switch (m) {
        case MIS_OK:                 return "OK";
        case MIS_NOT_PREPARED:       return "NOT_PREPARED";
        case MIS_INVALID:            return "INVALID";
        case MIS_CHANGED:            return "CHANGED";
        case MIS_END_ACTION:         return "END_ACTION";
        case MIS_SPEED_BAD:          return "SPEED_BAD";
        case MIS_READ_ERROR:         return "READ_ERROR";
        case MIS_UNSUPPORTED_ACTION: return "UNSUPPORTED_ACTION";
        case MIS_ALT_ABSOLUTE:       return "ALT_ABSOLUTE";
        case MIS_ALT_BAD:            return "ALT_BAD";
        default:                     return "NOT_READ";
    }
}

// ═══ הגדרות העקיפה ═══
//   סדר קבוע, זהה ב-FPGA (mission_rx.v), ב-ESP ובאפליקציה
struct MissionSettings {
    uint16_t stop  = MSET_DEF_STOP;    // מרחק זיהוי מכשול, cm מקצה הפרופלור
    uint16_t clear = MSET_DEF_CLEAR;   // מרחק שבו FRONT נחשב פנוי, cm
    uint16_t vclr  = MSET_DEF_VCLR;    // מרווח אנכי נוסף אחרי שה-FRONT פנוי, cm
    uint16_t fclr  = MSET_DEF_FCLR;    // מרווח אופקי אחרי הקצה האחורי, cm
    uint16_t ceil  = MSET_DEF_CEIL;    // תקרת גובה, cm יחסית לבית של INAV
    uint16_t depth = MSET_DEF_DEPTH;   // עומק המכשול, cm
    uint8_t  stage = MSET_DEF_STAGE;   // שלב בדיקה: 2 עצירה · 3 +טיפוס · 4 +מעבר · 5 מלא
    uint8_t  flags = MSET_DEF_FLAGS;   // bit0 = המראה אוטומטית מותרת
};

static uint16_t missionSetVal(const MissionSettings& s, uint8_t i) {
    switch (i) {
        case 0: return s.stop;  case 1: return s.clear; case 2: return s.vclr;  case 3: return s.fclr;
        case 4: return s.ceil;  case 5: return s.depth; case 6: return s.stage; default: return s.flags;
    }
}

// Fletcher: k = Σ (8−i)·v_i, i = 0..7 - כמו ב-mission_rx.v
static uint16_t missionSetCk(const MissionSettings& s) {
    uint32_t k = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t v = missionSetVal(s, i);
        k += (uint32_t)(8 - i) * (v > 1023 ? 1023 : v);
    }
    return (uint16_t)k;
}

// טווחים. fwdMaxCm = הטווח של FRONT במצב הנוכחי (SHORT / LONG).
// מחזיר nullptr אם תקין, אחרת שם השדה הבעייתי.
static const char* missionSetCheck(const MissionSettings& s, int fwdMaxCm) {
    if (s.stop  < MSET_MIN_STOP  || s.stop  > MSET_MAX_STOP)          return "STOP";
    if (s.stop  + MSET_FRONT_MARGIN_CM > fwdMaxCm)                    return "STOP_RANGE";
    if (s.clear < s.stop + 10    || s.clear > MSET_MAX_CLEAR)         return "CLEAR";
    if (s.clear + MSET_FRONT_MARGIN_CM > fwdMaxCm)                    return "CLEAR_RANGE";
    if (s.vclr  < MSET_MIN_VCLR  || s.vclr  > MSET_MAX_VCLR)          return "VCLR";
    if (s.fclr  < MSET_MIN_FCLR  || s.fclr  > MSET_MAX_FCLR)          return "FCLR";
    if (s.ceil  < MSET_MIN_CEIL  || s.ceil  > MSET_MAX_CEIL)          return "CEIL";
    if (s.depth < MSET_MIN_DEPTH || s.depth > MSET_MAX_DEPTH)         return "DEPTH";
    if (s.stage < 2 || s.stage > 5)                                   return "STAGE";
    // E1021C / F1021E: ההמראה האוטומטית מהקרקע הוסרה (ברומטר משובש ליד הקרקע, 21.10).
    //   הדגל חייב להיות 0 - כך האפליקציה מקבלת MSET:ERR:RANGE:TAKEOFF ולא הגדרה שלא תעבוד.
    if (s.flags != 0)                                                 return "TAKEOFF";
    return nullptr;
}

// "d,c,v,f,a,o,s,l" - בדיוק 8 מספרים שלמים אי-שליליים
static bool missionSetParse(const char* p, MissionSettings& out) {
    long v[8];
    for (uint8_t i = 0; i < 8; i++) {
        char* end = nullptr;
        if (*p < '0' || *p > '9') return false;
        v[i] = strtol(p, &end, 10);
        if (end == p || v[i] < 0 || v[i] > 1023) return false;
        p = end;
        if (i < 7) { if (*p != ',') return false; p++; }
    }
    while (*p == ' ' || *p == '\r' || *p == '\n') p++;
    if (*p != '\0') return false;
    out.stop = (uint16_t)v[0]; out.clear = (uint16_t)v[1]; out.vclr = (uint16_t)v[2]; out.fclr = (uint16_t)v[3];
    out.ceil = (uint16_t)v[4]; out.depth = (uint16_t)v[5]; out.stage = (uint8_t)v[6]; out.flags = (uint8_t)v[7];
    return true;
}

static int missionFormatSet(char* out, size_t n, const MissionSettings& s) {
    return snprintf(out, n, "%u,%u,%u,%u,%u,%u,%u,%u",
                    (unsigned)s.stop, (unsigned)s.clear, (unsigned)s.vclr, (unsigned)s.fclr,
                    (unsigned)s.ceil, (unsigned)s.depth, (unsigned)s.stage, (unsigned)s.flags);
}

// ── תשובת ה-FPGA (mission_tx.v) - 55 תווים בלי ה-LF:
//    "MST:SS:RR:CCCC:DDD:TTT:WW:G:VVV:AA:PPP:LLL:BB:KKKK:F:NN" ──
struct MissionStat {
    uint8_t  st    = 0;        // מצב ה-FSM
    uint8_t  rsn   = 0;        // סיבת העצירה / הדחייה האחרונה
    uint16_t chk   = 0;        // 16 ביטי SELF_CHECK
    uint16_t det   = 0x3FF;    // FWD בזיהוי, cm (3FF = אין)
    uint16_t stp   = 0x3FF;    // FWD אחרי העצירה, cm (3FF = אין)
    uint8_t  wp    = 0;        // ה-WP שלפני העקיפה
    uint8_t  stage = 0;        // שלב הבדיקה שהוחל
    uint16_t dv    = 0;        // מהירות ברגע הזיהוי, cm/s
    uint8_t  ta    = 0xFF;     // זמן עד POSHOLD מאושר, 10ms (FF = לא)
    uint16_t tsp   = 0;        // זמן עד עצירה יציבה, 10ms
    uint16_t trv   = 0;        // תזוזה מנקודת הזיהוי עד העצירה, cm
    uint8_t  rbk   = 0;        // חזרה לאחור אחרי העצירה, cm
    uint16_t sck   = 0;        // ה-SCK של סט ההגדרות שהוחל
    uint8_t  flg   = 0;        // bit0 סט תקף · bit1 DOWN ראה קרקע · bit2 המראה מהקרקע · bit3 מצערת מסונכרנת
    uint8_t  ntr   = 0;        // נייטרל המצערת, חצאי אחוז
};

static int missionHexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool missionHexField(const char* s, uint8_t n, uint32_t& v) {
    v = 0;
    for (uint8_t i = 0; i < n; i++) {
        int h = missionHexDigit(s[i]);
        if (h < 0) return false;
        v = (v << 4) | (uint32_t)h;
    }
    return true;
}

#define MST_LEN 55

static bool missionParseMst(const char* s, size_t n, MissionStat& out) {
    if (n != MST_LEN || memcmp(s, "MST:", 4) != 0) return false;
    static const uint8_t COLON[14] = { 6, 9, 14, 18, 22, 25, 27, 31, 34, 38, 42, 45, 50, 52 };
    for (uint8_t i = 0; i < 14; i++) if (s[COLON[i]] != ':') return false;

    // מיקום ואורך של כל שדה
    static const uint8_t POS[15] = { 4, 7, 10, 15, 19, 23, 26, 28, 32, 35, 39, 43, 46, 51, 53 };
    static const uint8_t LEN[15] = { 2, 2, 4,  3,  3,  2,  1,  3,  2,  3,  3,  2,  4,  1,  2 };
    uint32_t v[15];
    for (uint8_t i = 0; i < 15; i++) if (!missionHexField(s + POS[i], LEN[i], v[i])) return false;
    if (v[0] > MST_LAST || v[5] > MISSION_WP_MAX) return false;

    out.st  = (uint8_t)v[0];   out.rsn = (uint8_t)v[1];   out.chk = (uint16_t)v[2];
    out.det = (uint16_t)v[3];  out.stp = (uint16_t)v[4];  out.wp  = (uint8_t)v[5];
    out.stage = (uint8_t)v[6]; out.dv  = (uint16_t)v[7];  out.ta  = (uint8_t)v[8];
    out.tsp = (uint16_t)v[9];  out.trv = (uint16_t)v[10]; out.rbk = (uint8_t)v[11];
    out.sck = (uint16_t)v[12]; out.flg = (uint8_t)v[13];  out.ntr = (uint8_t)v[14];
    return true;
}

static bool missionStateActive(uint8_t st) {
    return st != MST_IDLE && st != MST_COMPLETE && st != MST_SUSPENDED;
}

// ── שורות ל-FPGA ──
static int32_t missionClamp(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// X / Y נשלחים מגולגלים ל-int16: ה-FPGA צריך רק הפרשים של כמה מטרים.
// ה-FPGA רווי ב-±32767, ולכן -32768 נשלח כ-32767-.
static int32_t missionWrap16(int32_t v) {
    int32_t w = (int32_t)(int16_t)(uint16_t)((uint32_t)v & 0xFFFFu);
    return (w == -32768) ? -32767 : w;
}

struct MissionFast {
    int32_t  posX = 0, posY = 0, posZ = 0;      // cm, NEU
    int32_t  velX = 0, velY = 0, velZ = 0;      // cm/s
    uint8_t  poseSeq = 0;
    uint8_t  navMode = 0, navState = 0, navWp = 0, navErr = 0, navSeq = 0;
    uint16_t modeFlags = 0;                     // FM_* (msp.h)
    bool     armed = false;
    uint8_t  flgSeq = 0;
    bool     gpsFix = false;                    // 3D
    int32_t  gpsSats = 0;
    int32_t  gpsEph = -1;                       // cm, -1 = לא ידוע
    uint8_t  gpsSeq = 0;
};

static int missionFormatFast(char* out, size_t n, const MissionFast& f) {
    return snprintf(out, n,
        "NPX:%ld NPY:%ld NPZ:%ld NVX:%ld NVY:%ld NVZ:%ld NPQ:%u "
        "NMD:%u NST:%u NWP:%u NER:%u NNQ:%u NFM:%u NAR:%u NFQ:%u "
        "GFX:%u GSA:%ld GEP:%ld GSQ:%u",
        (long)missionWrap16(f.posX), (long)missionWrap16(f.posY),
        (long)missionClamp(f.posZ, -32767, 32767),
        (long)missionClamp(f.velX, -2047, 2047), (long)missionClamp(f.velY, -2047, 2047),
        (long)missionClamp(f.velZ, -2047, 2047), (unsigned)f.poseSeq,
        (unsigned)f.navMode, (unsigned)f.navState, (unsigned)f.navWp, (unsigned)f.navErr,
        (unsigned)f.navSeq, (unsigned)(f.modeFlags & 0x7FFu), f.armed ? 1u : 0u, (unsigned)f.flgSeq,
        f.gpsFix ? 2u : 0u, (long)missionClamp(f.gpsSats, 0, 63),
        (long)(f.gpsEph < 0 ? 9999 : missionClamp(f.gpsEph, 0, 9999)), (unsigned)f.gpsSeq);
}

// לא ידוע: CAT / CUC / CRS = 3, CAS = 2047, CTM = 0, MAL = 0 - ערכים שה-SELF_CHECK לעולם לא מקבל
static int missionFormatSlow(char* out, size_t n, uint8_t mis, int32_t atm, int32_t ucm, int32_t rst,
                             int32_t effSpd, uint8_t bts, uint8_t seq, int32_t thrMid, int32_t alt1) {
    return snprintf(out, n, "MIS:%u CAT:%ld CUC:%ld CRS:%ld CAS:%ld BTS:%u SLQ:%u CTM:%ld MAL:%ld",
        (unsigned)mis,
        (long)((atm < 0 || atm > 2) ? 3 : atm),
        (long)((ucm < 0 || ucm > 2) ? 3 : ucm),
        (long)((rst < 0 || rst > 2) ? 3 : rst),
        (long)(effSpd < 0 ? 2047 : missionClamp(effSpd, 0, 2047)),
        (unsigned)(bts > 3 ? 3 : bts), (unsigned)seq,
        (long)((thrMid < 1 || thrMid > 100) ? 0 : thrMid),
        (long)((alt1 < 1 || alt1 > 2047) ? 0 : alt1));
}

static int missionFormatMse(char* out, size_t n, const MissionSettings& s) {
    char v[48];
    missionFormatSet(v, sizeof(v), s);
    return snprintf(out, n, "MSE:%s,%u", v, (unsigned)missionSetCk(s));
}

#ifndef MISSION_HOST_TEST
// ═══════════════════════════════════════════════
//   חלק ה-ESP - MSP, NVS, ה-UART ל-FPGA והטלמטריה
//   (נכלל אחרי msp.h, batt.h, lidar.h ו-fpga_comm.h)
// ═══════════════════════════════════════════════
#include <Preferences.h>

void respond(const String& msg);       // drone_esp32.ino

#define MV_IDLE  0
#define MV_ATM   1
#define MV_UCM   2
#define MV_RST   3
#define MV_ASPD  4
#define MV_MSPD  5
#define MV_TMID  6
#define MV_INFO  7
#define MV_WP    8

struct MissionEsp {
    // ── תוצאות הבדיקה האחרונה - אלה שנשלחות ל-FPGA ──
    uint8_t       mis      = MIS_NOT_READ;
    uint8_t       wpCount  = 0;
    bool          crcKnown = false;
    uint32_t      wpCrc    = 0;
    int32_t       atm = -1, ucm = -1, rst = -1, aspd = -1, mspd = -1, tmid = -1;
    int32_t       effSpd   = -1;         // המהירות האפקטיבית המרבית במשימה
    int32_t       alt1     = 0;          // גובה WP1, cm (0 = לא ידוע / לא תקין)
    unsigned long doneMs   = 0;          // סוף המחזור האחרון, 0 = עוד לא מאז הקישור
    bool          failed   = false;      // במחזור האחרון נכשלה קריאה ⇒ ניסיון חוזר מהיר

    // ── תמונת המשימה שהוכנה (RAM בלבד) ──
    bool          prepared  = false;
    uint8_t       prepCount = 0;
    uint32_t      prepCrc   = 0;
    bool          prepReq   = false;     // CMD:MPREP ממתין לסוף מחזור קריאה

    // ── מחזור בתהליך ──
    uint8_t       step = MV_IDLE, idx = 0, count = 0;
    bool          listValid = false, readErr = false;
    int32_t       nAtm = -1, nUcm = -1, nRst = -1, nAspd = -1, nMspd = -1, nTmid = -1;
    MissionEval   ev;

    // ── הגדרות העקיפה ──
    MissionSettings set;
    bool          setPend   = false;     // ממתינים לאישור ה-FPGA על סט חדש
    unsigned long setPendMs = 0;
    unsigned long mseMs     = 0;

    // ── תשובת ה-FPGA ──
    MissionStat   stat;
    unsigned long statMs = 0;

    // ── תזמון השורות ──
    unsigned long fastMs = 0, slowMs = 0, mstatMs = 0;
    uint8_t       slowSeq = 0;
};
static MissionEsp mission;
static Preferences missionPrefs;

static bool missionStatFresh() {
    return mission.statMs != 0 && millis() - mission.statMs < MISSION_MST_STALE_MS;
}

// E1022F: ה-FSM מעל המכשול - ה-DOWN מחפש את ראש המכשול (CROSS) או את הקצה האחורי (REAR_EDGE)
#define MST_CROSS       9
#define MST_REAR_EDGE   10
static bool missionDwnOverObstacle() {
    return missionStatFresh() && (mission.stat.st == MST_CROSS || mission.stat.st == MST_REAR_EDGE);
}

// ה-FSM באמצע משימה - לפי תשובת MST טרייה בלבד
static bool missionFsmActive() {
    return missionStatFresh() && missionStateActive(mission.stat.st);
}

// ═══ "משימה נבחרה" - רק אז רצים החלקים הכבדים של המשימה ═══
//   שאילתות POSE / NAV מבקר הטיסה, שורת ה-INAV המהירה ל-FPGA, ובדיקת המשימה
//   (שקוראת הגדרות INAV ב-MSP2_COMMON_SETTING - פעולה כבדה בבקר).
//   נבחרה = האפליקציה ביקשה WP (שורת השליטה), הכנה בתהליך, או משימה פעילה ב-FPGA.
//   בכל זמן אחר הרחפן עובד בדיוק כמו לפני המשימה.
static unsigned long appWpMs = 0;        // שורת שליטה אחרונה עם MODE:WP
void missionNoteAppMode(bool wp) { if (wp) { appWpMs = millis(); if (appWpMs == 0) appWpMs = 1; } else appWpMs = 0; }
static bool missionAppWp() { return appWpMs != 0 && millis() - appWpMs < 1000; }

bool missionEngaged() {
    return mission.prepReq || mission.step != MV_IDLE || missionFsmActive() ||
           (mission.prepared && missionAppWp());
}

// ה-FPGA אישר את הסט הנוכחי: אותו SCK, והוא בטווח (F bit0)
static bool missionSetConfirmed() {
    return missionStatFresh() && mission.stat.sck == missionSetCk(mission.set) && (mission.stat.flg & 1u);
}

// נקרא מ-fpga_comm.h על כל שורה שלמה מה-FPGA. true = זו שורת MST (גם אם פגומה)
bool missionTakeLine(const String& line) {
    String l = line;
    l.trim();
    if (!l.startsWith("MST:")) return false;
    MissionStat ms;
    if (missionParseMst(l.c_str(), l.length(), ms)) {
        mission.stat   = ms;
        mission.statMs = millis();
        if (mission.statMs == 0) mission.statMs = 1;
    }
    return true;
}

// ═══ הגדרות: NVS ═══
//   מקור האמת היחיד להגדרות העקיפה. נטען בהדלקה ולא נדרס בהתחברות האפליקציה.
void missionInit(int fwdMaxCm) {
    MissionSettings s;
    missionPrefs.begin("mission", true);
    size_t n = missionPrefs.getBytesLength("set");
    if (n == sizeof(MissionSettings)) missionPrefs.getBytes("set", &s, sizeof(s));
    missionPrefs.end();
    if (missionSetCheck(s, fwdMaxCm) != nullptr) {
        DBG_PRINTLN("[MSET] NVS empty or out of range - defaults");
        s = MissionSettings();
        // ברירת המחדל עצמה חייבת להתאים לטווח של FRONT (ב-SHORT המרחקים קצרים)
        if (missionSetCheck(s, fwdMaxCm) != nullptr) {
            s.stop  = (uint16_t)(fwdMaxCm - MSET_FRONT_MARGIN_CM - 30);
            s.clear = (uint16_t)(fwdMaxCm - MSET_FRONT_MARGIN_CM);
        }
    }
    mission.set = s;
}

static bool missionSaveSet(const MissionSettings& s) {
    missionPrefs.begin("mission", false);
    size_t w = missionPrefs.putBytes("set", &s, sizeof(s));
    missionPrefs.end();
    return w == sizeof(s);
}

static void missionSendMse() {
    char line[80];
    missionFormatMse(line, sizeof(line), mission.set);
    fpgaSend(String(line));
    mission.mseMs = millis();
}

// "MSET:d,c,v,f,a,o,s,l" מהאפליקציה. canChange = מנוטרל ועל הקרקע (ה-.ino)
void missionHandleSet(const String& pkt, bool canChange, int fwdMaxCm) {
    MissionSettings s;
    if (!missionSetParse(pkt.c_str() + 5, s)) { respond("MSET:ERR:FORMAT"); return; }
    if (!canChange)                           { respond("MSET:ERR:ARMED");  return; }
    if (missionFsmActive())                   { respond("MSET:ERR:ACTIVE"); return; }
    const char* bad = missionSetCheck(s, fwdMaxCm);
    if (bad)                                  { respond(String("MSET:ERR:RANGE:") + bad); return; }
    if (!missionSaveSet(s))                   { respond("MSET:ERR:NVS");    return; }
    mission.set       = s;
    mission.setPend   = true;
    mission.setPendMs = millis();
    missionSendMse();
    DBG_PRINTF("[MSET] saved, waiting for FPGA SCK %04X\n", (unsigned)missionSetCk(s));
}

// מצב SHORT של FRONT מקצר את הטווח - הסט הנוכחי חייב להתאים לו
bool missionSetFitsFront(int fwdMaxCm) {
    return missionSetCheck(mission.set, fwdMaxCm) == nullptr;
}

// ═══ הכנת המשימה ═══
// "CMD:MPREP" - רק כשמנוטרל. קריאה מלאה של המשימה מ-INAV, והתוצאה נשלחת בסוף המחזור
void missionHandlePrep(bool canPrepare) {
    if (!canPrepare)        { respond("MPREP:ERR:ARMED");  return; }
    if (missionFsmActive()) { respond("MPREP:ERR:ACTIVE"); return; }
    mission.prepReq = true;
    mission.step    = MV_IDLE;           // מחזור חדש מההתחלה
    mission.doneMs  = 0;                 // ⇒ מתחיל מיד
}

void missionHandleCancel() {
    if (missionFsmActive()) { respond("MCANCEL:ERR:ACTIVE"); return; }
    mission.prepared = false;
    mission.prepReq  = false;
    if (mission.mis == MIS_OK || mission.mis == MIS_CHANGED) mission.mis = MIS_NOT_PREPARED;
    respond("MCANCEL:OK");
}

// ═══ בדיקת המשימה וההגדרות ═══
static void missionVerifyReset() {
    mission.mis      = MIS_NOT_READ;
    mission.wpCount  = 0;
    mission.crcKnown = false;
    mission.wpCrc    = 0;
    mission.atm = mission.ucm = mission.rst = mission.aspd = mission.mspd = mission.tmid = -1;
    mission.effSpd   = -1;
    mission.alt1     = 0;
    mission.doneMs   = 0;
    mission.failed   = false;
    mission.step     = MV_IDLE;
    if (mission.prepReq) { mission.prepReq = false; respond("MPREP:ERR:NO_FC"); }
}

// שאילתה אחת בסבב, רק כשאין שאילתת סקר בשלה (ה-.ino בודק את זה קודם)
bool missionVerifyDue(const FCStatus& fc) {
    if (!fc.linkOK) return false;
    // ⚠️ לעולם לא כשמחומש: קריאת הגדרות (MSP2_COMMON_SETTING) כבדה לבקר הטיסה.
    //    מחזור שהתחיל על הקרקע ממתין עד הניתוק.
    if (fc.armed || !fc.armFresh) return false;
    unsigned long now = millis();
    if (now - lastAnyPollMs < MSP_MIN_GAP_MS) return false;
    if (mission.step != MV_IDLE) return true;           // מחזור באמצע (MPREP)
    if (!missionEngaged())       return false;          // בלי משימה נבחרת - לא קוראים כלום
    if (mission.doneMs == 0)     return true;           // עוד לא נבדק מאז הקישור / MPREP
    if (missionFsmActive())      return false;          // בזמן משימה - לא קוראים
    return now - mission.doneMs >= (mission.failed ? MISSION_VERIFY_RETRY_MS : MISSION_VERIFY_MS);
}

// MSP2_COMMON_SETTING - הערך הגולמי של הגדרה לפי שם. want = גודל הערך בבתים
static bool missionReadSetting(const char* name, uint8_t* out, int want) {
    uint8_t p[8];
    msp2Send(MSP2_COMMON_SETTING, (const uint8_t*)name, (uint16_t)(strlen(name) + 1));   // כולל '\0'
    int n = msp2Recv(MSP2_COMMON_SETTING, p, sizeof(p), MSP_SETTING_TIMEOUT_MS);
    if (n != want) return false;
    memcpy(out, p, (size_t)want);
    return true;
}

static void missionVerifyFinish() {
    int32_t eff = missionEffSpeed(mission.ev, mission.nAspd, mission.nMspd);
    bool allRead = !mission.readErr && mission.listValid && mission.count > 0 &&
                   mission.count <= MISSION_WP_MAX && mission.ev.n == mission.count;

    // MPREP: אם המבנה תקין - זו התמונה החדשה. אחרת ההכנה נכשלה והתמונה הקודמת נמחקת.
    if (mission.prepReq) {
        mission.prepReq = false;
        uint8_t m = missionEvalFinish(mission.ev, mission.readErr, mission.listValid, mission.count,
                                      eff, true, mission.count, mission.ev.crc);
        if (m == MIS_OK) {
            mission.prepared  = true;
            mission.prepCount = mission.count;
            mission.prepCrc   = mission.ev.crc;
            respond("MPREP:OK N:" + String(mission.count) + " ALT:" + String((long)mission.ev.alt1) +
                    " SPD:" + String((long)eff));
        } else {
            mission.prepared = false;
            respond(String("MPREP:ERR:") + missionMisName(m));
        }
    }

    uint8_t mis = missionEvalFinish(mission.ev, mission.readErr, mission.listValid, mission.count,
                                    eff, mission.prepared, mission.prepCount, mission.prepCrc);

#if FC_DEBUG
    if (debugEnabled && (mis != mission.mis || eff != mission.effSpd || mission.nTmid != mission.tmid)) {
        DBG_PRINTF("[MIS] %s  wp=%u crc=%08lX alt1=%ld eff=%ld  althold_thr=%ld user_ctrl=%ld "
                   "wp_restart=%ld auto=%ld max=%ld thr_mid=%ld\n",
                   missionMisName(mis), (unsigned)mission.count,
                   (unsigned long)(allRead ? mission.ev.crc : 0UL), (long)mission.ev.alt1, (long)eff,
                   (long)mission.nAtm, (long)mission.nUcm, (long)mission.nRst,
                   (long)mission.nAspd, (long)mission.nMspd, (long)mission.nTmid);
    }
#endif

    mission.mis      = mis;
    mission.wpCount  = mission.count;
    mission.crcKnown = allRead;
    mission.wpCrc    = allRead ? mission.ev.crc : 0;
    mission.atm      = mission.nAtm;
    mission.ucm      = mission.nUcm;
    mission.rst      = mission.nRst;
    mission.aspd     = mission.nAspd;
    mission.mspd     = mission.nMspd;
    mission.tmid     = mission.nTmid;
    mission.effSpd   = allRead ? eff : -1;
    // גובה WP1 נשלח רק כשהוא יחסי ותקין - אחרת 0 וה-FPGA דוחה START (bit10)
    mission.alt1     = (allRead && !mission.ev.abs1 && mission.ev.alt1 >= MISSION_ALT_MIN_CM &&
                        mission.ev.alt1 <= MISSION_ALT_MAX_CM) ? mission.ev.alt1 : 0;
    mission.failed   = mission.readErr || mission.nAtm < 0 || mission.nUcm < 0 || mission.nRst < 0 ||
                       mission.nAspd < 0 || mission.nMspd < 0 || mission.nTmid < 0;
    mission.doneMs   = millis();
    if (mission.doneMs == 0) mission.doneMs = 1;
    mission.step     = MV_IDLE;
}

void missionVerifyStep() {
    uint8_t buf[32];
    uint8_t v[2];
    int     len = 0;

    while (SerialFC.available()) SerialFC.read();       // ניקוי שאריות של תשובה ישנה
    lastAnyPollMs = millis();

    switch (mission.step) {
        case MV_IDLE:                                    // מחזור חדש
            mission.readErr   = false;
            mission.listValid = false;
            mission.count     = 0;
            mission.idx       = 0;
            missionEvalStart(mission.ev);
            [[fallthrough]];
        case MV_ATM:        // 0 STICK · 1 MID_STICK · 2 HOVER
            mission.nAtm = (missionReadSetting("nav_mc_althold_throttle", v, 1) && v[0] <= 2) ? v[0] : -1;
            mission.step = MV_UCM;
            break;
        case MV_UCM:        // 0 ATTI · 1 CRUISE
            mission.nUcm = (missionReadSetting("nav_user_control_mode", v, 1) && v[0] <= 1) ? v[0] : -1;
            mission.step = MV_RST;
            break;
        case MV_RST:        // 0 START · 1 RESUME · 2 SWITCH
            mission.nRst = (missionReadSetting("nav_wp_mission_restart", v, 1) && v[0] <= 2) ? v[0] : -1;
            mission.step = MV_ASPD;
            break;
        case MV_ASPD:       // uint16, cm/s
            mission.nAspd = missionReadSetting("nav_auto_speed", v, 2)
                          ? (int32_t)((uint16_t)v[0] | ((uint16_t)v[1] << 8)) : -1;
            mission.step  = MV_MSPD;
            break;
        case MV_MSPD:       // uint16, cm/s
            mission.nMspd = missionReadSetting("nav_max_auto_speed", v, 2)
                          ? (int32_t)((uint16_t)v[0] | ((uint16_t)v[1] << 8)) : -1;
            mission.step  = MV_TMID;
            break;
        case MV_TMID:       // uint8, % - הנייטרל של POSHOLD בהמראה מהקרקע
            mission.nTmid = (missionReadSetting("thr_mid", v, 1) && v[0] >= 1 && v[0] <= 100) ? v[0] : -1;
            mission.step  = MV_INFO;
            break;
        case MV_INFO:       // [0] reserved · [1] max · [2] valid · [3] count
            mspRequest(MSP_WP_GETINFO);
            if (mspResponse(MSP_WP_GETINFO, buf, sizeof(buf), len) && len >= 4) {
                mission.listValid = (buf[2] != 0);
                mission.count     = buf[3];
                if (mission.listValid && mission.count >= 1 && mission.count <= MISSION_WP_MAX) {
                    mission.idx  = 1;
                    mission.step = MV_WP;
                } else {
                    missionVerifyFinish();                  // אין מה לקרוא - INVALID
                }
            } else {
                mission.readErr = true;
                missionVerifyFinish();
            }
            break;
        case MV_WP: {
            uint8_t no = mission.idx;
            mspSend(MSP_WP, &no, 1);
            if (mspResponse(MSP_WP, buf, sizeof(buf), len) && len >= MISSION_WP_LEN && buf[0] == no) {
                missionEvalAdd(mission.ev, buf);
                if (no >= mission.count) missionVerifyFinish();
                else                     mission.idx++;
            } else {
                mission.readErr = true;
                missionVerifyFinish();
            }
            break;
        }
        default:
            mission.step = MV_IDLE;
            break;
    }
}

// ═══ בכל סבב של הלולאה: קליטת MST, שורות ל-FPGA ו-CMD:MSTAT ═══
void missionLoop(const FCStatus& fc, const BattData& batt) {
    unsigned long now = millis();

    fpgaPollRx();

    // הקישור ל-FC נפל ⇒ ייתכן שהבקר אותחל: ההגדרות כבר לא ידועות.
    // התמונה של המשימה נשמרת - הבדיקה הבאה תשווה אליה.
    if (!fc.linkOK && (mission.doneMs != 0 || mission.step != MV_IDLE || mission.mis != MIS_NOT_READ))
        missionVerifyReset();

    // שורת ה-INAV המהירה (מיקום, ניווט) - רק כשמשימה נבחרה. בלעדיה ה-FSM רדום ממילא.
    if (missionEngaged() && now - mission.fastMs >= MISSION_FAST_LINE_MS) {
        mission.fastMs = now;
        MissionFast f;
        f.posX = fc.posX;  f.posY = fc.posY;  f.posZ = fc.posZ;
        f.velX = fc.velX;  f.velY = fc.velY;  f.velZ = fc.velZ;
        f.poseSeq   = fc.poseSeq;
        f.navMode   = fc.navMode;   f.navState = fc.navState;  f.navWp = fc.navWp;
        f.navErr    = fc.navErr;    f.navSeq   = fc.navSeq;
        f.modeFlags = fc.modeFlags;
        f.armed     = fc.armed;
        f.flgSeq    = fc.flgSeq;
        f.gpsFix    = fc.gpsFix;
        f.gpsSats   = fc.gpsSats;
        f.gpsEph    = fc.gpsEph;
        f.gpsSeq    = fc.gpsSeq;
        char line[200];
        missionFormatFast(line, sizeof(line), f);
        fpgaSend(String(line));
    }

    // ═══ בלי משימה נבחרת: רק המינימום ═══
    //   השורה האיטית - רק כשמשימה נבחרה. סט ההגדרות - כשמשימה נבחרה, כששמרת סט
    //   חדש, או עד שה-FPGA אישר את הסט (למשל אחרי הדלקה). בירור הסטטוס - 5Hz
    //   כשמשימה נבחרה, 2Hz אחרת: מספיק כדי לזהות משימה פעילה ולהציג את המצב.
    //   שום דבר מזה לא עובר ל-Matek - אלה שורות קצרות ל-FPGA בלבד.
    bool engaged = missionEngaged();

    if (engaged && now - mission.slowMs >= MISSION_SLOW_LINE_MS) {
        mission.slowMs = now;
        mission.slowSeq++;
        char line[128];
        missionFormatSlow(line, sizeof(line), mission.mis, mission.atm, mission.ucm, mission.rst,
                          mission.effSpd, batt.fresh ? batt.state : (uint8_t)BATT_NOT_PRESENT,
                          mission.slowSeq, mission.tmid, mission.alt1);
        fpgaSend(String(line));
    }

    // סט ההגדרות - כל שנייה, כדי שגם FPGA שאותחל יקבל אותו (START חסום עד שיתקבל)
    bool mseNeeded = engaged || mission.setPend || !missionSetConfirmed();
    unsigned long mseGap = engaged || mission.setPend ? MISSION_MSE_MS : 2 * MISSION_MSE_MS;
    if (mseNeeded && now - mission.mseMs >= mseGap) missionSendMse();

    // אישור הסט החדש: רק כשה-FPGA החזיר את אותו SCK והסט בטווח
    if (mission.setPend) {
        if (missionSetConfirmed()) {
            mission.setPend = false;
            respond("MSET:OK");
        } else if (now - mission.setPendMs >= MISSION_SET_CONFIRM_MS) {
            mission.setPend = false;
            respond("MSET:ERR:NO_CONFIRM");
        }
    }

    if (now - mission.mstatMs >= (engaged ? MISSION_STAT_MS : MISSION_STAT_IDLE_MS)) {
        mission.mstatMs = now;
        fpgaSend("CMD:MSTAT");
    }
}

// שדות המשימה בסוף חבילת הטלמטריה לאפליקציה
String missionTelemetry(const LidarData& lid) {
    const MissionStat& s = mission.stat;
    char set[48];
    missionFormatSet(set, sizeof(set), mission.set);
    char buf[420];
    snprintf(buf, sizeof(buf),
        " MSV:%u MST:%u MRS:%02X MCK:%04X MDD:%ld MDS:%ld MWW:%u MSG:%u"
        " FRC:%u DRC:%u MIS:%u MWN:%u MWC:%08lX MCF:%ld/%ld/%ld/%ld/%ld/%ld"
        " MCS:%ld MAL:%ld MPR:%u MSE:%s MSK:%u MNT:%u MFL:%X"
        " MBV:%u MBA:%d MBS:%u MBT:%u MBR:%u",
        missionStatFresh() ? 1u : 0u, (unsigned)s.st, (unsigned)s.rsn, (unsigned)s.chk,
        (long)(s.det >= 0x3FF ? -1 : (long)s.det), (long)(s.stp >= 0x3FF ? -1 : (long)s.stp),
        (unsigned)s.wp, (unsigned)s.stage,
        (unsigned)lid.fwdCls, (unsigned)lid.dwnCls, (unsigned)mission.mis, (unsigned)mission.wpCount,
        (unsigned long)(mission.crcKnown ? mission.wpCrc : 0UL),
        (long)mission.atm, (long)mission.ucm, (long)mission.rst, (long)mission.aspd,
        (long)mission.mspd, (long)mission.tmid,
        (long)mission.effSpd, (long)mission.alt1, mission.prepared ? 1u : 0u,
        set, missionSetConfirmed() ? 1u : 0u, (unsigned)s.ntr, (unsigned)s.flg,
        (unsigned)s.dv, (s.ta == 0xFF) ? -1 : (int)s.ta, (unsigned)s.tsp, (unsigned)s.trv, (unsigned)s.rbk);
    return String(buf);
}
#endif   // MISSION_HOST_TEST

#endif   // MISSION_ENABLE
