#pragma once
#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════
//   E1022A/B - SURFACE ידני: ההחלטה אם לבקש מה-FPGA את CH8, ומסירת המצערת ביציאה
//
//   ⚠️ INAV 9.1 קורא את מתג SURFACE רק בכניסה ל-ALTHOLD / POSHOLD. ה-FPGA מוציא את
//      CH8 ומוסיף "כניסה מחדש" (200ms של ALTHOLD ב-CH7) בכל החלפה - drone_top.v.
//      כאן נקבע אם לבקש (SRF:1 בשורת החיישנים, 20Hz).
//
//   SRF:1 רק כשכל אלה מתקיימים:
//     1. האפליקציה מבקשת (SRF:1 בשורת השליטה), ויש שורות מהאפליקציה ב-SURF_APP_STALE_MS.
//     2. האפליקציה בחרה MODE:POSHOLD (ה-FPGA בודק שוב את CH7 היוצא).
//     3. אין משימה, והמצפן לא קפוא (MGH:2 ⇒ ה-FPGA עובר בעצמו ל-ALTHOLD).
//     4. INAV מדווח rangefinder (rangefinder_hardware = MSP).
//     5. מדידה חיובית (VALID עד SURF_MAX_MM, או קרקע 10mm) **נכתבה בפועל ל-UART** של INAV
//        ב-SURF_LOSS_MS האחרונים (E1022B: לא רק נכנסה לתור). מדידה שלילית שנכתבה, או
//        מעל SURF_MAX_MM ⇒ לא תקין מיד.
//   כשהתנאי 5 נכשל בזמן SURFACE - SRF:0 **וחסימה** עד שהאפליקציה שולחת SRF:0 ואז SRF:1.
//
//   ═══ E1022B / E1022C ═══
//   1. מסירת המצערת ביציאה - **ב-FPGA** (F1022B, drone_top.v 3.6). כאן רק:
//        · מרכז המקל של האפליקציה ב-SURFACE (SNT) נשלח ל-FPGA בכל שורת חיישנים כ-THS
//          (‎-1 כשאין: האפליקציה עוד לא ב-SURFACE בסשן הזה);
//        · ההחלטה על היציאה (SRF:0) נשלחת מיד. ה-FPGA מוסיף 100ms של CH3 = THS לפני היציאה,
//          ומחזיק את THS אחריה עד שהאפליקציה מתיישרת - בלי טיימר ובלי שורות שליטה מה-ESP.
//        (E1022B עשה את זה כאן, עם שורת שליטה שנוצרה ב-ESP: ההקדמה לא הובטחה בפער WiFi,
//         השורה האריכה את LINK_LOSS, והשחרור אחרי 3 שנ' היה לפי טיימר.)
//   2. פער WiFi: SURF_APP_STALE_MS = 1500 (מעל LINK_LOSS_MS של ה-FPGA, 1000). פער קצר
//      (נמדד עד 581ms) לא משנה כלום: ה-FPGA ממשיך את הערכים האחרונים ו-SURFACE נשאר.
//      פער ארוך: ה-FPGA עוצר את ה-RC (Failsafe של INAV) והבקשה ננעלת - **אין** כניסה חוזרת
//      מעצמה כשהשורות חוזרות; צריך SRF:0 ואז SRF:1 מהאפליקציה.
//   3. תקינות ה-DOWN נקבעת בכתיבה בפועל ל-UART (rfmsp.h: rfMspService), לא בכניסה לתור.
//   E1022C: מצפן קפוא בזמן SURFACE ⇒ SRF:0 וחסימה (כמו אובדן DOWN) - אין כניסה חוזרת מעצמה.
//   E1022D: כניסה רק כש-INAV כבר סומך על ה-AGL (srfRel - אותו חישוב של INAV על המסגרות שנכתבו,
//      ראו SURF_REL_* ב-config.h). עד אז SRS = 5 בלי חסימה; ה-DOWN עצמו עדיין חייב להישאר תקין.
//   E1022E: כניסה רק עד SURF_ENTER_MAX_MM (90 ס"מ); מעל זה SRS = 5 בלי חסימה.
//
//   מצב לטלמטריה (SRS): 0 כבוי · 1 פעיל · 2 נחסם: DOWN אבד (צריך לבקש מחדש)
//                        3 נחסם: אין rangefinder ב-INAV · 4 נחסם: לא POSHOLD / משימה
//                        7 נחסם: מצפן קפוא (צריך לבקש מחדש)
//                        5 נחסם: אין מדידה תקינה ברגע הבקשה · 6 נחסם: פער בשורות האפליקציה
// ═══════════════════════════════════════════════════════════

static bool     srfAppReq      = false;   // הבקשה האחרונה שהתקבלה מהאפליקציה
static bool     srfAppReqRx    = false;   // לזיהוי בקשה חדשה בשורות שהתקבלו (0 ⇒ 1)
static bool     srfReqEdge     = false;
static uint32_t srfAppMs       = 0;       // מתי הגיעה שורה אחרונה
static bool     srfAppPoshold  = false;   // MODE:POSHOLD באותה שורה
static int      srfAppCenterH  = -1;      // SNT: מרכז המקל של האפליקציה ב-SURFACE (חצאי אחוז)
static bool     srfLatchOff    = false;
static uint32_t srfOkMs        = 0;       // מדידה חיובית אחרונה שנכתבה ל-INAV (0 = אין)
static uint8_t  srfState       = 0;
static uint32_t srfLosses      = 0;       // אבחון: כמה פעמים SURFACE כובה בגלל DOWN
static bool     srfOutPrev     = false;   // SRF שנשלח בפעם הקודמת
static uint32_t srfExits       = 0;       // אבחון: יציאות אוטומטיות (DOWN / פער / מצפן) מתוך SURFACE פעיל
static float    srfRel         = 0.0f;    // E1022D: reliability של INAV, מחושב כאן על מה שנכתב
static uint32_t srfRelMs       = 0;       // המסגרת האחרונה שנכתבה (0 = אין)
static int32_t  srfLastMm      = -1;      // E1022E: המדידה האחרונה שנכתבה

// כל מסגרת שנכתבה בפועל ל-UART של INAV (rfmsp.h). mm שלילי = מחוץ לטווח
static inline void surfNoteRf(int32_t mm) {
#if SURF_ENABLE
    uint32_t n = millis();
    bool ok = (mm > 0 && mm <= SURF_MAX_MM);
    if (ok) srfOkMs = n ? n : 1;
    else    srfOkMs = 0;
    srfLastMm = mm;
    // E1022D: כמו updatePositionEstimator_SurfaceTopic ב-INAV 9.1 (כל מסגרת נקראת שם פעם אחת: המשימה כל 30ms)
    if (srfRelMs == 0 || (n - srfRelMs) > SURF_REL_TIMEOUT_MS) srfRel = 0.0f;
    else {
        float dt = (n - srfRelMs) * 0.001f;
        float a  = dt / (dt + SURF_REL_TAU_S);
        srfRel   = srfRel * (1.0f - a) + (ok ? 1.0f : 0.0f) * a;
    }
    srfRelMs = n ? n : 1;
#else
    (void)mm;
#endif
}

// מכל שורת שליטה של האפליקציה. centerH < 0 = לא נשלח
static inline void surfNoteApp(bool req, bool poshold, int centerH) {
    if (req && !srfAppReqRx) srfReqEdge = true;
    srfAppReqRx   = req;
    srfAppReq     = req;
    srfAppPoshold = poshold;
    if (centerH >= 0) srfAppCenterH = centerH;
    srfAppMs      = millis();
}

// ההחלטה עצמה. 1 = לבקש SURFACE
static inline uint8_t surfEvaluate(bool missionActive, bool rfPresent, bool magFrozen) {
#if SURF_ENABLE
    uint32_t now = millis();
    bool fresh = srfAppMs != 0 && (now - srfAppMs) <= SURF_APP_STALE_MS;
    if (!fresh) {
        if (srfAppReq) { srfLatchOff = true; srfState = 6; }   // E1022B: לא חוזר מעצמו
        else           srfState = 0;
        return 0;
    }
    if (!srfAppReq) { srfLatchOff = false; srfReqEdge = false; srfState = 0; return 0; }
    if (srfReqEdge) { srfLatchOff = false; srfReqEdge = false; }
    if (srfLatchOff)                                  return 0;   // srfState נשאר 2 / 5 / 6 / 7
    if (magFrozen)                                    { srfState = 7; srfLatchOff = true; return 0; }   // E1022C
    if (!rfPresent)                                   { srfState = 3; return 0; }
    if (!srfAppPoshold || missionActive)              { srfState = 4; return 0; }
    bool downOk = srfOkMs != 0 && (now - srfOkMs) <= SURF_LOSS_MS;
    if (!downOk) {
        srfState = (srfState == 1) ? 2 : 5;
        if (srfState == 2) srfLosses++;
        srfLatchOff = true;
        return 0;
    }
    // E1022D: כניסה רק כש-INAV כבר סומך על ה-AGL; בזמן SURFACE פעיל - הבדיקה של SURF_LOSS_MS למעלה
    if (srfState != 1 && srfRel < SURF_REL_ENTER) { srfState = 5; return 0; }
    // E1022E: כניסה רק עד SURF_ENTER_MAX_MM (מחכים בלי חסימה עד שהרחפן נמוך יותר)
    if (srfState != 1 && srfLastMm > SURF_ENTER_MAX_MM) { srfState = 5; return 0; }
    srfState = 1;
    return 1;
#else
    (void)missionActive; (void)rfPresent; (void)magFrozen;
    return 0;
#endif
}

// לפני כל שורת חיישנים ל-FPGA: SRF (1 = לבקש SURFACE)
static inline uint8_t surfTick(bool missionActive, bool rfPresent, bool magFrozen) {
    uint8_t s = surfEvaluate(missionActive, rfPresent, magFrozen);
#if SURF_ENABLE
    if (s && !srfOutPrev) srfAppCenterH = -1;   // מרכז מסשן קודם אינו תקף; האפליקציה שולחת SNT רק כשהיא ב-SURFACE
    if (!s && srfOutPrev && srfAppReq) srfExits++;
    srfOutPrev = (s != 0);
#endif
    return s;
}

// THS לשורת החיישנים: מרכז המקל של האפליקציה ב-SURFACE (חצאי אחוז), ‎-1 = אין
// E1022D: לטלמטריה (SRS), באחוזים
static inline int surfRelPct() {
#if SURF_ENABLE
    if (srfRelMs == 0 || (millis() - srfRelMs) > SURF_REL_TIMEOUT_MS) return 0;
    return (int)(srfRel * 100.0f + 0.5f);
#else
    return 0;
#endif
}

static inline int surfThs() {
#if SURF_ENABLE
    if (srfAppCenterH < 0) return -1;
    int h = srfAppCenterH;
    if (h < SURF_HOLD_MIN_H) h = SURF_HOLD_MIN_H;     // מעל min_check: INAV לוקח אותו כנייטרל (STICK)
    if (h > SURF_HOLD_MAX_H) h = SURF_HOLD_MAX_H;
    return h;
#else
    return -1;
#endif
}
