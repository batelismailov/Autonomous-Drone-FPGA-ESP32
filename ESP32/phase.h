#pragma once
#include "config.h"
#include "lidar.h"
#include "msp.h"

// ═══════════════════════════════════════════════════════════
//   מכונת שלבי הטיסה - קרקע מול אוויר
//
//   רצה כאן ולא באפליקציה, משום שה-ESP הוא הרכיב היחיד שמחזיק בו-זמנית את
//   שלושת מקורות המידע: מצב החימוש שמדווח INAV, חיישן המרחק התחתון, ואומדן
//   הגובה הפנימי. האפליקציה וה-FPGA מקבלים ממנו את השלב המוכן.
//
//     GROUND_DISARMED  →  GROUND_ARMED  →  TAKEOFF_CONFIRMING  →  AIRBORNE
//
//   ⚠️ שלושה כללי בטיחות שקובעים את כל הקובץ:
//
//   1. **היעדר מידע אינו ראיה.** מרגע שהתקבלה עדות ראשונה להמראה, המערכת
//      לא חוזרת לקרקע רק משום שחלף זמן או שהחיישן הפסיק להחזיר קריאה.
//      חזרה לקרקע מחייבת עדות חיובית שאנחנו על הקרקע, או ניתוק.
//      בגרסה קודמת היה כאן פסק זמן שהחזיר לקרקע אחרי 1.2 שניות - וזה היה
//      מסוכן בדיוק במצב שבו הרחפן באוויר וה-DWN איבד קריאה.
//
//   2. **"רצופות" פירושו רצופות.** מוני האישור והקרקע נספרים רק על דגימות
//      חדשות, וכל דגימה חדשה שאינה תומכת בתנאי מאפסת את שני המונים. שלוש
//      דגימות קרקע שמופרדות ב-NO_RETURN אינן שלוש דגימות רצופות, ואסור
//      שיצטברו לכדי החלטה. איפוס המונה אינו החלטה: הוא רק דוחה אותה,
//      והמערכת נשארת בינתיים במצב המוגן.
//
//   3. **רצפת המצערת נאכפת כאן ולא באפליקציה.** בזמן אישור ההמראה נשמרת
//      רצפה, וכל ערך נמוך ממנה נחתך אליה לפני שהוא ממשיך אל ה-FPGA.
//      האפליקציה עושה את אותו דבר לצורכי תצוגה, אבל הבטיחות לא יכולה
//      להיות תלויה בזמן החזרה של טלמטריה דרך רשת אלחוטית.
//
//   ⚠️ חימוש אינו המראה, ומצב טיסה אינו המראה. רק החיישן קובע.
// ═══════════════════════════════════════════════════════════
#define PH_GROUND_DISARMED  0
#define PH_GROUND_ARMED     1
#define PH_TAKEOFF_CONFIRM  2
#define PH_AIRBORNE         3

struct PhaseState {
    uint8_t phase      = PH_GROUND_DISARMED;
    bool    fwdActive  = false;   // מותר להשתמש ב-FWD להחלטות מכשול
    int     thrFloor   = -1;      // רצפת מצערת באחוזים, -1 = אין רצפה
    // הרצפה מוחזקת גם אחרי המעבר ל-AIRBORNE, עד שהאפליקציה מאשרת שהיא
    // כבר יודעת שאנחנו באוויר. ראו updatePhase.
    bool    floorAckWait  = false;
    bool    floorAckByTmo = false;  // הרצפה שוחררה בגיבוי הזמן ולא באישור
    // ה-DWN רואה קרקע בטווח שימושי. מידע בלבד: אינו שולט במצערת ואינו
    // מחליף את בקרת הגובה של INAV.
    bool    nearGround = false;
    // E1022G: מגע מאומת במצב ניווט (SURFACE). השלב נשאר AIRBORNE - ראו PHASE_NAVTD_*
    bool    navTouch   = false;
    // E1022N: מעבר מהיר ל-ANGLE+סרק בנגיעה (ראו PHASE_FASTTD_*). ה-ESP מחליף את שורת השליטה כל עוד הנעילה פעילה.
    bool    fastTd     = false;
    unsigned long navTdLastMs = 0;   // E1022I: הפעם האחרונה שהוכחת המגע במצב ניווט התקיימה
    // אבחון בלבד
    uint8_t confirmRun      = 0;  // דגימות תקינות רצופות בחלון האישור
    uint8_t groundRun       = 0;  // דגימות קרקע רצופות בחלון האישור
    bool    altFallbackUsed = false;
};

static uint8_t       phLastDwnSeq   = 0;
static bool          phSeqInit      = false;
static unsigned long phAltSince     = 0;
static unsigned long phLandSince    = 0;
static unsigned long phNavTdSince   = 0;   // E1022G
static uint8_t       phNavTdAirRun  = 0;   // E1022G: דגימות "באוויר" רצופות
static uint8_t       phFastRun      = 0;   // E1022N: דגימות DWN חדשות רצופות עם הוכחת קרקע
static unsigned long phFastRunMs    = 0;   // E1022N: זמן דגימת הקרקע האחרונה שנספרה
static unsigned long phFastTcRaw    = 0;   // E1022N: dwnTcRawMs האחרון שנספר
static unsigned long phFastRelSince = 0;   // E1022N: מתי המקל עלה מעל סף השחרור
static unsigned long phFastLockUntil = 0;  // E1022N: עד מתי אסור להפעיל מחדש
static int           phLastValidCm  = -1;  // E1022J: המדידה הגולמית התקינה האחרונה של ה-DOWN (lid.dwnRawCm), ‎-1 = אין
static unsigned long phFwdSince     = 0;
static unsigned long phAirborneSince = 0;
static unsigned long phTakeoffAltSince = 0;   // כמה זמן INAV מדווח גובה המראה
static uint8_t       phClimbRun        = 0;   // דגימות VSPD חדשות רצופות מעל הסף
static unsigned long phClimbSeenMs     = 0;   // מתי אושרה לאחרונה עדות טיפוס
static uint8_t       phLastAltSeq      = 0;   // מונה דגימות הגובה שכבר נספרו
static bool          phAltSeqInit      = false;
static unsigned long phNearGroundMs   = 0;    // מתי נראתה לאחרונה קרקע בטווח
static int           phNearGroundCm   = -1;   // ומה היה המרחק אז - קובע בזיהוי המגע
// E1021C: TOO_CLOSE שהתחיל מיד אחרי התקרבות טרייה ונמשך ברצף מאז (רק הוא עדות מגע)
static bool          phTcApproach     = false;
static bool          phTcPrev         = false;

const char* phaseName(uint8_t p) {
    switch (p) {
        case PH_GROUND_ARMED:    return "GROUND_ARMED";
        case PH_TAKEOFF_CONFIRM: return "TAKEOFF_CONFIRMING";
        case PH_AIRBORNE:        return "AIRBORNE";
        default:                 return "GROUND_DISARMED";
    }
}

// ═══════════════════════════════════════════════
//   שיא מצערת קצר - לקביעת רצפת ההמראה
//
//   חוצץ מעגלי קטן של ערכי המצערת האחרונים עם חותמת זמן לכל אחד. השיא
//   מחושב רק על ערכים שנמצאים בתוך החלון, ולכן ערך ישן נושר מעצמו בלי
//   שצריך לנקות אותו. כך "שיא אחרון" אינו הופך ל"שיא מאז החימוש".
// ═══════════════════════════════════════════════
struct ThrottleTrace {
    uint8_t       pct[PHASE_THR_TRACE_N];
    unsigned long ms [PHASE_THR_TRACE_N];
    uint8_t       idx = 0;
    bool          any = false;
};

void thrTraceClear(ThrottleTrace& t) {
    for (uint8_t i = 0; i < PHASE_THR_TRACE_N; i++) { t.pct[i] = 0; t.ms[i] = 0; }
    t.idx = 0;
    t.any = false;
}

void thrTraceNote(ThrottleTrace& t, int pct, unsigned long now) {
    if (pct < 0)   return;
    if (pct > 100) pct = 100;
    t.pct[t.idx] = (uint8_t)pct;
    t.ms [t.idx] = now;
    t.idx = (uint8_t)((t.idx + 1) % PHASE_THR_TRACE_N);
    t.any = true;
}

// -1 כשאין אף ערך בתוך החלון
int thrTracePeak(const ThrottleTrace& t, unsigned long now, unsigned long windowMs) {
    int peak = -1;
    if (!t.any) return -1;
    for (uint8_t i = 0; i < PHASE_THR_TRACE_N; i++) {
        if (t.ms[i] == 0)                 continue;   // תא שעוד לא נכתב
        if (now - t.ms[i] > windowMs)     continue;   // מחוץ לחלון
        if ((int)t.pct[i] > peak)         peak = (int)t.pct[i];
    }
    return peak;
}

// INAV מנהל את הגובה בעצמו - אז לא מסיקים נחיתה ממצערת ומחיישן
static bool phNavMode(const FCStatus& fc) {
    if (!fc.modesFresh) return false;
    return (fc.modeFlags & (FM_NAV_ALTHOLD | FM_NAV_POSHOLD | FM_NAV_RTH |
                            FM_NAV_WP | FM_FAILSAFE)) != 0;
}

// E1022N: מצב שבו מותר מעבר מהיר ל-ANGLE בנגיעה - נחיתה ידנית של הטייס ב-POSHOLD / ALTHOLD / SURFACE.
//   לא ב-WP, ‏RTH, ‏FAILSAFE (שם INAV נוחת ומנתק בעצמו, או שמשימה פעילה). ‏SURFACE לבדו אינו תנאי: ב-28.10
//   כל הנחיתות הקופצות היו ב-0x33 (ALTHOLD+POSHOLD) אחרי יציאה אוטומטית מ-SURFACE, ורק 9 מ-178 תחילות נחיתה ביומנים
//   נרשמו עם ביט SURFACE.
static bool phFastModeOk(const FCStatus& fc) {
    if (!fc.modesFresh) return false;
    if (fc.modeFlags & (FM_NAV_WP | FM_NAV_RTH | FM_FAILSAFE)) return false;
    return (fc.modeFlags & (FM_NAV_ALTHOLD | FM_NAV_POSHOLD | FM_SURFACE)) != 0;
}

// E1022P: תנאי **שחרור** של fastTd. בזמן שהנעילה פעילה ה-ESP עצמו כופה ANGLE, ולכן INAV מדווח ANGLE בלי
//   ALTHOLD/POSHOLD (ביומן 01.11: FMD עבר 0x33 -> 0x03 כ-0.2s אחרי ההפעלה, והקוד פירש את זה כיציאה ממצב מותר
//   ושחרר את הנעילה אחרי 0.1-0.2s; INAV חזר ל-ALTHOLD עם z שגוי אחרי צניחת הברומטר והרחפן קפץ ~70cm).
//   לכן בשחרור בודקים רק מצבים אסורים (WP / RTH / FAILSAFE) ושהמצבים טריים; ההפעלה נשארת phFastModeOk.
static bool phFastHoldOk(const FCStatus& fc) {
    if (!fc.modesFresh) return false;
    return (fc.modeFlags & (FM_NAV_WP | FM_NAV_RTH | FM_FAILSAFE)) == 0;
}

// ═══════════════════════════════════════════════
//   קריאה אחת לכל תמונת מצב של החיישנים (20Hz)
//
//   relAltValid / relAltCm - הגובה היחסי לנקודת החימוש, מ-MSP_ALTITUDE.
//   משמש **רק** כגיבוי לזיהוי השלב ולאישור החיישן הקדמי: לא מוצג, לא נשלח
//   ל-FPGA, ולא נכנס לשום לולאת בקרה.
//
//   descentIntent  - ביט מפורש מהאפליקציה: מקל המצערת נדחף מטה.
//   lastThrPct     - ערך המצערת האחרון שהתקבל.
//   recentThrPeak  - השיא בחלון הקצר האחרון, -1 אם אין.
//                    הרצפה היא המרבי מבין השניים, כדי ששחרור רגע לפני
//                    דגימת ה-DWN לא ייצור רצפה נמוכה שאינה מגינה.
//   appAirborneAck - האפליקציה שידרה בשורת השליטה שהיא כבר יודעת AIRBORNE.
// ═══════════════════════════════════════════════
void updatePhase(PhaseState& p, const FCStatus& fc, const LidarData& lid,
                 bool relAltValid, int32_t relAltCm, bool descentIntent,
                 int lastThrPct, int recentThrPeak, bool appAirborneAck,
                 bool fastTdAllowed = false) {
    unsigned long now = millis();

    // בלי סטטוס אמין מה-FC אי אפשר לדעת אם מחומשים - לא נוגעים בשלב
    if (!fc.linkOK || !fc.armFresh) return;

    if (!fc.armed) {
        if (p.phase != PH_GROUND_DISARMED) p.phase = PH_GROUND_DISARMED;
        p.confirmRun      = 0;
        p.groundRun       = 0;
        p.altFallbackUsed = false;
        p.fwdActive       = false;
        p.nearGround      = false;
        p.navTouch        = false;
        p.fastTd          = false;
        phFastRun         = 0;
        phFastRunMs       = 0;
        phFastTcRaw       = 0;
        phFastRelSince    = 0;
        phFastLockUntil   = 0;
        p.navTdLastMs     = 0;
        phLastValidCm     = -1;
        phNavTdSince      = 0;
        phNavTdAirRun     = 0;
        p.thrFloor        = -1;
        p.floorAckWait    = false;
        p.floorAckByTmo   = false;
        phFwdSince        = 0;
        phAltSince        = 0;
        phLandSince       = 0;
        phAirborneSince   = 0;
        phTakeoffAltSince = 0;
        phClimbRun        = 0;
        phClimbSeenMs     = 0;
        phNearGroundMs    = 0;
        phNearGroundCm    = -1;
        phTcApproach      = false;
        phTcPrev          = false;
        return;
    }

    // דגימת DWN חדשה באמת? השורה נשלחת כל 50ms גם בלי מדידה חדשה
    bool newSample = !phSeqInit || (lid.dwnSeq != phLastDwnSeq);
    if (newSample) { phLastDwnSeq = lid.dwnSeq; phSeqInit = true; }

    bool dwnValid     = (lid.dwnSt == LID_VALID);
    phLastValidCm = lid.dwnRawCm;                           // E1022J (גולמי - dwn המסונן מפגר בירידה)
    bool aboveTakeoff = dwnValid && (lid.dwn > PHASE_TAKEOFF_CM);
    // עדות חיובית שאנחנו על הקרקע - הדבר היחיד שמבטל אישור המראה
    bool onGround     = (lid.dwnSt == LID_TOO_CLOSE) ||
                        (dwnValid && lid.dwn <= PHASE_LAND_CM);

    // ═══ זיכרון קרבה לקרקע ═══
    //   מתעדכן רק מדגימה תקינה. בטיסה גבוהה ה-DWN מחוץ לטווח וזה מצב תקין
    //   לחלוטין - הזיכרון פשוט מתיישן ואינו מעיד על דבר. הוא נדרש כדי
    //   שהשתתקות החיישן ממש מעל הקרקע תוכל לשמש עדות מגע, בלי שהשתתקות
    //   בגובה תיחשב לאותו דבר.
    if (newSample && dwnValid && lid.dwn <= PHASE_NEAR_GROUND_CM) {
        phNearGroundMs = now;
        phNearGroundCm = lid.dwn;
    }
    p.nearGround = (phNearGroundMs != 0) && (now - phNearGroundMs <= PHASE_NEAR_GROUND_MS);

    // ═══ TOO_CLOSE שמקורו בהתקרבות (E1021C) ═══
    //   נמצא בטיסה: דגימה שגויה בגובה 2.1m נעלה TOO_CLOSE ל-56 שנ'. TOO_CLOSE כזה
    //   (בלי מדידה תקינה קרובה לפניו) אינו עדות מגע: רק TOO_CLOSE שהתחיל כשהזיכרון
    //   "היינו באזור הקרקע" עוד טרי, וכל עוד הוא נמשך ברצף. כל מצב אחר מאפס.
    //   E1021D: וגם רק כל עוד יש מדידה גולמית טרייה שתומכת בו. TOO_CLOSE שנשמר רק בגלל
    //   נעילה (NO_RETURN / UNCERTAIN) יותר מ-PHASE_TC_FRESH_MS שובר את הרצף לצמיתות.
    bool tcFresh = (lid.dwnTcRawMs != 0) && (now - lid.dwnTcRawMs <= PHASE_TC_FRESH_MS);
    bool tcNow   = (lid.dwnSt == LID_TOO_CLOSE);
    if (!tcNow || !tcFresh) phTcApproach = false;
    else if (!phTcPrev)     phTcApproach = p.nearGround;
    phTcPrev = tcNow && tcFresh;

    // ═══ עדות טיפוס אמיתי ═══
    //   הברומטר של INAV נודד - נצפתה נדידה של יותר מחצי מטר ברחפן שעומד
    //   לגמרי במקום. לכן גובה לבדו אינו יכול להעיד על המראה. המהירות האנכית
    //   כן: היא נגזרת, ולכן סחיפה איטית של הלחץ לא מייצרת אותה.
    //
    //   העדות נשמרת לזמן קצר (PHASE_CLIMB_MEMORY_MS) ולא נדרשת ברגע ההחלטה
    //   עצמו: עד שהגובה יציב כבר PHASE_TAKEOFF_ALT_MS, הטיפוס עשוי היה
    //   להסתיים ולהפוך לריחוף - וזו עדיין הייתה המראה.
    //   ⚠️ נספרות דגימות **חדשות** של MSP_ALTITUDE לפי המונה `altSeq`, ולא זמן.
    //      הסקר הוא 4Hz ומכונת השלבים רצה ב-20Hz: ספירה לפי זמן הייתה סופרת
    //      את אותה דגימה שמורה חמש פעמים, ודגימה חולפת אחת הייתה מספיקה כדי
    //      להיראות כמו טיפוס רצוף. אותו עיקרון בדיוק כמו FSEQ/DSEQ בלידאר.
    bool newAltSample = fc.altFresh && (!phAltSeqInit || fc.altSeq != phLastAltSeq);
    if (!fc.altFresh) {
        phClimbRun = 0;                      // אין נתוני גובה - הרצף נקטע
    } else if (newAltSample) {
        phLastAltSeq = fc.altSeq;
        phAltSeqInit = true;
        if (fc.vspdCms >= PHASE_TAKEOFF_VSPD_CMS) {
            if (phClimbRun < 255) phClimbRun++;
            if (phClimbRun >= PHASE_TAKEOFF_CLIMB_SAMPLES) phClimbSeenMs = now;
        } else {
            phClimbRun = 0;                  // דגימה שאינה טיפוס קוטעת את הרצף
        }
    }
    bool climbRecent = (phClimbSeenMs != 0) && (now - phClimbSeenMs <= PHASE_CLIMB_MEMORY_MS);

    // מצערת שמתאימה להמראה. נדרשת בשני מסלולי הגיבוי של INAV - גם בכניסה
    // מ-GROUND_ARMED וגם באישור מ-TAKEOFF_CONFIRM - ולכן מחושבת כאן.
    bool thrTakeoff = (recentThrPeak >= PHASE_TAKEOFF_THR_PCT) ||
                      (lastThrPct    >= PHASE_TAKEOFF_THR_PCT);

    switch (p.phase) {

        case PH_GROUND_DISARMED:
            p.phase      = PH_GROUND_ARMED;     // INAV מדווח מחומש
            p.confirmRun = 0;
            p.groundRun  = 0;
            break;

        // ─── מחומש על הקרקע: שחרור הסטיק = מצערת מינימלית, INAV מסובב בסרק ───
        //
        //   ⚠️ שני מסלולים בלתי תלויים לזיהוי המראה, ולא באותה רמת ראיה:
        //     · DWN תקין מעל סף ההמראה - מדידה ישירה, ולכן מספיקה לבדה.
        //     · מסלול INAV - **אומדן**, ולכן דורש שלוש עדויות יחד: גובה יציב,
        //       טיפוס אמיתי לפי המהירות האנכית, ומצערת שמתאימה להמראה.
        //       ראו את הנימוק בגוף ה-case.
        //
        //   בלי המקור השני, דגימה חסרה של ה-DWN משאירה אותנו ב"קרקע" בזמן
        //   שהרחפן כבר באוויר. אז האפליקציה ממפה את הסטיק סביב מרכז הקרקע,
        //   כלומר שחרור המקל מצווה מצערת אפס על רחפן שטס. זו בדיוק הסיבה
        //   שהחיישן התחתון אינו רשאי להיות נקודת כשל יחידה.
        case PH_GROUND_ARMED: {
            bool dwnEvidence = newSample && aboveTakeoff;

            // ⚠️ מסלול ה-INAV דורש **שלוש** עדויות בלתי תלויות, ולא גובה לבדו:
            //    גובה יציב, טיפוס אמיתי שנמדד, ומצערת שמתאימה להמראה.
            //    סחיפת ברומטר - שראינו אצלנו מעל חצי מטר ברחפן עומד - מספקת
            //    את הראשונה בלבד, ולכן אינה יכולה לזייף המראה.
            bool altEvidence = false;
            if (relAltValid && relAltCm >= PHASE_TAKEOFF_ALT_CM) {
                if (phTakeoffAltSince == 0) phTakeoffAltSince = now;
                altEvidence = (now - phTakeoffAltSince >= PHASE_TAKEOFF_ALT_MS) &&
                              climbRecent && thrTakeoff;
            } else {
                phTakeoffAltSince = 0;          // ירד מתחת לסף - הרצף נקטע
            }

            if (dwnEvidence || altEvidence) {
                // ⚠️ הרצפה נקבעת מהמרבי בין החבילה האחרונה לשיא הקצר. הטייס
                //    יכול היה להרפות מעט בדיוק לפני הדגימה שהפעילה את השלב,
                //    ואז החבילה האחרונה לבדה אינה מייצגת את מצערת ההמראה.
                int floorPct = lastThrPct;
                if (recentThrPeak > floorPct) floorPct = recentThrPeak;

                p.thrFloor      = (floorPct >= 0) ? floorPct : -1;
                p.groundRun     = 0;
                p.floorAckByTmo = false;
                phAltSince      = 0;

                if (dwnEvidence) {
                    p.phase        = PH_TAKEOFF_CONFIRM;
                    p.confirmRun   = 1;
                    p.floorAckWait = false;
                } else {
                    // עדות הגובה כבר חזקה מתנאי האישור הפנימי - 45cm למשך
                    // 700ms מול 28cm למשך 500ms - ולכן אין מה לאשר. נכנסים
                    // ישירות ל-AIRBORNE, עם רצפת המצערת ועם ההמתנה לאישור
                    // האפליקציה, בדיוק כמו בכל מעבר אחר.
                    p.phase           = PH_AIRBORNE;
                    p.confirmRun      = 0;
                    p.altFallbackUsed = true;
                    p.floorAckWait    = (p.thrFloor >= 0);
                    phAirborneSince   = now;
                    phLandSince       = 0;
                }
            }
            break;
        }

        // ─── עדות ראשונה להמראה התקבלה, ממתינים לאישור ───
        //   כאן פועלת רצפת המצערת: ערך נמוך מהנשמר נחתך לפני השליחה ל-FPGA.
        //   אין פסק זמן. יציאה מהמצב רק בעדות חיובית לכאן או לכאן.
        case PH_TAKEOFF_CONFIRM: {
            if (newSample) {
                // ⚠️ כל דגימה חדשה מאפסת את הרצף הנגדי, וכל דגימה שאינה תומכת
                //    באף כיוון מאפסת את שניהם. זו המשמעות של "דגימות רצופות".
                if (aboveTakeoff) {
                    p.groundRun = 0;
                    if (p.confirmRun < 255) p.confirmRun++;
                    if (p.confirmRun >= PHASE_TAKEOFF_SAMPLES) {
                        p.phase         = PH_AIRBORNE;
                        p.floorAckWait  = (p.thrFloor >= 0);
                        p.floorAckByTmo = false;
                        phAirborneSince = now;
                        phLandSince     = 0;
                        break;
                    }
                }
                else if (onGround) {
                    // עדות חיובית שאנחנו על הקרקע - כנראה קריאה חולפת
                    p.confirmRun = 0;
                    if (p.groundRun < 255) p.groundRun++;
                    if (p.groundRun >= PHASE_CONFIRM_GROUND_SAMPLES) {
                        p.phase        = PH_GROUND_ARMED;
                        p.thrFloor     = -1;
                        p.floorAckWait = false;
                        p.groundRun    = 0;
                        break;
                    }
                }
                else {
                    // מדידה תקינה בין הספים, NO_RETURN, UNCERTAIN או FAULT:
                    // אף אחד מהם אינו ראיה, ולכן אף אחד מהם לא ממשיך רצף קיים.
                    // ⚠️ איפוס הרצף **אינו** חזרה לקרקע: נשארים כאן, עם רצפת
                    //    המצערת פעילה, עד שתגיע עדות חיובית לכיוון כלשהו.
                    p.confirmRun = 0;
                    p.groundRun  = 0;
                }
            }

            // ─── גיבוי: ה-DWN יצא מהטווח או הפסיק להחזיר קריאה ───
            //
            //   ⚠️ גם כאן גובה לבדו אינו מספיק. נכון שכדי להגיע לשלב הזה
            //      כבר נדרשה קריאת DWN מעל סף ההמראה, אבל קריאה שגויה אחת
            //      יחד עם סחיפת ברומטר עדיין הייתה יכולה להשלים זיהוי המראה
            //      שקרי. לכן אותן שתי עדויות שנדרשות בכניסה מ-GROUND_ARMED
            //      נדרשות גם באישור כאן: טיפוס שנמדד, ומצערת שמתאימה להמראה.
            if (relAltValid && relAltCm >= PHASE_ALT_FALLBACK_CM &&
                climbRecent && thrTakeoff) {
                if (phAltSince == 0) phAltSince = now;
                if (now - phAltSince >= PHASE_ALT_STABLE_MS) {
                    p.phase           = PH_AIRBORNE;
                    p.altFallbackUsed = true;
                    p.floorAckWait    = (p.thrFloor >= 0);
                    p.floorAckByTmo   = false;
                    phAirborneSince   = now;
                    phLandSince       = 0;
                }
            } else {
                phAltSince = 0;
            }
            break;
        }

        // ─── באוויר: רק כאן שחרור הסטיק מחזיר למרכז הריחוף הידני ───
        case PH_AIRBORNE: {
            // ═══ שחרור רצפת המצערת - רק כשידוע שהאפליקציה כבר באותו עמוד ═══
            //   ה-ESP קובע AIRBORNE מיידית, אבל האפליקציה מקבלת את השלב רק
            //   בטלמטריה של 10Hz. בחלון הזה היא עדיין ממפה את הסטיק סביב
            //   מרכז הקרקע, ולכן חבילה נמוכה יכולה לצאת ממנה. הרצפה נשארת
            //   פעילה עד שהיא מחזירה PHA=3, כלומר הוכחה שהיא כבר יודעת.
            //
            //   ⚠️ הגיבוי בזמן אינו "היעדר מידע כראיה": הוא לא מסיק שום מצב
            //      פיזי, אלא רק מוותר על הגבלה. בלעדיו, אפליקציה שאינה מכירה
            //      את השדה הייתה נועלת את הרחפן במצערת שאי אפשר להוריד ממנה.
            if (p.floorAckWait) {
                if (appAirborneAck) {
                    p.thrFloor     = -1;
                    p.floorAckWait = false;
                } else if (now - phAirborneSince >= PHASE_AIRBORNE_ACK_MS) {
                    p.thrFloor      = -1;
                    p.floorAckWait  = false;
                    p.floorAckByTmo = true;
                }
            }

            // ═══ מגע קרקע - הצטלבות עדויות, לא חיישן יחיד ═══
            //
            //   ⚠️ ה-DWN מפסיק למדוד מתחת לטווח המינימלי שלו, ולכן "החיישן
            //      השתתק" אינו נחיתה בפני עצמו: אותו דבר קורה כשהוא יוצא
            //      מהטווח כלפי מעלה וגם כשהוא נכשל. מה שהופך אותו לעדות הוא
            //      **הרצף** - 20cm, 12cm, 8cm, 5cm ואז שקט - בזמן שהטייס
            //      מוריד מצערת ומצהיר על ירידה.
            //
            //   לכן המגע מחייב את כל אלה יחד:
            //     · לא במצב ניווט - שם INAV נוחת ומנתק בעצמו
            //     · כוונת ירידה מפורשת מהאפליקציה
            //     · מצערת נמוכה - מי שמרחף אינו נוחת
            //     · עדות קרקע: מדידה תקינה מתחת לסף הנחיתה או TOO_CLOSE,
            //       או שהחיישן השתתק **מיד אחרי** שראה את אזור הקרקע
            //     · והכול יציב PHASE_TOUCHDOWN_MS
            bool dwnSilent   = !dwnValid && (lid.dwnSt != LID_TOO_CLOSE);
            bool thrLow      = (lastThrPct >= 0) && (lastThrPct <= PHASE_TOUCHDOWN_THR_PCT);

            // ⚠️ מסלול "החיישן השתתק" מחייב שהמדידה התקינה **האחרונה** לפני
            //    השתיקה תהיה ממש על הקרקע, וגם שהמהירות האנכית כבר אפסית.
            //    בלי שני אלה, רצף כמו 55cm ואז שקט היה נספר כנחיתה בזמן
            //    שהרחפן עוד באוויר. הרצף שאנחנו מחפשים הוא 20 → 12 → 8 → 5
            //    ואז שקט, כשהרחפן כבר לא יורד.
            int  vspdAbs     = (fc.vspdCms < 0) ? -fc.vspdCms : fc.vspdCms;
            bool vspdStill   = fc.altFresh && (vspdAbs <= PHASE_TOUCHDOWN_VSPD_CMS);
            bool lastWasLow  = (phNearGroundCm >= 0) &&
                               (phNearGroundCm <= PHASE_TOUCHDOWN_LAST_CM);

            // E1021C: TOO_CLOSE נחשב רק כשמקורו בהתקרבות, ובנוסף בדיקת סבירות מול INAV:
            //   INAV גבוה מ-PHASE_TOUCHDOWN_ALT_MAX_CM ⇒ אין מגע. ליד הקרקע הברומטר מראה
            //   נמוך מהאמת, ולכן במגע אמיתי זה תמיד מתקיים - והוא לבדו גם אינו הוכחה.
            bool tdGround    = (tcNow && phTcApproach) ||
                               (dwnValid && lid.dwn <= PHASE_LAND_CM);
            bool altPlausible = !relAltValid || relAltCm <= PHASE_TOUCHDOWN_ALT_MAX_CM;
            bool groundProof = altPlausible &&
                               (tdGround || (dwnSilent && p.nearGround && lastWasLow && vspdStill));

            // E1022P: בזמן ש-fastTd נעול ה-ESP כופה ANGLE ו-INAV מדווח ANGLE; זה עדיין נחיתה במצב ניווט (POSHOLD/ALTHOLD),
            //   ולכן לא עוברים למסלול ה-ANGLE הישן (600ms -> GROUND_ARMED), שהיה משחרר את הנעילה ומחזיר ALTHOLD על הקרקע.
            bool navLike = phNavMode(fc) || p.fastTd;
            if (!navLike && descentIntent && thrLow && groundProof) {
                if (phLandSince == 0) phLandSince = now;
                if (now - phLandSince >= PHASE_TOUCHDOWN_MS) {
                    p.phase        = PH_GROUND_ARMED;
                    p.confirmRun   = 0;
                    p.groundRun    = 0;
                    p.thrFloor     = -1;
                    p.floorAckWait = false;
                    p.nearGround   = false;
                    phLandSince    = 0;
                    phNearGroundMs = 0;
                    phNearGroundCm = -1;
                }
            } else {
                phLandSince = 0;
            }

            // ═══ E1022G: אותה הוכחת מגע גם במצב ניווט (SURFACE) - בלי לשנות את השלב ═══
            //   ב-POSHOLD השלב לא עובר ל-GROUND_ARMED (INAV נוחת בעצמו), אבל ב-SURFACE
            //   הטייס נוחת ידנית. האפליקציה צריכה מגע **מאומת** כדי לנעול 0%.
            // E1022M: במצב ניווט מסלול "החיישן השתתק" תקף רק כל עוד מדידת DOWN **תקינה** טרייה
            //   (PHASE_NAVTD_SILENT_MS). זיכרון הקרבה (PHASE_NEAR_GROUND_MS, 1.5 שנ') לבדו לא מספיק: מדידה
            //   של 8cm, התרוממות ושתיקת החיישן נראו אותו דבר כמו נחיתה והשלימו 600ms (770ms ללא מדידה חדשה).
            //   נחיתה אמיתית עוברת דרך tdGround (TOO_CLOSE עם התקרבות / DWN ≤ 5cm): 35/35 ביומנים האמיתיים.
            //   אחרי ש-navTouch נקבע, שתיקה על הקרקע לא מוחקת אותו (ללא שינוי).
            bool navSilentFresh = (phNearGroundMs != 0) && (now - phNearGroundMs <= PHASE_NAVTD_SILENT_MS);
            bool groundProofNav = altPlausible &&
                                  (tdGround || (dwnSilent && p.nearGround && navSilentFresh && lastWasLow && vspdStill));
            if (navLike && descentIntent && thrLow && groundProofNav) {
                if (phNavTdSince == 0) phNavTdSince = now;
                p.navTdLastMs = now;
                if (now - phNavTdSince >= PHASE_TOUCHDOWN_MS) p.navTouch = true;
            } else {
                phNavTdSince = 0;
            }

            // ═══ E1022N: מסלול מהיר - אותה הוכחה, PHASE_FASTTD_SAMPLES דגימות קרקע חדשות במקום 600ms ═══
            //   הדגימות חייבות להיות **רצופות**: כל דגימה חדשה שאינה עדות קרקע (כולל NO_RETURN) מאפסת את המונה.
//   "דגימת קרקע" = דגימה חדשה שהיא עצמה עדות: DWN תקין ≤ PHASE_LAND_CM, או דגימה גולמית חדשה של
            //   TOO_CLOSE אחרי התקרבות (dwnTcRawMs השתנה). נעילת TOO_CLOSE שנשארת על NO_RETURN לא נספרת.
            //   ההפעלה מחייבת גם מדידה גולמית מאומתת אחרונה ≤ PHASE_NAVTD_EXIT_LAST_CM (בנחיתות אמיתיות 4-12cm;
            //   ריחוף ב-20-45cm עם קריאות 0mm שגויות לא עונה על זה) ו-INAV ≤ PHASE_TOUCHDOWN_ALT_MAX_CM.
            bool fastLastLow = phLastValidCm >= 0 && phLastValidCm <= PHASE_NAVTD_EXIT_LAST_CM;
            if (!p.fastTd && fastTdAllowed && phFastModeOk(fc) && descentIntent && thrLow &&
                (phFastLockUntil == 0 || now >= phFastLockUntil)) {
                if (phFastRun != 0 && now - phFastRunMs > PHASE_FASTTD_WINDOW_MS) phFastRun = 0;
                if (newSample) {
                    bool gSample = false;
                    if (dwnValid && lid.dwn <= PHASE_LAND_CM) gSample = true;
                    else if (tcNow && phTcApproach && lid.dwnTcRawMs != 0 && lid.dwnTcRawMs != phFastTcRaw) {
                        gSample = true;
                        phFastTcRaw = lid.dwnTcRawMs;
                    }
                    if (gSample && altPlausible && fastLastLow) {
                        phFastRun = (uint8_t)(phFastRun + (phFastRun < 255));
                        phFastRunMs = now;
                    } else {
                        phFastRun = 0;             // E1022N: כל דגימה חדשה שאינה עדות קרקע שוברת את הרצף
                    }
                }
                if (phFastRun >= PHASE_FASTTD_SAMPLES) {
                    p.fastTd       = true;
                    p.navTouch     = true;          // אותה הוכחה ⇒ האפליקציה נועלת 0% ו-THS=-1 (SURFACE יוצא)
                    p.navTdLastMs  = now;
                    phFastRelSince = 0;
                }
            } else if (!p.fastTd) {
                phFastRun = 0;
            }
            break;
        }
    }

    // E1022G: navTouch נמחק רק בעדות לאוויר (ולא בשתיקת DWN או בשחרור המקל)
    if (p.navTouch) {
        if (newSample) phNavTdAirRun = aboveTakeoff ? (uint8_t)(phNavTdAirRun + (phNavTdAirRun < 255)) : 0;
        if (phNavTdAirRun >= PHASE_NAVTD_CLEAR_SAMPLES ||
            (relAltValid && relAltCm > PHASE_TOUCHDOWN_ALT_MAX_CM)) {
            p.navTouch    = false;
            phNavTdSince  = 0;
        }
    }
    if (!p.navTouch) phNavTdAirRun = 0;
    // E1022N: שחרור הנעילה המהירה - ביטול navTouch (עדות לאוויר), יציאה מ-AIRBORNE, או מקל למעלה
    if (p.fastTd) {
        if (lastThrPct >= PHASE_FASTTD_RELEASE_PCT) { if (phFastRelSince == 0) phFastRelSince = now; }
        else phFastRelSince = 0;
        bool stickRel = phFastRelSince != 0 && now - phFastRelSince >= PHASE_FASTTD_RELEASE_MS;
        if (!p.navTouch || p.phase != PH_AIRBORNE || stickRel || !fastTdAllowed || !phFastHoldOk(fc)) {
            p.fastTd        = false;
            phFastRun       = 0;
            phFastRelSince  = 0;
            phFastLockUntil = now + PHASE_FASTTD_LOCK_MS;
            if (phFastLockUntil == 0) phFastLockUntil = 1;
        }
    }
    // E1022I: עדות לאוויר מבטלת גם הוכחה שעוד לא הושלמה
    if ((newSample && aboveTakeoff) || (relAltValid && relAltCm > PHASE_TOUCHDOWN_ALT_MAX_CM)) p.navTdLastMs = 0;

    // ═══ האם מותר להשתמש בחיישן הקדמי להחלטות מכשול ═══
    //   על הקרקע ובגובה נמוך החיישן רואה רצפה, דשא או משטח משופע: הוא מותקן
    //   2-3cm מעל הרצפה ושדה הראייה שלו כ-27°. המדידה ממשיכה לאבחון, אבל
    //   היא לא תפעיל לוגיקת מכשולים.
    //
    //   ⚠️ NO_RETURN של DWN **אינו** עדות לגובה. הוא מתקבל גם מדשא, ממשטח
    //      כהה ומזווית פגיעה גרועה, כלומר גם בגובה נמוך מאוד. לכן העדות
    //      לגובה היא מדידת DWN תקינה, או הגובה היחסי של INAV - ו-NO_RETURN
    //      לבדו נחשב היעדר מידע שאינו משנה מצב לאף כיוון.
    if (p.phase != PH_AIRBORNE) {
        p.fwdActive = false;
        phFwdSince  = 0;
    } else {
        bool hiEvidence = (dwnValid && lid.dwn >= PHASE_FWD_ACTIVE_CM) ||
                          (relAltValid && relAltCm >= PHASE_FWD_ALT_CM);
        bool loEvidence = (lid.dwnSt == LID_TOO_CLOSE) ||
                          (dwnValid && lid.dwn < PHASE_FWD_ACTIVE_CM);

        if (hiEvidence) {
            if (phFwdSince == 0) phFwdSince = now;
            if (now - phFwdSince >= PHASE_FWD_ACTIVE_MS) p.fwdActive = true;
        } else if (loEvidence) {
            p.fwdActive = false;
            phFwdSince  = 0;
        }
        // אחרת (כולל NO_RETURN): אין מידע - משאירים את המצב כפי שהוא
    }
}

// ═══════════════════════════════════════════════
//   אכיפת רצפת המצערת על שורת שליטה לפני שהיא ממשיכה ל-FPGA.
//   מחליפה רק את הערך שאחרי "THR:" ומשאירה את שאר השורה כמו שהיא.
// ═══════════════════════════════════════════════
// E1022I: נקרא ברגע שה-ESP מפסיק לבקש SURFACE (1 ⇒ 0), **לפני** שה-THS של אותה שורה נקבע.
//   אם הוכחת המגע התקיימה ממש עכשיו - זו נחיתה שהיציאה קטעה את אימותה: navTouch מיד, כדי
//   שה-FPGA ייצא בלי החזקת מרכז והאפליקציה תנעל 0%.
void phaseSurfExit(PhaseState& p, unsigned long now) {
    if (!p.navTouch && p.navTdLastMs != 0 && now - p.navTdLastMs <= PHASE_NAVTD_EXIT_MS &&
        phLastValidCm >= 0 && phLastValidCm <= PHASE_NAVTD_EXIT_LAST_CM)            // E1022J
        p.navTouch = true;
}

// E1022N: שורת שליטה במצב "נגיעה מהירה": MODE:ANGLE ומצערת 0 (THR, THH ו-TH2 נקבעים לאפס ע"י הקורא)
String phaseForceAngleIdle(const String& line) {
    String out = line;
    int m = out.indexOf("MODE:");
    if (m >= 0) {
        int e = out.indexOf(' ', m);
        out = out.substring(0, m) + "MODE:ANGLE" + (e >= 0 ? out.substring(e) : String(""));
    } else {
        out += " MODE:ANGLE";
    }
    const char* keys[] = { "THR:", "THH:" };
    for (const char* k : keys) {
        int i = out.indexOf(k);
        if (i < 0) continue;
        int e = out.indexOf(' ', i);
        out = out.substring(0, i) + k + "0" + (e >= 0 ? out.substring(e) : String(""));
    }
    return out;
}

String applyThrottleFloor(const String& line, int floorPct) {
    if (floorPct < 0) return line;
    int i = line.indexOf("THR:");
    if (i < 0) return line;

    int j = i + 4;
    int k = j;
    while (k < (int)line.length() && isDigit(line[k])) k++;
    if (k == j) return line;                       // אין מספר - לא נוגעים

    if (line.substring(j, k).toInt() >= floorPct) return line;
    return line.substring(0, j) + String(floorPct) + line.substring(k);
}
