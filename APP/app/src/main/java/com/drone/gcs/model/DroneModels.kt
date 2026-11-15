package com.drone.gcs.model

import kotlin.math.roundToInt

// ---------------------------------------------------------------------------
// סטטוס רכיבים - 0=תקלה 1=תקין 2=לא ידוע
// ---------------------------------------------------------------------------
enum class CheckState { OK, FAIL, UNKNOWN }

data class SystemChecks(
    val fpga:     CheckState = CheckState.UNKNOWN,
    val fcLink:   CheckState = CheckState.UNKNOWN,
    val imu:      CheckState = CheckState.UNKNOWN,
    val battery:  CheckState = CheckState.UNKNOWN,
    val gps:      CheckState = CheckState.UNKNOWN,
    val lidarFwd: CheckState = CheckState.UNKNOWN,
    val lidarDwn: CheckState = CheckState.UNKNOWN,
) {
    // בדיקות החובה לחימוש בטיסה ידנית (ANGLE).
    // GPS לא ברשימה: הוא נדרש רק למצבי ניווט, ולכן הוא מוצג אבל לא חוסם.
    fun blockingReason(): String? = when {
        fpga     != CheckState.OK   -> "FPGA LINK"
        fcLink   != CheckState.OK   -> "FC LINK"
        imu      != CheckState.OK   -> "IMU / ATT"
        battery  != CheckState.OK   -> "BATTERY"
        lidarFwd == CheckState.FAIL -> "LIDAR FRONT"
        lidarDwn == CheckState.FAIL -> "LIDAR DOWN"
        else                        -> null
    }

    val ready: Boolean get() = blockingReason() == null

    companion object {
        private fun c(ch: Char) = when (ch) {
            '1'  -> CheckState.OK
            '0'  -> CheckState.FAIL
            else -> CheckState.UNKNOWN
        }

        fun parse(s: String): SystemChecks {
            if (s.length < 7) return SystemChecks()
            return SystemChecks(
                fpga     = c(s[0]),
                fcLink   = c(s[1]),
                imu      = c(s[2]),
                battery  = c(s[3]),
                gps      = c(s[4]),
                lidarFwd = c(s[5]),
                lidarDwn = c(s[6])
            )
        }
    }
}

// ---------------------------------------------------------------------------
// מצב חיישן LiDAR - FOK / DOK, זהה ל-ESP ול-FPGA
// ---------------------------------------------------------------------------
object LidarState {
    const val FAULT     = 0     // לא עונה / אין דגימה 300ms
    const val VALID     = 1     // מרחק אמין
    const val NO_RETURN = 2     // אין מטרה בטווח
    const val UNCERTAIN = 3     // החזר לא אמין או קפיצה שעוד לא אושרה
    const val TOO_CLOSE = 4     // מתחת לטווח המינימלי
}

// ---------------------------------------------------------------------------
// טווחי הלידאר לפי מצב המדידה, וצבע לפי שלישים של הטווח השמיש
//
//   הקיזוז של FRONT (18cm מהחיישן עד קצה הפרופלור) מוחל **פעם אחת** ב-ESP,
//   והמרחק שמגיע לכאן כבר מקצה הפרופלור. כאן הוא משמש רק כדי לדעת מהו
//   המרחק המרבי שיכול להופיע: הטווח של המצב פחות הקיזוז.
//   DOWN נמדד מתחתית הרחפן (קיזוז 1cm). זה מרחק למשטח שמתחת, לא גובה.
// ---------------------------------------------------------------------------
object LidarRange {
    const val SHORT_CM     = 130
    const val LONG_CM      = 400
    const val FWD_MOUNT_CM = 18
    const val DWN_MOUNT_CM = 1

    enum class Band { NEAR, MID, FAR }

    // ── שלישים עם היסטרזיס: מעבר לשליש אחר רק כשהמרחק עבר את הגבול ב-HYST_CM ──
    //   כך מרחק שנמצא בדיוק על הגבול לא מקפיץ את הצבע הלוך ושוב.
    const val HYST_CM = 4

    fun bandHyst(cm: Int, maxCm: Int, mountCm: Int, prev: Band?): Band {
        val b = band(cm, maxCm, mountCm)
        if (prev == null || b == prev) return b
        val usable = (maxCm - mountCm).coerceAtLeast(3)
        val t1 = usable / 3f; val t2 = usable * 2f / 3f
        val ok = when {
            b.ordinal > prev.ordinal -> cm >= (if (b == Band.MID) t1 else t2) + HYST_CM
            else                     -> cm <= (if (b == Band.MID) t2 else t1) - HYST_CM
        }
        return if (ok) b else prev
    }

    // השליש שבו המרחק נמצא, מתוך הטווח השמיש בפועל
    fun band(cm: Int, maxCm: Int, mountCm: Int): Band {
        val usable = (maxCm - mountCm).coerceAtLeast(3)
        return when {
            cm * 3 < usable     -> Band.NEAR
            cm * 3 < usable * 2 -> Band.MID
            else                -> Band.FAR
        }
    }
}

// ---------------------------------------------------------------------------
// שלב הטיסה - מחושב ב-ESP32 ומגיע בטלמטריה בשדה PHS.
//
// האפליקציה **לא** מחשבת אותו: רק ה-ESP מחזיק בו-זמנית את מצב החימוש של INAV,
// את חיישן המרחק התחתון ואת אומדן הגובה הפנימי, ולכן הוא זה שקובע.
//
//   ARMED  ≠ AIRBORNE      חימוש פירושו שהמנועים חיים, לא שהרחפן באוויר
//   ANGLE  ≠ AIRBORNE      מצב טיסה הוא הגדרת ייצוב, לא מצב פיזי
//
// TAKEOFF_CONFIRMING הוא מצב הביניים שסוגר את החור המסוכן: מרגע שיש עדות
// ראשונה להמראה ועד האישור, שחרור מקל המצערת **אינו** מחזיר למינימום אלא
// מחזיק את הערך האחרון שהטייס נתן.
//
// שלושה מרכזי מצערת שונים לחלוטין, ואסור לערבב ביניהם:
//   Motor Idle          - INAV, כשהמנועים מחומשים על הקרקע
//   Manual Hover Center - האפליקציה, רק ב-AIRBORNE ובטיסה ידנית
//   ALTHOLD Neutral     - INAV, רק במצב שמירת גובה
// ---------------------------------------------------------------------------
enum class FlightPhase(val code: Int, val label: String, val hint: String) {
    GROUND_DISARMED(0, "GROUND",   "DISARMED"),
    GROUND_ARMED   (1, "GROUND",   "ARMED - MOTORS IDLE"),
    TAKEOFF_CONFIRM(2, "TAKEOFF",  "CONFIRMING - HOLDING THROTTLE"),
    AIRBORNE       (3, "AIRBORNE", "HOVER CENTER ACTIVE");

    // באוויר או בדרך לשם: בשני המצבים אסור להחזיר מצערת למינימום בשחרור המקל
    val offGround: Boolean get() = this == TAKEOFF_CONFIRM || this == AIRBORNE

    companion object {
        fun from(code: Int): FlightPhase = entries.firstOrNull { it.code == code } ?: GROUND_DISARMED
    }
}

// ---------------------------------------------------------------------------
// רשות השליטה - האם מותר לאפליקציה לשדר שורות שליטה בכלל
//
// ⚠️ זהו **לא** מצב הקישור. הקישור יכול להיות תקין לחלוטין בזמן שאסור לשדר:
//    מיד אחרי פתיחת האפליקציה, לפני שהתקבלה טלמטריה כלשהי, אין לאפליקציה
//    מושג מה מצב הרחפן - והערכים ההתחלתיים שלה הם אפסים. שידור שלהם לרחפן
//    שנמצא באוויר פירושו מצערת אפס.
//
//    לכן ברירת המחדל היא לא לשדר, ופתיחת השליטה מחייבת עדות חיובית:
//      WAITING    אין עדיין טלמטריה. נשלח רק APP_PING.
//      ACTIVE     התקבלה טלמטריה שמראה מצב קרקע בטוח, או שהטייס אישר חזרה.
//      SUSPENDED  הרחפן מחומש או באוויר ואנחנו לא יודעים מה נשלח לו קודם.
//                 חזרה לשליטה מחייבת אישור מפורש, עם הערך הידוע ולא עם אפס.
//      LOST       הקשר נפל בזמן שליטה. ה-FPGA כבר עצר את זרם ה-RC ונשאר נעול
//                 עד DISARM, ולכן אין טעם ואין היתר לחזור לשדר מעצמנו.
// ---------------------------------------------------------------------------
enum class ControlAuthority(val label: String, val hint: String) {
    WAITING  ("CONTROL IDLE",      "WAITING FOR TELEMETRY - NO CONTROL SENT"),
    ACTIVE   ("CONTROL ACTIVE",    "CONTROL STREAM RUNNING"),
    SUSPENDED("CONTROL SUSPENDED", "DRONE IS ARMED - EXPLICIT RESUME REQUIRED"),
    LOST     ("CONTROL STOPPED",   "LINK LOST - DISARM TO CLEAR");

    val streaming: Boolean get() = this == ACTIVE
}

// ---------------------------------------------------------------------------
// מצב הטיסה בפועל ב-INAV - שדה FMD מה-ESP.
// אלה ביטים שלנו: ה-ESP ממפה אותם מ-MSP2_INAV_STATUS לפי MSP_BOXIDS,
// כך שמיקום הביטים של INAV לא מקובע בשום מקום.
//
// ⚠️ זה מה ש-INAV **מבצע**, לא מה שביקשנו. כל החלטה על התנהגות המצערת
//    נגזרת מכאן בלבד ולעולם לא מ-FlightMode שנבחר בממשק.
// ---------------------------------------------------------------------------
object InavMode {
    const val ARM         = 1 shl 0
    const val ANGLE       = 1 shl 1
    const val HORIZON     = 1 shl 2
    const val MANUAL      = 1 shl 3
    const val NAV_ALTHOLD = 1 shl 4
    const val NAV_POSHOLD = 1 shl 5
    const val NAV_RTH     = 1 shl 6
    const val NAV_WP      = 1 shl 7
    const val NAV_COURSE  = 1 shl 8
    const val NAV_CRUISE  = 1 shl 9
    const val FAILSAFE    = 1 shl 10
    const val SURFACE     = 1 shl 11    // E1022A: SURFACE **בפועל** (הדגל הפנימי של INAV)

    // מצבים שבהם INAV כבר לא מתייחס למצערת כאל "כמה כוח למנועים":
    // שמירת גובה ומיקום (המצערת = פקודת טיפוס/ירידה), RTH ו-WP (INAV שולט בגובה) ו-Failsafe.
    // COURSE HOLD ו-CRUISE לא בפנים: הם לא בהכרח שולטים בגובה.
    const val NAV_THROTTLE_MASK = NAV_ALTHOLD or NAV_POSHOLD or NAV_RTH or NAV_WP or FAILSAFE

    fun label(flags: Int, valid: Boolean): String = when {
        !valid                         -> "---"
        (flags and FAILSAFE)    != 0   -> "FAILSAFE"
        (flags and NAV_RTH)     != 0   -> "RTH"
        (flags and NAV_WP)      != 0   -> "WP"
        (flags and NAV_POSHOLD) != 0   -> "POSHOLD"
        (flags and NAV_CRUISE)  != 0   -> "CRUISE"
        (flags and NAV_COURSE)  != 0   -> "COURSE"
        (flags and NAV_ALTHOLD) != 0   -> "ALTHOLD"
        (flags and MANUAL)      != 0   -> "MANUAL"
        (flags and HORIZON)     != 0   -> "HORIZON"
        (flags and ANGLE)       != 0   -> "ANGLE"
        else                           -> "ACRO"
    }
}

// ═══════════════════════════════════════════════════════════
//   SURFACE (E1022A) - המקל הוא **גובה** מעל הקרקע, לא קצב
//
//   INAV 9.1 (navigation_multicopter.c): altTarget = scaleRange(rcCommand[THROTTLE],
//   idle, max, 0, nav_max_terrain_follow_alt). rcCommand עובר את עקומת המצערת
//   (thr_mid, thr_expo) אחרי min_check. הפונקציות כאן הן אותו חישוב בדיוק, במספרים
//   שלמים כמו ב-INAV (rc_curves.c / rc_controls.c). אומת מול הבלקבוקס של 21.10:
//   1200µs ⇒ 1213, 1580µs ⇒ 1589 (כמו בחישוב), ובמבחן SITL של INAV 9.1.0.
//   ⚠️ תלוי בהגדרות: thr_mid 65 · thr_expo 20 · throttle_idle 8 · min_check 1100 ·
//      nav_max_terrain_follow_alt 100. שינוי באחת מהן ⇒ לעדכן כאן.
//   יעד ≤ 10cm ⇒ INAV יורד ב-50cm/s (נחיתה), ומתחת ל-min_check גם מזהה נחיתה ומנתק.
// ---------------------------------------------------------------------------
object SurfaceMap {
    private const val THR_MID = 65
    private const val THR_EXPO = 20
    private const val IDLE = 1000 + (1000 * 8) / 100      // throttle_idle 8% של 1000..2000 ⇒ 1080
    private const val MAX = 2000
    private const val MIN_CHECK = 1100
    const val TERRAIN_MAX_CM = 100

    private val lookup: IntArray = IntArray(11) { i ->
        val tmp = 10 * i - THR_MID
        val y = if (tmp > 0) 100 - THR_MID else if (tmp < 0) THR_MID else 1
        val v = 10 * THR_MID + tmp * (100 - THR_EXPO + THR_EXPO * (tmp * tmp) / (y * y)) / 10
        IDLE + (MAX - IDLE) * v / 1000
    }

    // rcCommand[THROTTLE] של INAV עבור ערוץ RC ב-µs
    fun rcCommand(us: Int): Int {
        val t = (us.coerceIn(MIN_CHECK, MAX) - MIN_CHECK) * 1000 / (MAX - MIN_CHECK)
        if (t > 999) return MAX
        val st = t / 100
        return lookup[st] + (t - st * 100) * (lookup[st + 1] - lookup[st]) / 100
    }

    // יעד הגובה של INAV (cm) לחצאי אחוז שהאפליקציה שולחת (µs = 1000 + 5·half)
    fun targetCm(half: Int): Int = (rcCommand(1000 + 5 * half.coerceIn(0, 200)) - IDLE) * TERRAIN_MAX_CM / (MAX - IDLE)

    // חצאי האחוז הקטנים ביותר שנותנים יעד ≥ cm (המעבר החלק בכניסה ל-SURFACE)
    fun halfForCm(cm: Int): Int {
        val c = cm.coerceIn(0, TERRAIN_MAX_CM)
        for (h in 0..200) if (targetCm(h) >= c) return h
        return 200
    }
}

// מצב SURFACE מה-ESP (SRS) - surface.h
object SurfaceState {
    const val OFF      = 0
    const val ACTIVE   = 1     // מבוקש ומותר: ה-FPGA מוציא CH8
    const val LOST     = 2     // DOWN אבד בזמן SURFACE - כבוי עד בקשה חדשה
    const val NO_RF    = 3     // INAV לא מדווח rangefinder (rangefinder_hardware ≠ MSP)
    const val MODE     = 4     // לא POSHOLD / משימה פעילה
    const val NO_DOWN  = 5     // אין מדידת DOWN תקינה ברגע הבקשה
    const val LINK_GAP = 6     // E1022B: פער בשורות האפליקציה (> 1.5s) - כבוי עד בקשה חדשה
    const val COMPASS  = 7     // E1022C: מצפן קפוא (ה-FPGA עבר ל-ALTHOLD) - כבוי עד בקשה חדשה

    fun label(v: Int): String = when (v) {
        OFF     -> "OFF"
        ACTIVE  -> "REQUESTED"
        LOST    -> "DOWN LOST - TAP ON AGAIN"
        NO_RF   -> "INAV: NO RANGEFINDER"
        MODE    -> "POSHOLD ONLY"
        NO_DOWN -> "NO DOWN READING"
        LINK_GAP -> "APP LINK GAP - TAP ON AGAIN"
        COMPASS  -> "COMPASS FROZEN - TAP ON AGAIN"
        else    -> "?"
    }
}

// nav_mc_althold_throttle ב-INAV - שדה ATM מה-ESP (הסדר זהה ל-settings.yaml)
object AltHoldThrottle {
    const val UNKNOWN   = -1
    const val STICK     = 0     // הנייטרל = המצערת ברגע הכניסה למצב
    const val MID_STICK = 1     // הנייטרל = thr_mid
    const val HOVER     = 2     // הנייטרל = nav_mc_hover_thr

    fun label(v: Int): String = when (v) {
        STICK     -> "STICK"
        MID_STICK -> "MID_STICK"
        HOVER     -> "HOVER"
        else      -> "NOT READ"     // כישלון קריאה לא מוסתר
    }
}

// בריאות המצפן - שדה MGH מה-ESP. ה-ESP קובע אותה לפי תגובת X/Y/Z לתנועה אמיתית
// (ג'יירו / Roll-Pitch), כי INAV עלול לדווח MAG=OK על מצפן קפוא.
object MagHealth {
    const val UNVERIFIED = 0
    const val OK         = 1
    const val FROZEN     = 2

    fun warning(v: Int): String? = when (v) {
        OK     -> null
        FROZEN -> "COMPASS FROZEN"
        else   -> "COMPASS NOT VERIFIED"
    }
}

// ---------------------------------------------------------------------------
// Telemetry - מגיע מה-ESP32 דרך UDP
//
// ⚠️ אין כאן ALT / VSPD: אומדן הגובה הוא באחריות INAV בלבד. ה-ESP קורא
//    MSP_ALTITUDE לשימוש פנימי (סבירות DWN, גיבוי לזיהוי המראה ואישור החיישן
//    הקדמי) ולא משדר אותו.
// ---------------------------------------------------------------------------
data class TelemetryData(
    val batteryPercent: Int = 0,
    val isArmed: Boolean = false,

    val lidarFwd: Int = 0,                 // cm - מקצה הפרופלור הקדמי
    val lidarDwn: Int = 0,                 // cm - מתחתית הרחפן
    val lidarFwdState: Int = LidarState.FAULT,
    val lidarDwnState: Int = LidarState.FAULT,
    // מצב המדידה שהוחל **בפועל** בחיישן (FLM / DLM מה-ESP, אחרי קריאה חוזרת), והטווח
    // המרבי שלו (FMX / DMX). האפליקציה לא מניחה מצב - היא מציגה רק את מה שדווח.
    val fwdLong: Boolean = false,
    val dwnLong: Boolean = false,
    val fwdMaxCm: Int = LidarRange.SHORT_CM,
    val dwnMaxCm: Int = LidarRange.SHORT_CM,
    val lidarModeKnown: Boolean = false,   // FLM התקבל לפחות פעם אחת

    // שלב הטיסה, כפי שה-ESP קבע אותו
    val phase: FlightPhase = FlightPhase.GROUND_DISARMED,
    // האם מותר להשתמש בחיישן הקדמי להחלטות מכשול. על הקרקע ובגובה נמוך הוא
    // רואה רצפה, ולכן המדידה ממשיכה אבל אינה משמשת להחלטות.
    val fwdActive: Boolean = false,

    // המצערת שה-ESP באמת העביר ל-FPGA לאחרונה (שדה CTH), -1 אם טרם התקבלה.
    // זה הערך שמשמש לחזרה לשליטה אחרי הפעלה מחדש - במקום אפס.
    // ברזולוציה של חצי אחוז (65.5), כמו מה שה-FPGA באמת מקבל.
    val lastThrottlePct: Float = -1f,

    val inavModes: Int = 0,                // ביטי InavMode
    val inavModesValid: Boolean = false,   // המיפוי ידוע והסטטוס טרי
    val altHoldThrMode: Int = AltHoldThrottle.UNKNOWN,
    val surfState: Int = SurfaceState.OFF,  // SRS מה-ESP (E1022A)
    val surfLosses: Int = 0,
    val surfHandoffs: Int = 0,             // SRS שדה 3: יציאות אוטומטיות מתוך SURFACE פעיל (E1022C)
    val surfTouch: Boolean = false,        // SRS שדה 5: מגע מאומת ב-ESP במצב ניווט (E1022G). ESP ישן ⇒ false

    val attRoll: Float = 0f,               // מעלות, בדיוק של עשירית
    val attPitch: Float = 0f,
    val heading: Int = 0,
    // מגנטומטר - הקריאות החיות של INAV (MSP_RAW_IMU). תצוגה בלבד, מגיע ב-5Hz.
    val magValid: Boolean = false,
    val magX: Int = 0,
    val magY: Int = 0,
    val magZ: Int = 0,
    // MagHealth - מגיע בכל חבילת 10Hz. חוסם בקשת POSHOLD / RTH / WP כשאינו OK.
    val magHealth: Int = MagHealth.UNVERIFIED,
    val fcLinkOK: Boolean = false,
    val failsafeWarn: Boolean = false,

    val checks: SystemChecks = SystemChecks(),

    val gpsStatus: String = "NO_FIX",
    val gpsLat: Double = 0.0,
    val gpsLon: Double = 0.0,
    // GPS מורחב - תצוגה ואבחון בלבד, מגיע ב-5Hz. -1 = לא ידוע.
    val gpsSats: Int = -1,
    val gpsHdop: Int = -1,                 // HDOP × 100, כמו ב-INAV
    val gpsEph: Int = -1,                  // cm
    val gpsEpv: Int = -1,                  // cm
    val lastUpdateMs: Long = 0L,
    // גרסת הקוד של ה-ESP (FWV) ואבחון של השנייה האחרונה (DGN):
    // משך סבב מרבי · פער מרבי בין שורות שליטה ל-FPGA · שאילתות MSP · שאילתות שנכשלו
    val espVersion: String = "",
    val fpgaVersion: String = "",          // FPV - גרסת ה-FPGA דרך ה-ESP. "-" = ה-FPGA לא דיווח
    val diagLoopMs: Int = -1,
    val diagCtrlGapMs: Int = -1,
    val diagMspReq: Int = -1,
    val diagMspFail: Int = -1,
    val fpgaState: FpgaState = FpgaState.UNKNOWN,
    val fpgaDebug: String = "??",          // ספרת דגלים (hex) + סיבת העצירה האחרונה
    val batteryVoltCv: Int = 0,            // 0.01V - כפי שה-Matek מדווח
    val batteryState: Int = 3,             // 0=OK 1=WARN 2=CRIT 3=NOT_PRESENT
    val batteryCells: Int = 0,
    // Mission FSM - רק כש-ה-ESP בנוי עם MISSION_ENABLE=1. אחרת נשאר ברירת המחדל.
    val mission: MissionStatus = MissionStatus(),
) {
    // המנועים חיים - שתי עדויות נפרדות, כל אחת מספיקה בפני עצמה
    val motorsLive: Boolean get() = isArmed || fpgaState.motorsLive

    // קליטת לוויינים טובה דיה למצבי ניווט
    val gpsFix3D: Boolean get() = gpsStatus == "3D"

    // ═══ מה חוסם חזרה לשליטה, או null אם מותר ═══
    //
    //   פתיחה מחדש של זרם השליטה מחייבת תמונת מצב **ודאית**. כל שורה כאן היא
    //   דבר שבלעדיו אין על מה לבסס את החבילה הבאה שנשדר:
    //     · המצערת שנשלחה בפועל (CTH) - אחרת נתחיל מערך שרירותי
    //     · דיווח מצבים תקף (FMV) - אחרת איננו יודעים מה INAV מבצע
    //     · אין Failsafe - שם INAV מנהל את האירוע, ואסור להתערב באמצע
    //     · ה-FPGA אינו נעול - LINK_LOSS ו-ABORT משתחררים בניתוק מפורש בלבד.
    //       **אין יציאה אוטומטית מהם**, וזו כוונת תכנון ולא מגבלה שיש לעקוף.
    //     · מצב INAV הפעיל מזוהה כאחד מחמשת המצבים - אחרת אין למה לסנכרן
    //
    //   אותה פונקציה משמשת את ה-ViewModel ואת הכפתור בממשק, כדי ששניהם
    //   יסכימו תמיד: הכפתור לא יציע פעולה שה-ViewModel ידחה.
    fun resumeBlockedBy(): String? = when {
        lastThrottlePct < 0f                         -> "NO KNOWN THROTTLE"
        !inavModesValid                              -> "NO MODE STATUS"
        (inavModes and InavMode.FAILSAFE) != 0       -> "INAV FAILSAFE"
        fpgaState == FpgaState.LINK_LOSS             -> "FPGA LINK LOST - DISARM"
        fpgaState == FpgaState.ABORT                 -> "FPGA ABORT - DISARM"
        FlightMode.fromInav(inavModes, true) == null -> "INAV MODE NOT RECOGNISED"
        else                                         -> null
    }
}

// ---------------------------------------------------------------------------
// תצוגת לידאר יציבה - שכבת תצוגה בלבד, לא נוגעת בשום החלטה
//
//   ה-ESP מסווג כל דגימה בנפרד, ובתנועה (למשל כשמזיזים את הרחפן ביד) דגימה
//   בודדת יכולה לצאת UNCERTAIN בגלל קפיצה שעוד לא אושרה. בתצוגה זה נראה כריצוד
//   של 100ms שאי אפשר לקרוא. לכן:
//     · מצב חדש מוצג רק אחרי שנמשך HOLD_MS ברצף. עד אז נשאר המצב הקודם,
//       ובמצב VALID - המרחק האחרון שנמדד.
//     · TOO CLOSE מוצג מיד (מסוכן), וכך גם FAULT שנמשך כבר 300ms ב-ESP,
//       ויציאה מ-FAULT לכל מצב אחר.
//     · צבע השליש עם היסטרזיס (LidarRange.bandHyst).
//   ⚠️ ה-FPGA מקבל כל דגימה כמו שהיא - הסינון כאן הוא רק למה שהעין רואה.
// ---------------------------------------------------------------------------
data class LidarView(
    val cls: Int = MissionLidar.FAULT,      // המצב המוצג (MissionLidar.*)
    val cm: Int = -1,                       // המרחק המוצג, -1 = אין
    val band: LidarRange.Band? = null,      // צבע השליש (רק כש-VALID)
)

class LidarSmoother(private val mountCm: Int, private val holdMs: Long = 300L) {
    var view = LidarView(); private set
    private var pendCls = -1
    private var pendSince = 0L

    fun update(cls: Int, cm: Int, maxCm: Int, nowMs: Long): LidarView {
        // מיד: TOO CLOSE (מסוכן), FAULT (כבר נמשך 300ms ב-ESP), ויציאה מ-FAULT (אין מה להחזיק)
        val immediate = cls == MissionLidar.TOO_CLOSE || cls == MissionLidar.FAULT || view.cls == MissionLidar.FAULT
        val shownCls = if (cls == view.cls || immediate) {
            pendCls = -1; cls
        } else {
            if (cls != pendCls) { pendCls = cls; pendSince = nowMs }
            if (nowMs - pendSince >= holdMs) { pendCls = -1; cls } else view.cls
        }
        view = if (shownCls == MissionLidar.VALID) {
            // מוצג VALID: מרחק חדש רק מדגימה VALID, אחרת נשאר האחרון
            val c = if (cls == MissionLidar.VALID && cm >= 0) cm else view.cm
            LidarView(shownCls, c, if (c >= 0) LidarRange.bandHyst(c, maxCm, mountCm, view.band) else null)
        } else LidarView(shownCls, view.cm, null)
        return view
    }
}

// ---------------------------------------------------------------------------
// Pre-flight state
// ---------------------------------------------------------------------------
sealed class PreFlightState {
    object Idle : PreFlightState()
    object Checking : PreFlightState()
    object MotorChecking : PreFlightState()
    object Ready : PreFlightState()
    data class Fault(val errorCode: String) : PreFlightState()
}

// ---------------------------------------------------------------------------
// Flight modes - מה שהאפליקציה **מבקשת**
//
// השרשרת: האפליקציה מבקשת ⇒ ה-ESP מעביר ⇒ ה-FPGA ממיר לערך AUX על CH7 ⇒
//         **INAV מחליט** אם להפעיל ⇒ מה שפעיל בפועל חוזר בשדה FMD.
//
// ⚠️ שתי הצלעות האלה נפרדות לחלוטין בממשק ובקוד. POSHOLD בלי GPS פשוט לא
//    יופעל, והאפליקציה תמשיך להציג ANGLE כמצב הפעיל - כי זה מה שקורה באמת.
//    לוגיקת המצערת נגזרת מ-InavMode.NAV_THROTTLE_MASK בלבד, ולא מכאן.
//
// ערכי CH7 ב-FPGA, תואמים לטווחי ה-AUX ב-INAV:
//   ANGLE 1000 · ALTHOLD 1250 · POSHOLD 1500 · RTH 1750 · WP 2000
// ---------------------------------------------------------------------------
enum class FlightMode(val cmd: String, val label: String, val hint: String) {
    ANGLE  ("MODE:ANGLE",   "ANGLE",   "MANUAL - INAV STABILISES ONLY"),
    ALTHOLD("MODE:ALTHOLD", "ALTHOLD", "INAV HOLDS ALTITUDE"),
    POSHOLD("MODE:POSHOLD", "POSHOLD", "INAV HOLDS ALTITUDE AND POSITION"),
    RTH    ("MODE:RTH",     "RTH",     "INAV RETURNS HOME AT 2.5 M AND HOVERS - LAND MANUALLY"),
    WP     ("MODE:WP",      "WP",      "MISSION SELECTED - START FLIES IT");

    val needsGps: Boolean get() = this == POSHOLD || this == RTH || this == WP

    // מה חוסם את הבחירה כרגע, או null אם מותר לבקש.
    //
    // כל מצב שאינו ANGLE דורש את שלושת אלה, בסדר הזה: מנועים חיים, הרחפן
    // באוויר, ודיווח מצבים תקף מ-INAV. מצבי GPS דורשים בנוסף קיבוע תלת-ממדי.
    // דיווח המצבים נבדק **לפני** ה-GPS ובלי קשר אליו: בלי FMV אנחנו לא יודעים
    // מה INAV מבצע, ואז אין על מה לבסס בקשה - גם לא ל-ALTHOLD.
    //
    // ⚠️ זו שכבת ממשק בלבד, ולא שכבת בטיחות: INAV הוא שכבת האימות האחרונה ויסרב
    //    להפעיל מצב שתנאיו אינם מתקיימים, גם אם הכפתור כאן היה פתוח.
    // ⚠️ POSHOLD / RTH / WP - אלה שתלויים ב-Heading - דורשים גם מצפן מאומת
    //    (MGH=1). זו אותה חסימה שה-FPGA אוכף על CH7; כאן היא רק מכבה את הכפתור.
    //
    // ═══ WP = "משימה נבחרה", לא מצב ידני ═══
    //   מנוטרל ועל הקרקע: מותר תמיד - זו הכנת המשימה (CMD:MPREP). לא מניע מנועים:
    //   ה-FPGA מוציא אותו ל-CH7 כ-ANGLE על הקרקע, ו-WP אמיתי רק מה-Mission FSM.
    //   מחומש על הקרקע: רק אם המשימה כבר הוכנה (ההכנה עצמה רק כשמנוטרל).
    //   באוויר: כמו POSHOLD (ה-FPGA מוציא POSHOLD עד START).
    fun blockedBy(t: TelemetryData): String? = when {
        this == ANGLE                   -> null
        this == WP && !t.motorsLive && !t.phase.offGround -> null
        this == WP && t.phase != FlightPhase.AIRBORNE ->
            if (t.mission.prepared) null else "PREPARE WHEN DISARMED"
        !t.motorsLive                   -> "ARMED"
        t.phase != FlightPhase.AIRBORNE -> "AIRBORNE"
        !t.inavModesValid               -> "MODE STATUS"
        needsGps && !t.gpsFix3D         -> "GPS FIX"
        needsGps && t.magHealth != MagHealth.OK -> "COMPASS"
        else                            -> null
    }

    companion object {
        // המצב ש-INAV מבצע כרגע, מתורגם לאחד מחמשת המצבים שלנו.
        //
        // משמש בחזרה לשליטה: הבקשה מסונכרנת למה שקורה בפועל, כי חזרה עם
        // ANGLE הייתה מורידה את CH7 ל-1000 ומבטלת מצב ניווט פעיל - למשל RTH
        // באמצע חזרה הביתה.
        //
        // null = **אין מצב מזוהה**, ואז אסור לחזור לשליטה בכלל. זה כולל
        // Failsafe (INAV מנהל את האירוע), דיווח לא תקף, וכל מצב שאינו אחד
        // מהחמישה - ACRO, HORIZON או MANUAL. היעדר דגלי ניווט אינו ANGLE:
        // ANGLE מזוהה רק כשהדגל שלו באמת דלוק.
        fun fromInav(flags: Int, valid: Boolean): FlightMode? = when {
            !valid                                -> null
            (flags and InavMode.FAILSAFE)    != 0 -> null
            (flags and InavMode.NAV_RTH)     != 0 -> RTH
            (flags and InavMode.NAV_WP)      != 0 -> WP
            (flags and InavMode.NAV_POSHOLD) != 0 -> POSHOLD
            (flags and InavMode.NAV_ALTHOLD) != 0 -> ALTHOLD
            (flags and InavMode.ANGLE)       != 0 -> ANGLE
            else                                  -> null
        }
    }
}

// ---------------------------------------------------------------------------
// Link state - מצב הקישור בלבד. מה מותר לשדר נקבע ב-ControlAuthority.
// ---------------------------------------------------------------------------
enum class LinkState { CONNECTING, CONNECTED, LOST }

// ---------------------------------------------------------------------------
// Control values - נשלח ל-ESP32 ב-20Hz
//
// MODE הוא בקשת מצב הטיסה. ה-ESP מעביר אותו כמו שהוא, וה-FPGA ממיר אותו
// לערך AUX. הוא אינו משנה שום דבר בהתנהגות האפליקציה עצמה.
//
// DSC הוא ביט כוונת ירידה: מקל המצערת נדחף מטה. ה-ESP משתמש בו כתנאי
// לזיהוי נחיתה, במקום להסיק כוונה מאחוז המצערת.
//
// PHA הוא אישור השלב: השלב האחרון שהאפליקציה **קיבלה** בטלמטריה. ה-ESP
// משתמש בו כדי לדעת מתי מותר לשחרר את רצפת המצערת - הוא עובר ל-AIRBORNE
// מיידית, אבל האפליקציה יודעת על כך רק אחרי עד 100ms ואף יותר.
//
// ⚠️ ה-FPGA מתעלם מ-DSC ומ-PHA: מפענח הפקודות שלו מזהה מפתחות ידועים בלבד,
//    ומפתח לא מוכר אינו מבטל את תקפות שורת השליטה.
//
// SRF (E1022A) הוא בקשת SURFACE. ה-ESP מסיר אותו מהשורה ומחליט בעצמו (surface.h);
// SNT (E1022B) הוא מרכז המקל ב-SURFACE: ה-ESP מעביר אותו ל-FPGA (THS), וה-FPGA מחזיק אותו ביציאה;
// ה-FPGA מוציא CH8 רק ב-POSHOLD. מה שקורה בפועל חוזר ב-FMD (ביט SURFACE) וב-SRS.
//
// RPL הוא מתג הגבלת Roll/Pitch ל-75% (RC 1125..1875). ה-FPGA מחיל אותו רק
// ב-ANGLE וב-ALTHOLD ידני, לפי CH7 שיוצא בפועל ו-INAV, ובמעבר הדרגתי.
//
// THH היא המצערת ביחידות של **חצי אחוז** (0..200, 131 = 65.5%). THR נשאר
// באחוזים שלמים (THH / 2) בשביל רכיבים בגרסה ישנה. ה-ESP קורא את THH, אוכף
// עליו את רצפת ההמראה, ושולח ל-FPGA את הערך הסופי בשדה TH2.
// ⚠️ לא שולחים עשרוני בתוך THR: מפענח FPGA ישן היה קורא "65.5" כ-655
//    וחותך ל-100% - מצערת מלאה.
// ---------------------------------------------------------------------------
data class ControlValues(
    val throttle: Float = 0f,            // 0 - 100
    val yaw: Float = 0f,                 // -100 - 100
    val pitch: Float = 0f,
    val roll: Float = 0f,
    val mode: FlightMode = FlightMode.ANGLE,
    val descentIntent: Boolean = false,
    val phaseAck: Int = 0,               // FlightPhase.code של השלב שהתקבל
    val rpLimit: Boolean = false,        // RPL: הגבלת Roll/Pitch ל-75% ב-ANGLE / ALTHOLD (ה-FPGA). כבויה עד שהטייס מפעיל
    val surface: Boolean = false,        // SRF: בקשת SURFACE (E1022A). ה-ESP וה-FPGA מחליטים אם מותר
    val surfCenterHalf: Int = -1         // SNT: מרכז המקל ב-SURFACE, חצאי אחוז (E1022B). ‎-1 = לא ב-SURFACE
) {
    // המצערת כפי שהיא נשלחת בפועל: מכומתת לחצאי אחוז (0..200).
    // כל תצוגה של המצערת נגזרת מכאן, כדי שהמסך יראה בדיוק את מה שנשלח.
    val throttleHalf: Int get() = (throttle * 2f).roundToInt().coerceIn(0, 200)
    val throttleSent: Float get() = throttleHalf / 2f

    fun toUdpString(): String =
        "THR:${throttleHalf / 2} THH:$throttleHalf YAW:${yaw.toInt()} PCH:${pitch.toInt()} RLL:${roll.toInt()} " +
        "${mode.cmd} DSC:${if (descentIntent) 1 else 0} PHA:$phaseAck RPL:${if (rpLimit) 1 else 0} SRF:${if (surface) 1 else 0} SNT:$surfCenterHalf"
}

// ---------------------------------------------------------------------------
// FPGA flight-state machine - mirrors flight_fsm.v
//
// ה-FPGA הוא שכבת בטיחות ובוררות בלבד. אין בו נחיתה ואין הורדת מצערת:
// באובדן קשר הוא עוצר את זרם ה-RC, ו-INAV מפעיל את ה-Failsafe שלו.
// ---------------------------------------------------------------------------
enum class FpgaState(val code: Int, val label: String, val hint: String) {
    DISARMED (0, "DISARMED",  "PRESS SYSTEM START"),
    READY    (1, "READY",     "READY TO ARM"),
    ARMED    (2, "ARMED",     "MOTORS LIVE"),
    LINK_LOSS(3, "LINK LOST", "RC STREAM STOPPED - INAV FAILSAFE"),
    RESERVED (4, "---",       "UNUSED STATE"),
    ABORT    (5, "ABORT",     "PRESS DISARM TO CLEAR"),
    UNKNOWN  (-1, "---",      "NO FPGA LINK");

    val motorsLive: Boolean get() = this == ARMED || this == LINK_LOSS

    // מצב שבו ה-FPGA לא יקבל שליטה ממילא, עד שתגיע פקודת ניתוק מפורשת
    val locked: Boolean get() = this == LINK_LOSS || this == ABORT

    companion object {
        fun from(code: Int): FpgaState = entries.firstOrNull { it.code == code } ?: UNKNOWN
    }
}

// ---------------------------------------------------------------------------
// Mission FSM - המשימה האוטונומית
//   FPGA: mission_fsm.v (כל ההחלטות) · ESP: mission.h (קריאה מ-INAV, הכנה, הגדרות)
//
// כלול תמיד ורדום עד START מפורש. האפליקציה אינה מחליטה דבר:
//   WP (מנוטרל)  ⇒ CMD:MPREP - ה-ESP קורא את המשימה שב-INAV ושומר תמונה שלה
//   ביטול WP     ⇒ CMD:MCANCEL
//   START        ⇒ CMD:MSTART - ה-FPGA בודק 16 תנאי SELF_CHECK ומחליט
//   STOP         ⇒ CMD:MSTOP  - פתוח תמיד
// המצב, הסיבה והבדיקות שנכשלו חוזרים בטלמטריה. החסימה כאן רק מכבה כפתור
// שהלחיצה עליו בוודאות תידחה.
// ---------------------------------------------------------------------------
object MissionConfig {
    const val ENABLED = true
}

// מצבי ה-FSM - הקודים של mission_fsm.v
enum class MissionState(val code: Int, val label: String) {
    IDLE       (0,  "WAITING"),
    HOLD_START (1,  "HOLD BEFORE START"),
    WP_ENTER   (2,  "WP REQUESTED"),
    WP_ACTIVE  (3,  "FLYING WP"),
    OBST_STOP  (4,  "OBSTACLE - STOP"),
    CLIMB      (5,  "CLIMB TO CLEAR"),
    CLEARANCE  (6,  "VERTICAL CLEARANCE"),
    CLEAR_HOLD (7,  "HOLD - VERTICAL STOP"),
    GROUND_REF (8,  "GROUND REFERENCE"),
    CROSS      (9,  "CROSSING"),
    REAR_EDGE  (10, "WAIT REAR EDGE"),
    REAR_CLR   (11, "FORWARD CLEARANCE"),
    REAR_STOP  (12, "STOP AFTER OBSTACLE"),
    RET_ALT    (13, "RETURN TO MISSION ALT"),
    ALT_CONFIRM(14, "ALTITUDE CONFIRM"),
    RESUME_WP  (15, "RESUME WP"),
    WP_DONE    (16, "AT B - POSHOLD"),
    COMPLETE   (17, "MISSION COMPLETE"),
    SUSPENDED  (18, "SUSPENDED"),
    ABORT_HOLD (19, "ABORT - HOLD"),
    ABORT_RTH  (20, "ABORT - RTH"),
    TO_POSHOLD (21, "TAKEOFF - POSHOLD"),
    TO_CLIMB   (22, "TAKEOFF - CLIMB"),
    TO_SETTLE  (23, "TAKEOFF - SETTLE"),
    DONE_HOLD  (24, "COMPLETE - WAITING FOR PILOT"),
    HOLD_SYNC  (25, "HOLD - MATCH THROTTLE"),
    UNKNOWN    (-1, "---");

    // ה-FSM שולט ב-RC. IDLE / COMPLETE / SUSPENDED = מעבר ישיר: האפליקציה שולטת.
    val active: Boolean get() = code in 1..25 && code != 17 && code != 18

    // עצירה לפני מכשול ועד חזרה למסלול
    val avoiding: Boolean get() = code in 4..15

    companion object {
        fun from(code: Int): MissionState = entries.firstOrNull { it.code == code } ?: UNKNOWN
    }
}

// סיבת העצירה / הדחייה האחרונה - MRS (hex), הקודים של mission_fsm.v
object MissionReason {
    const val START_REJECTED = 0x01
    private val NAMES = mapOf(
        0x00 to "NONE",            0x01 to "START REJECTED",  0x02 to "PILOT STOP",      0x03 to "PILOT TAKEOVER",
        0x04 to "FPGA SAFETY",     0x05 to "INAV MODE ACK",   0x06 to "STALE POSE",      0x07 to "STALE NAV",
        0x08 to "STALE FLAGS",     0x09 to "STALE GPS",       0x0A to "STALE SENSORS",   0x0B to "STALE FRONT",
        0x0C to "STALE DOWN",      0x0D to "COMPASS",         0x0E to "GPS",             0x0F to "INAV FAILSAFE",
        0x10 to "INAV DISARMED",   0x11 to "NAV ERROR",       0x12 to "FRONT INACTIVE",  0x13 to "STOP TIMEOUT",
        0x14 to "TEST STAGE END",  0x15 to "STOP TOO CLOSE",  0x16 to "CEILING",         0x17 to "CLIMB TIMEOUT",
        0x1A to "MIN GAP",         0x1C to "FRONT OBSTACLE",  0x1D to "OBST LONGER THAN SET", 0x1F to "ALT TIMEOUT",
        0x20 to "WP MISMATCH",     0x21 to "WP STALL",        0x22 to "BATTERY",         0x23 to "MISSION TIME",
        0x25 to "CROSS TIMEOUT",   0x26 to "OBSTACLE LOST",   0x28 to "INAV EMERGENCY",  0x29 to "STALE SLOW LINE",
        0x2A to "NO LIFTOFF",      0x2B to "SETTLE TIMEOUT",  0x2C to "SETTINGS",        0x2D to "THROTTLE NOT MATCHED",
        0x2E to "CROSS DRIFT",     0x2F to "NO DIRECTION",    0x30 to "GPS TOO WEAK TO CROSS",
    )

    fun label(code: Int): String = NAMES[code] ?: "0x%02X".format(code)
}

// 16 ביטי SELF_CHECK - MCK (hex). כולם נדרשים ל-START.
//   חלק מהביטים בודקים דבר אחר על הקרקע (המראה אוטומטית) ובאוויר (המראה ידנית).
object MissionCheck {
    val NAMES = listOf(
        "FPGA ARMED", "INAV ARMED", "TAKEOFF ENABLED / AIRBORNE", "WP SELECTED / POSHOLD",
        "COMPASS OK", "GPS 12 SATS / EPH 1.5M", "NO FAILSAFE", "POSE FRESH", "NAV FRESH", "LIDARS OK",
        "MISSION READY", "INAV CONFIG + SETTINGS", "WP 1 / RESUME", "BATTERY OK",
        "STICKS CENTERED / THR LOW", "STOPPED"
    )
    const val ALL  = 0xFFFF
    // 25.10: הרחפן עומד (‎≤15cm/s) - האפליקציה מחכה לו ב-START ולא חוסמת עליו את הכפתור
    const val STOPPED = 1 shl 15
    const val STOPPED_NAME = "STOPPED"
    // ההכנה (מנוטרל): הכל חוץ מהחימוש - מה שה-ARM עוד יוסיף
    // 25.10: בלי STOPPED - ההכנה לא "מהבהבת" בגלל ריחוף לא שקט; START מחכה לו בנפרד
    const val PREP = 0x7FFC

    // הבדיקות שנכשלות, לפי סדר הביטים
    fun failing(bits: Int, required: Int = ALL): List<String> =
        NAMES.indices.filter { ((required shr it) and 1) == 1 && ((bits shr it) and 1) == 0 }.map { NAMES[it] }
}

// ═══ 25.10: לוגיקה טהורה שנבדקת על המחשב (tests/AppLogicTest.kt) ═══

// נעילת המצערת ל-0% אחרי נגיעה ב-SURFACE
//   נעילה רק לפי מגע **מאומת ב-ESP** (SRS שדה 5 = navTouch, E1022G): אותה הוכחה כמו GROUND_ARMED
//   ב-ANGLE - מדידת קרקע גולמית טרייה עם התקרבות (או DOWN ≤ 5cm), סבירות מול INAV (≤ 100cm),
//   כוונת ירידה ומצערת נמוכה, ברצף 600ms. ב-POSHOLD/SURFACE השלב נשאר AIRBORNE, ולכן לא PHS.
//   TOO_CLOSE שהאפליקציה רואה לבדו לא מספיק (יכול להיות נעילה ישנה).
//   שחרור: ניתוק, דחיפה ברורה למעלה, או שה-ESP מחק את המגע (עדות לאוויר: 2 דגימות DOWN תקינות
//   רצופות מעל 12cm או INAV מעל 100cm) - אז חלה שוב המצערת הרגילה, כולל המסירה הבטוחה.
//   אובדן DOWN על הקרקע **אינו** משחרר (שם הוא המצב הרגיל), וגם לא יציאה מ-SURFACE על הקרקע.
//   E1022I: inSurface = SURFACE פעיל **או** הפריים של היציאה (navTouch שה-ESP קבע ביציאה עצמה).
object SurfTouchdown {
    const val RELEASE_STICK = 0.30f
    fun latch(inSurface: Boolean, stickY: Float, espTouch: Boolean): Boolean =
        inSurface && stickY <= -1f && espTouch
    fun release(motorsLive: Boolean, stickY: Float, espTouch: Boolean): Boolean =
        !motorsLive || stickY >= RELEASE_STICK || !espTouch
    // SRS:state,losses,exits,rel%,touch - שדה 5 חסר (ESP ישן) ⇒ false
    fun parseSrs(srs: String): Boolean = srs.split(',').getOrNull(4) == "1"
}

// המתנת START ליציבות (STOPPED). פקיעת הזמן נבדקת **לפני** כל שליחה, ואחריה כל חסימה אחרת.
object StartWait {
    enum class Step { WAIT, SEND, TIMEOUT, BLOCKED }
    fun step(nowMs: Long, untilMs: Long, otherBlock: String?, fullBlock: String?): Step = when {
        nowMs >= untilMs  -> Step.TIMEOUT
        otherBlock != null -> Step.BLOCKED
        fullBlock == null  -> Step.SEND
        else               -> Step.WAIT
    }
    // דחיית FPGA בזמן ההמתנה: ממשיכים לחכות רק אם החלון פתוח והסיבה היחידה היא STOPPED
    fun retryAfterReject(nowMs: Long, untilMs: Long, failing: List<String>): Boolean =
        untilMs != 0L && nowMs < untilMs && failing == listOf(MissionCheck.STOPPED_NAME)
}

// בדיקת משימת ה-WP ב-ESP - MIS
object MissionVerify {
    const val OK = 1

    fun label(code: Int): String = when (code) {
        0    -> "NOT READ"
        1    -> "READY"
        2    -> "NOT PREPARED"
        3    -> "INVALID IN INAV"
        4    -> "CHANGED SINCE PREPARE"
        5    -> "HAS LAND / RTH"
        7    -> "WP SPEED > 100 CM/S"
        8    -> "READ ERROR"
        9    -> "UNSUPPORTED ACTION"
        10   -> "WP1 ALT ABSOLUTE"
        11   -> "WP1 ALT OUT OF RANGE"
        else -> "?"
    }
}

// סיווג דגימת הלידאר ל-Mission FSM - FRC / DRC
//   ⚠️ OUT OF RANGE = חיישן תקין בלי מטרה בטווח: לא תקלה, ואינו הוכחה לנתיב
//      פנוי. רק FAULT הוא תקלה. אף אחד מהם אינו ירוק.
object MissionLidar {
    const val FAULT        = 0
    const val VALID        = 1
    const val OUT_OF_RANGE = 2
    const val WEAK         = 3
    const val UNCERTAIN    = 4
    const val TOO_CLOSE    = 5
    const val NO_SIGNAL    = 6

    fun label(c: Int): String = when (c) {
        FAULT        -> "FAULT"
        VALID        -> "VALID"
        OUT_OF_RANGE -> "OUT OF RANGE"
        WEAK         -> "WEAK TARGET"
        UNCERTAIN    -> "UNCERTAIN"
        TOO_CLOSE    -> "TOO CLOSE"
        NO_SIGNAL    -> "NO SIGNAL"
        else         -> "?"
    }
}

// ---------------------------------------------------------------------------
// הגדרות העקיפה - Mission Settings
//
//   מקור האמת: NVS של ה-ESP. האפליקציה רק מציגה את הסט שה-ESP מדווח (MSE)
//   ושולחת שינוי (MSET). הצלחה מוצגת רק אחרי שה-ESP ענה MSET:OK - כלומר
//   אחרי שה-FPGA החזיר את אותו checksum (MSK=1).
//   שינוי מותר רק כשמנוטרל, על הקרקע, והמשימה לא פעילה.
//   ⚠️ אין כאן הגדרות של INAV (מהירות WP, PID, nav) - הן נקבעות ב-INAV בלבד.
// ---------------------------------------------------------------------------
data class MissionSettings(
    val stopCm:  Int = 100,     // Obstacle Detection Distance
    val clearCm: Int = 150,     // Front Clear Distance
    val vclrCm:  Int = 70,      // Extra Vertical Clearance
    val fclrCm:  Int = 70,      // Extra Forward Clearance
    val ceilCm:  Int = 400,     // Maximum Mission Altitude (יחסית לבית של INAV)
    val depthCm: Int = 50,      // Obstacle Depth
    val stage:   Int = 2,       // שלב בדיקה: 2 עצירה · 3 +טיפוס · 4 +מעבר · 5 מלא
    val takeoff: Boolean = false, // המראה אוטומטית מהקרקע (כבויה עד אישור מפורש)
) {
    fun toCmd(): String = "MSET:$stopCm,$clearCm,$vclrCm,$fclrCm,$ceilCm,$depthCm,$stage,${if (takeoff) 1 else 0}"

    // אותם טווחים כמו ב-ESP (missionSetCheck) וב-FPGA. fwdMaxCm = הטווח של FRONT במצב הנוכחי.
    fun problem(fwdMaxCm: Int): String? = when {
        stopCm !in Field.STOP.range                 -> Field.STOP.label
        stopCm + FRONT_MARGIN_CM > fwdMaxCm         -> "${Field.STOP.label} > FRONT RANGE"
        clearCm < stopCm + 10 || clearCm > Field.CLEAR.range.last -> Field.CLEAR.label
        clearCm + FRONT_MARGIN_CM > fwdMaxCm        -> "${Field.CLEAR.label} > FRONT RANGE"
        vclrCm !in Field.VCLR.range                 -> Field.VCLR.label
        fclrCm !in Field.FCLR.range                 -> Field.FCLR.label
        ceilCm !in Field.CEIL.range                 -> Field.CEIL.label
        depthCm !in Field.DEPTH.range               -> Field.DEPTH.label
        stage !in 2..5                              -> "TEST STAGE"
        else                                        -> null
    }

    enum class Field(val label: String, val range: IntRange) {
        STOP ("OBSTACLE DETECTION DISTANCE", 40..300),
        CLEAR("FRONT CLEAR DISTANCE",        50..380),
        VCLR ("EXTRA VERTICAL CLEARANCE",    30..150),
        FCLR ("EXTRA FORWARD CLEARANCE",     30..200),
        CEIL ("MAXIMUM MISSION ALTITUDE",    150..400),
        DEPTH("OBSTACLE DEPTH",              10..200),
    }

    companion object {
        const val FRONT_MARGIN_CM = 10

        // "d,c,v,f,a,o,s,l" - כמו MSE של ה-ESP (בלי ה-checksum)
        fun parse(csv: String?): MissionSettings? {
            val v = csv?.split(',')?.map { it.toIntOrNull() ?: return null } ?: return null
            if (v.size != 8) return null
            return MissionSettings(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7] == 1)
        }
    }
}

// נתוני הבלימה של העצירה האחרונה (MBV / MBA / MBS / MBT / MBR)
data class BrakeMetrics(
    val speedCms: Int = 0,      // מהירות ברגע הזיהוי
    val ackMs: Int = -1,        // עד ש-INAV אישר POSHOLD (-1 = לא אושר)
    val stopMs: Int = 0,        // עד עצירה יציבה
    val travelCm: Int = 0,      // תזוזה מנקודת הזיהוי עד העצירה
    val reboundCm: Int = 0,     // חזרה לאחור אחרי העצירה
)

data class MissionStatus(
    val present: Boolean = false,          // ה-ESP שולח שדות משימה
    val fresh: Boolean = false,            // MSV - תשובת MST טרייה מה-FPGA
    val state: MissionState = MissionState.UNKNOWN,
    val reason: Int = 0,                   // MRS
    val checks: Int = 0,                   // MCK
    val detCm: Int = -1,                   // MDD - FWD בזיהוי, -1 = אין
    val stopCm: Int = -1,                  // MDS - FWD אחרי העצירה, -1 = אין
    val savedWp: Int = 0,                  // MWW - ה-WP שלפני העקיפה
    val stage: Int = 0,                    // MSG - שלב הבדיקה שהוחל ב-FPGA
    val frontCls: Int = MissionLidar.FAULT,
    val downCls: Int = MissionLidar.FAULT,
    val verify: Int = 0,                   // MIS
    val wpCount: Int = 0,                  // MWN - נקרא בפועל מ-INAV
    val wpCrc: String = "--------",        // MWC
    val inavCfg: String = "",              // MCF - althold_thr / user_ctrl / wp_restart / auto / max / thr_mid
    val effSpeedCms: Int = -1,             // MCS - המהירות האפקטיבית המרבית במשימה
    val wp1AltCm: Int = 0,                 // MAL - גובה WP1 (0 = לא ידוע / לא תקין)
    val prepared: Boolean = false,         // MPR - המשימה הוכנה (CMD:MPREP)
    val settings: MissionSettings? = null, // MSE - הסט שב-ESP (NVS)
    val settingsConfirmed: Boolean = false,// MSK - ה-FPGA אישר את אותו סט
    val neutralHalf: Int = -1,             // MNT - נייטרל המצערת, חצאי אחוז
    val flags: Int = 0,                    // MFL - bit0 סט תקף · bit1 DOWN ראה קרקע · bit2 המראה · bit3 מסונכרן
    val brake: BrakeMetrics = BrakeMetrics(),
) {
    // ה-FSM שולט ב-RC כרגע
    val active: Boolean get() = fresh && state.active

    val throttleSynced: Boolean get() = (flags and 8) != 0
    val groundStart: Boolean get() = (flags and 4) != 0
    val takeoffEnabled: Boolean get() = settings?.takeoff == true

    // ═══ מצב המשימה לתצוגה - מצבים נפרדים, לא "פעיל/לא פעיל" ═══
    enum class Phase(val label: String) {
        NONE("NOT SELECTED"), SELECTED("SELECTED - NOT READY"), READY("MISSION READY"),
        STARTED("STARTED"), OBSTACLE("PAUSED FOR OBSTACLE"), COMPLETED("MISSION COMPLETE"),
        ABORTED("ABORTED")
    }

    fun phase(selected: Boolean): Phase = when {
        fresh && (state == MissionState.DONE_HOLD || state == MissionState.COMPLETE ||
                  state == MissionState.WP_DONE)                          -> Phase.COMPLETED
        fresh && state.avoiding                                          -> Phase.OBSTACLE
        active                                                           -> Phase.STARTED
        fresh && state == MissionState.SUSPENDED                         -> Phase.ABORTED
        !selected                                                        -> Phase.NONE
        prepared && verify == MissionVerify.OK &&
            MissionCheck.failing(checks, MissionCheck.PREP).isEmpty()    -> Phase.READY
        else                                                             -> Phase.SELECTED
    }

    // למה המשימה עדיין לא מוכנה (בזמן הכנה, לפני ARM), או null
    fun prepBlockedBy(): String? = when {
        !present                       -> "NO ESP"
        !fresh                         -> "NO FPGA REPLY"
        !prepared                      -> "NOT PREPARED"
        verify != MissionVerify.OK     -> MissionVerify.label(verify)
        settings == null               -> "NO SETTINGS"
        !settingsConfirmed             -> "SETTINGS NOT CONFIRMED"
        else -> MissionCheck.failing(checks, MissionCheck.PREP).firstOrNull()
    }

    // מה חוסם START, או null. ⚠️ שכבת ממשק בלבד - ה-FPGA בודק הכל שוב ומחליט.
    fun startBlockedBy(requested: FlightMode, streaming: Boolean, required: Int = MissionCheck.ALL): String? = when {
        !MissionConfig.ENABLED            -> "DISABLED"
        !present                          -> "NO ESP"
        !fresh                            -> "NO FPGA REPLY"
        state.active                      -> "RUNNING"
        !streaming                        -> "NO CONTROL"
        requested != FlightMode.WP        -> "SELECT WP"
        !prepared                         -> "NOT PREPARED"
        !settingsConfirmed                -> "SETTINGS NOT CONFIRMED"
        else -> MissionCheck.failing(checks, required).let { f ->
            if (f.isEmpty()) null else f[0] + (if (f.size > 1) " +${f.size - 1}" else "")
        }
    }

    companion object {
        // חבילה בלי MSV משאירה את הערך הקודם
        fun parse(f: Map<String, String>, prev: MissionStatus): MissionStatus {
            val msv = f["MSV"]?.toIntOrNull() ?: return prev
            return MissionStatus(
                present  = true,
                fresh    = msv == 1,
                state    = if (msv == 1) MissionState.from(f["MST"]?.toIntOrNull() ?: -1) else MissionState.UNKNOWN,
                reason   = f["MRS"]?.toIntOrNull(16) ?: 0,
                checks   = f["MCK"]?.toIntOrNull(16) ?: 0,
                detCm    = f["MDD"]?.toIntOrNull() ?: -1,
                stopCm   = f["MDS"]?.toIntOrNull() ?: -1,
                savedWp  = f["MWW"]?.toIntOrNull() ?: 0,
                stage    = f["MSG"]?.toIntOrNull() ?: 0,
                frontCls = f["FRC"]?.toIntOrNull() ?: MissionLidar.FAULT,
                downCls  = f["DRC"]?.toIntOrNull() ?: MissionLidar.FAULT,
                verify   = f["MIS"]?.toIntOrNull() ?: 0,
                wpCount  = f["MWN"]?.toIntOrNull() ?: 0,
                wpCrc    = f["MWC"] ?: "--------",
                inavCfg  = f["MCF"] ?: "",
                effSpeedCms = f["MCS"]?.toIntOrNull() ?: -1,
                wp1AltCm = f["MAL"]?.toIntOrNull() ?: 0,
                prepared = f["MPR"]?.toIntOrNull() == 1,
                settings = MissionSettings.parse(f["MSE"]),
                settingsConfirmed = f["MSK"]?.toIntOrNull() == 1,
                neutralHalf = if (msv == 1) f["MNT"]?.toIntOrNull() ?: -1 else -1,
                flags    = f["MFL"]?.toIntOrNull(16) ?: 0,
                brake    = BrakeMetrics(
                    speedCms  = f["MBV"]?.toIntOrNull() ?: 0,
                    ackMs     = (f["MBA"]?.toIntOrNull() ?: -1).let { if (it < 0) -1 else it * 10 },
                    stopMs    = (f["MBS"]?.toIntOrNull() ?: 0) * 10,
                    travelCm  = f["MBT"]?.toIntOrNull() ?: 0,
                    reboundCm = f["MBR"]?.toIntOrNull() ?: 0,
                ),
            )
        }
    }
}
