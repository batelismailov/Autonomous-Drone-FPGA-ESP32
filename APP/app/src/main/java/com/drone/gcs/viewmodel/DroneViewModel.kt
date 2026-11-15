package com.drone.gcs.viewmodel

import android.app.Application
import android.content.Context
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.drone.gcs.model.*
import com.drone.gcs.network.UdpManager
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.roundToInt

private const val SEND_INTERVAL_MS = 50L
private const val LINK_TIMEOUT_MS  = 500L
private const val APP_PING_MS      = 1000L

// ── מצערת ידנית ──
// מרכז הריחוף הידני, באחוזי ערוץ RC כמו שהאפליקציה שולחת (58% ⇒ 1580µs).
// נמדד מלוג הטיסה: ריחוף בערך 1584-1586 בצד ה-RC, ו-1641 בצד הפנימי של INAV.
private const val HOVER_DEFAULT      = 58f      // רק ברירת מחדל - הערך הנשמר גובר עליה
private const val HOVER_MIN          = 20f
private const val HOVER_MAX          = 75f
private const val DESCEND_RANGE      = 15f      // חלון ירידה יחסי - במצבי ניווט ועל הקרקע

// ── רצפת הירידה באוויר ──
//   כשהרחפן באוויר (PHS = AIRBORNE) הירידה נמדדת אל **ערך מוחלט** ולא בחלון
//   יחסי, בכל המצבים - ANGLE, ALTHOLD, POSHOLD, RTH, WP. מקל מלא למטה מגיע
//   לערך הזה (20% ⇒ CH3 1200µs), בלי תלות במרכז הריחוף או בנייטרל של ALTHOLD.
private const val AIRBORNE_DESCENT_MIN = 20f

// ── E1022C: גבולות מרכז ה-SURFACE שה-FPGA מחזיק ביציאה (ESP: SURF_HOLD_MIN_H / MAX_H) ──
private const val SURF_HOLD_MIN_PCT = 20f
private const val SURF_HOLD_MAX_PCT = 90f
// ── 25.10: START מחכה עד 5 שנ' לרגע שבו הרחפן עומד (בדיקת STOPPED של ה-FPGA) ──
private const val START_WAIT_MS   = 5000L

// ── שמירת מרכז הריחוף בין הפעלות ──
private const val PREFS_NAME         = "drone_gcs"
private const val KEY_HOVER_PCT      = "hover_pct"
private const val KEY_RP_LIMIT       = "rp_limit"       // הגבלת Roll/Pitch ל-75% - ברירת מחדל כבויה
private const val KEY_RP_RESET       = "rp_limit_reset_v1"   // איפוס חד-פעמי של ON שנשמר בגרסה קודמת
private const val MODE_ISSUE_MS      = 1000L            // מצב מבוקש שלא פעיל זמן כזה ⇒ התראה
private const val LATCH_THRESHOLD    = 0.15f
private const val THR_SLEW_UP_PER_S  = 60f

// ── חזרה מהירה למרכז הריחוף ──
//   מגביל העלייה הרגיל (60%/s) נועד למנוע קפיצת דחף בעלייה יזומה. אבל
//   כשהרחפן באוויר, בטיסה ידנית, המקל שוחרר והמצערת **מתחת** למרכז הריחוף -
//   כל מילישנייה שהיא נשארת שם היא ירידה. שם המגביל הופך לנזק: הרחפן מאבד
//   גובה עד שהמצערת מגיעה לריחוף, וכשהיא מגיעה כבר יש מהירות אנכית כלפי מטה.
//
//   בחלון הזה בלבד מוחלף הקצב באחד שסוגר את המרחק מאפס ועד מרכז הריחוף בזמן
//   שלמטה. הקצב נגזר מ-hoverPercent ולא ממספר קבוע, כדי שעדכון מרכז הריחוף
//   יחזיק גם כאן.
//
//   ⚠️ זרם השליטה הוא 20Hz, כלומר חבילה כל 50ms. 0.08s הם פחות משתי חבילות,
//      ולכן בפועל החזרה תתפרס על 1-2 חבילות - זו הרזולוציה, לא בחירה.
private const val FAST_HOVER_RETURN_S = 0.08f

// ── מרכז המצערת לפי מצב הטיסה בפועל ב-INAV ──
private const val RECENTER_EPS       = 0.05f   // |מקל| קטן מזה = הטייס הרפה
private const val DESCENT_EPS        = 0.15f   // מקל דחוף מטה = כוונת ירידה מפורשת
private const val CENTER_SLEW_PER_S  = 20f     // יציאה משמירת גובה: חזרה רכה למרכז הידני
private const val NAV_THR_LOW_PCT    = 10f     // INAV: מצערת נמוכה (min_check 1100) ⇒ נייטרל = thr_mid
private const val INAV_THR_MID_PCT   = 65f     // thr_mid ב-INAV - הנייטרל של MID_STICK

// ריחוף לשימוש כש-nav_mc_althold_throttle = HOVER.
// null עד שנסנכרן אותו מול nav_mc_hover_thr - עד אז HOVER מתנהג כמו STICK.
private val HOVER_NEUTRAL_PCT: Float? = null

// ── שעון בקר הטיסה ──
private const val RTC_RESEND_MS      = 5 * 60 * 1000L   // רענון תקופתי של השעה

// ── תשובות לפקודות משימה / לידאר ──
private const val REPLY_TIMEOUT_MS   = 4000L            // בלי תשובה ⇒ הודעת שגיאה
// אחרי LMODE:OK הכפתור נשאר נעול עד שהטלמטריה מראה את המצב החדש, ולפחות זמן זה -
// כדי שלחיצה מהירה לא תשלח שוב לפי FLM/DLM ישן ותחליף את החיישן שוב ושוב.
// ≥ LIDAR_MODE_MIN_GAP_MS ב-ESP (1000), כך שהאפליקציה לא מקבלת BUSY.
private const val LIDAR_SETTLE_MS    = 1200L

// ═══════════════════════════════════════════════════════
//   צליל + רטט לאישור - רק אחרי הצלחה **מאושרת**, וצליל + רטט אחרים לשגיאה
//   שמירת הגדרות · SHORT/LONG · הכנת משימה · מצב טיסה ש-INAV אישר · START שהתחיל בפועל.
//   ⚠️ לא נוגע בצלילי ARM / DISARM / Failsafe (של בקר הטיסה).
// ═══════════════════════════════════════════════════════
private class Beeper(context: Context) {
    // ערוץ המוזיקה: נשמע גם במצב רטט / שקט, לפי עוצמת המוזיקה של הטאבלט
    private val tone = runCatching { ToneGenerator(AudioManager.STREAM_MUSIC, 90) }.getOrNull()
    private val vib: Vibrator? = runCatching { context.getSystemService(Vibrator::class.java) }.getOrNull()

    // אישור: צליל קצר + רטט קצר אחד. שגיאה: צליל ארוך + שני רטטים.
    fun ok() {
        runCatching { tone?.startTone(ToneGenerator.TONE_PROP_ACK, 180) }
        runCatching { vib?.vibrate(VibrationEffect.createOneShot(80, VibrationEffect.DEFAULT_AMPLITUDE)) }
    }
    fun err() {
        runCatching { tone?.startTone(ToneGenerator.TONE_PROP_NACK, 450) }
        runCatching { vib?.vibrate(VibrationEffect.createWaveform(longArrayOf(0, 150, 100, 150), -1)) }
    }
    // לחיצה על כפתור: צליל קצרצר + רטט קצרצר - שונה מהאישור ומהשגיאה
    fun tap() {
        runCatching { tone?.startTone(ToneGenerator.TONE_PROP_BEEP, 60) }
        runCatching { vib?.vibrate(VibrationEffect.createOneShot(30, VibrationEffect.DEFAULT_AMPLITUDE)) }
    }
    fun release() { runCatching { tone?.release() } }
}

// ═══════════════════════════════════════════════════════
//   מקליט טיסה - כל חבילה שנכנסת (R) ויוצאת (T), ואירועים (E), עם זמן במילישניות.
//   רק מידע שכבר עובר באפליקציה: אין שום שאילתה נוספת לרחפן.
//   קובץ לכל הפעלה של האפליקציה, ב-Android/data/<app>/files/flightlogs.
//   הכתיבה בחוט נפרד; תור מלא ⇒ השורה נזרקת (לעולם לא מעכב את השליטה).
//   נשמרים 30 הקבצים האחרונים.
// ═══════════════════════════════════════════════════════
private class FlightRecorder(context: Context) {
    private val q = LinkedBlockingQueue<String>(8192)
    val file: File? = runCatching {
        val dir = context.getExternalFilesDir("flightlogs") ?: return@runCatching null
        dir.mkdirs()
        dir.listFiles()?.filter { it.name.startsWith("flight_") }?.sortedByDescending { it.name }
            ?.drop(29)?.forEach { it.delete() }
        File(dir, "flight_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + ".log")
    }.getOrNull()

    init {
        val f = file
        if (f != null) Thread {
            runCatching {
                BufferedWriter(FileWriter(f, true)).use { w ->
                    var lastFlush = System.currentTimeMillis()
                    while (true) {
                        val line = q.poll(1, TimeUnit.SECONDS)
                        if (line != null) {
                            if (line === STOP) break
                            w.write(line); w.newLine()
                        }
                        val now = System.currentTimeMillis()
                        if (line == null || now - lastFlush >= 1000) { w.flush(); lastFlush = now }
                    }
                    w.flush()
                }
            }
        }.apply { isDaemon = true; name = "flight-recorder"; start() }
    }

    fun log(kind: Char, text: String) {
        if (file != null) q.offer("${System.currentTimeMillis()} $kind $text")
    }
    fun close() { if (file != null) q.offer(STOP) }

    companion object { private const val STOP = "\u0000STOP" }
}

// הודעה קצרה לטייס: מה קרה עם הבקשה האחרונה (שמירה, מצב חיישן, הכנה, START)
data class Notice(val text: String, val ok: Boolean, val atMs: Long = System.currentTimeMillis())

// ⚠️ AndroidViewModel ולא ViewModel: מרכז הריחוף נשמר בין הפעלות של האפליקציה,
//    ולשם כך נדרש Context. viewModel() ב-Compose בונה אותו עם ה-Application.
class DroneViewModel(app: Application) : AndroidViewModel(app) {

    private val udp = UdpManager()

    private val prefs = app.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    private val _telemetry       = MutableStateFlow(TelemetryData())
    val telemetry: StateFlow<TelemetryData> = _telemetry.asStateFlow()

    // ⚠️ הגבלת R/P כבויה עד בחירה מפורשת. ערך ON שנשמר בגרסה קודמת (שבה ברירת
    //    המחדל הייתה פעילה) מאופס פעם אחת, כדי שלא ימשיך לפעול בלי שנבחר מחדש.
    private fun loadRpLimit(): Boolean {
        if (!prefs.getBoolean(KEY_RP_RESET, false)) {
            prefs.edit().putBoolean(KEY_RP_LIMIT, false).putBoolean(KEY_RP_RESET, true).apply()
            return false
        }
        return prefs.getBoolean(KEY_RP_LIMIT, false)
    }

    private val _control         = MutableStateFlow(ControlValues(rpLimit = loadRpLimit()))
    val control: StateFlow<ControlValues> = _control.asStateFlow()

    private val _preFlightState = MutableStateFlow<PreFlightState>(PreFlightState.Idle)
    val preFlightState: StateFlow<PreFlightState> = _preFlightState.asStateFlow()

    private val _linkState      = MutableStateFlow(LinkState.CONNECTING)
    val linkState: StateFlow<LinkState> = _linkState.asStateFlow()

    // ⚠️ רשות השליטה, בנפרד ממצב הקישור. ראו ControlAuthority ו-launchSendLoop.
    private val _controlAuthority = MutableStateFlow(ControlAuthority.WAITING)
    val controlAuthority: StateFlow<ControlAuthority> = _controlAuthority.asStateFlow()

    // מרכז הריחוף האחרון שהטייס קבע, או ברירת המחדל בהפעלה ראשונה בלבד.
    // הערך מוגבל לטווח המותר גם בקריאה, כדי שערך שמור פגום לא ייצור מרכז חריג.
    private val _hoverPercent   = MutableStateFlow(
        prefs.getFloat(KEY_HOVER_PCT, HOVER_DEFAULT).coerceIn(HOVER_MIN, HOVER_MAX)
    )
    val hoverPercent: StateFlow<Float> = _hoverPercent.asStateFlow()

    private val _latched        = MutableStateFlow(false)
    val latched: StateFlow<Boolean> = _latched.asStateFlow()

    private val _systemStarted  = MutableStateFlow(false)
    val systemStarted: StateFlow<Boolean> = _systemStarted.asStateFlow()

    // INAV במצב שבו המצערת היא פקודת גובה / ניווט ⇒ ה-+/- מוסתרים
    private val _navThrottle    = MutableStateFlow(false)
    val navThrottle: StateFlow<Boolean> = _navThrottle.asStateFlow()

    // הנייטרל שנשלח בשחרור המקל בזמן שמירת גובה
    private val _altHoldNeutral = MutableStateFlow(HOVER_DEFAULT)
    val altHoldNeutral: StateFlow<Float> = _altHoldNeutral.asStateFlow()

    // ── משימה, הגדרות ולידאר ──
    private val beeper = Beeper(app)

    // תצוגת הלידאר היציבה (LidarSmoother) - FRONT / DOWN
    private val fwdSmoother = LidarSmoother(LidarRange.FWD_MOUNT_CM)
    private val dwnSmoother = LidarSmoother(LidarRange.DWN_MOUNT_CM)
    // מצב מבוקש שאינו פעיל ב-INAV יותר מ-MODE_ISSUE_MS, עם הסיבה. null = תקין
    private val _modeIssue = MutableStateFlow<String?>(null)
    val modeIssue: StateFlow<String?> = _modeIssue.asStateFlow()
    private var modeMismatchSinceMs = 0L

    private val recorder = FlightRecorder(app)
    val recorderFile: String? = recorder.file?.name

    private val _lidarView = MutableStateFlow(LidarView() to LidarView())
    val lidarView: StateFlow<Pair<LidarView, LidarView>> = _lidarView.asStateFlow()

    private val _notice = MutableStateFlow<Notice?>(null)
    val notice: StateFlow<Notice?> = _notice.asStateFlow()

    // שמירת הגדרות ממתינה לתשובת ה-ESP (MSET:OK רק אחרי אישור ה-FPGA)
    private val _settingsPending = MutableStateFlow(false)
    val settingsPending: StateFlow<Boolean> = _settingsPending.asStateFlow()

    // שינוי SHORT/LONG ממתין: 'F' / 'D'
    private val _lidarPending = MutableStateFlow<Set<Char>>(emptySet())
    val lidarPending: StateFlow<Set<Char>> = _lidarPending.asStateFlow()

    private val _prepPending = MutableStateFlow(false)
    val prepPending: StateFlow<Boolean> = _prepPending.asStateFlow()

    private var settingsSentMs = 0L
    private var prepSentMs     = 0L
    private val lidarSentMs    = mutableMapOf<Char, Long>()
    private val lidarWant      = mutableMapOf<Char, Boolean>()   // המצב שביקשנו (LONG?)
    private val lidarOkMs      = mutableMapOf<Char, Long>()      // מתי ה-ESP אישר
    private var startWaitUntilMs = 0L        // 25.10: START מחכה ל-STOPPED עד הזמן הזה (0 = לא מחכה)
    private val _startWaiting = MutableStateFlow(false)
    val startWaiting: StateFlow<Boolean> = _startWaiting
    private var startPendingMs = 0L          // START נשלח - צליל רק כשהמשימה באמת התחילה / נדחתה
    private var modeConfirmPending: FlightMode? = null

    private var stickYNorm = 0f

    // מצב פנימי של לולאת השליחה - רק היא כותבת אליו
    private var centerNow    = 0f               // המרכז שהמקל ממופה סביבו כרגע
    private var navLast      = false
    private var holdOutput   = false            // מעבר למצב ניווט כשהמקל מוחזק
    private var holdValue    = 0f
    private var phaseLast    = FlightPhase.GROUND_DISARMED
    // מגע קרקע בזמן שהמקל מוחזק: מיפוי האוויר נשאר בתוקף עד השחרור (מעבר חלק)
    private var airborneMapHeld = false
    private var centerJump   = false            // שינוי שלב ממתין - המרכז יזוז בשחרור המקל
    // E1022A: SURFACE בפועל (FMD) כפי שלולאת השליחה ראתה, ומרכז שממתין לשחרור המקל
    private var surfLast     = false
    private var surfCenterPending: Float? = null
    // E1022B: מסירת המצערת ביציאה מ-SURFACE - המרכז נשלח עד שחרור המקל (ראו surfaceHandoff)
    private var surfHandoffHold  = false
    private var surfHandoffValue = 0f
    // 25.10: נגיעה בקרקע ב-SURFACE עם מקל מלא למטה ⇒ 0% עד ניתוק (בלי המראה חוזרת בשחרור המקל)
    private var surfTouchdown    = false
    private var takeoffHold  = false            // בחלון אישור ההמראה - רצפה לערך המצערת
    private var takeoffValue = 0f
    private var lastPingMs   = 0L
    private var lastRtcMs    = 0L

    init {
        udp.onTx = { recorder.log('T', it) }
        recorder.log('E', "APP START")
        udp.startReceiving(viewModelScope)
        launchSendLoop()
        launchReceiveLoop()
        launchLinkWatchdog()
    }

    // ── קלט מהמקלות ──
    fun updateThrottleStick(yNorm: Float) {
        stickYNorm = yNorm.coerceIn(-1f, 1f)
        if (!_latched.value && stickYNorm > LATCH_THRESHOLD) {
            _latched.value = true
        }
        // הערך הגבוה ביותר שהטייס נתן מאז שהוא על הקרקע. אם שלב אישור ההמראה
        // יגיע בטלמטריה באיחור, הרצפה המקומית עדיין תתחיל מהערך הנכון.
        if (phaseLast != FlightPhase.AIRBORNE) {
            takeoffValue = maxOf(takeoffValue, _control.value.throttle)
        }
    }

    fun updateYaw(v: Float)   = _control.update { it.copy(yaw   = v.coerceIn(-100f, 100f)) }
    fun updatePitch(v: Float) = _control.update { it.copy(pitch = v.coerceIn(-100f, 100f)) }
    fun updateRoll(v: Float)  = _control.update { it.copy(roll  = v.coerceIn(-100f, 100f)) }

    // ה-+/- משנים רק את מרכז הריחוף הידני. בשמירת גובה הם מוסתרים ולא משפיעים.
    //   הערך נשמר מיד, כדי שסגירת האפליקציה - מכוונת או לא - לא תחזיר את
    //   הטייס לברירת המחדל בטיסה הבאה.
    //   הצעד הוא חצי אחוז, והערך מיושר לרשת של 0.5 - בדיוק הרזולוציה שמגיעה
    //   עד פקודת ה-RC ב-FPGA (THH / TH2).
    fun trimHover(delta: Float) {
        if (_navThrottle.value) return
        _hoverPercent.update { (((it + delta) * 2f).roundToInt() / 2f).coerceIn(HOVER_MIN, HOVER_MAX) }
        prefs.edit().putFloat(KEY_HOVER_PCT, _hoverPercent.value).apply()
    }

    // ── מיפוי מקל → מצערת סביב המרכז הנוכחי ──
    private fun throttleTarget(): Float {
        if (!_latched.value) return 0f
        // ── באוויר: AIRBORNE_DESCENT_MIN הוא גם היעד של מקל מלא למטה וגם רצפה ──
        //   מהרגע ש-PHS = AIRBORNE, מקל מלא למטה נותן **בדיוק** 20% (CH3 1200µs),
        //   ושום מצב פנימי לא מוריד את הפקודה מתחת לזה: לא centerNow שנשאר 0
        //   כשהמקל הוחזק מהקרקע לתוך AIRBORNE, לא נייטרל NAV נמוך מ-20%, ולא
        //   ערך מוחזק במעבר מצב. כשהמרכז ≥ 20% הרצפה לא משנה כלום.
        //
        //   ⚠️ השלב נלקח מ-phaseLast - מה ש-updateThrottleCenter ראה באותו טיק -
        //      ולא ישירות מהטלמטריה. חבילה שמגיעה באמצע הטיק הייתה אחרת נותנת
        //      טיק אחד שבו המרכז עוד של האוויר והמיפוי כבר של הקרקע.
        //   ⚠️ airborneMapHeld: מגע קרקע כשהמקל מוחזק אינו משנה את המיפוי. המצערת
        //      שנשלחה נשארת בדיוק כפי שהיא לאותו מיקום מקל, תזוזה של הטייס עדיין
        //      פועלת מיד, ומיפוי הקרקע נכנס רק בשחרור המקל.
        val airborne = phaseLast == FlightPhase.AIRBORNE || airborneMapHeld
        // ═══ E1022A: ב-SURFACE המצערת היא **גובה** (SurfaceMap) ולא כוח ═══
        //   הרצפה של 20% (1200µs) הייתה יעד של 14cm: מקל מלא למטה לא היה נוחת לעולם
        //   (אומת ב-SITL של INAV 9.1). ב-SURFACE מקל מלא למטה = 0% ⇒ יעד 0 ⇒ INAV יורד
        //   ב-50cm/s, נוגע, ומזהה נחיתה (מצערת < min_check, AGL אמין) ומנתק.
        //   כש-INAV מפסיק לדווח SURFACE - הרצפה חוזרת מיד.
        val floor = if (surfLast) 0f else AIRBORNE_DESCENT_MIN
        // 25.10: אחרי נגיעה ב-SURFACE - 0% עד ניתוק, גם כשהמקל משוחרר (24.10 23:48: שחרור
        //   המקל 2 שנ' אחרי הנגיעה החזיר את המרכז, ו-INAV התחיל להמריא ליעד 53cm)
        if (surfTouchdown) return 0f
        // E1022B: אחרי יציאה מ-SURFACE בזמן שהמקל מוחזק - המרכז, עד שחרור המקל
        if (surfHandoffHold) return surfHandoffValue
        if (airborne && stickYNorm <= -1f) return floor
        if (holdOutput) return if (airborne) maxOf(holdValue, floor) else holdValue
        val c = centerNow
        // ── טווח הירידה ──
        //   באוויר (PHS = AIRBORNE), בכל מצב טיסה, הירידה נמדדת אל ערך **מוחלט**:
        //   מקל מלא למטה מגיע ל-AIRBORNE_DESCENT_MIN בלי תלות במרכז הריחוף או
        //   בנייטרל של ALTHOLD. maxOf מגן על המקרה שבו המרכז כבר מתחת לרצפה -
        //   שם אין טווח ירידה כלל.
        //
        //   על הקרקע ובאישור ההמראה נשאר החלון היחסי כפי שהיה.
        val downSpan = if (airborne) maxOf(0f, c - floor) else DESCEND_RANGE

        val mapped = if (stickYNorm >= 0f) c + stickYNorm * (100f - c)
                     else                  c + stickYNorm * downSpan
        // בחלון אישור ההמראה המצערת לא יורדת מתחת למה שהטייס כבר נתן
        val out = if (takeoffHold) maxOf(mapped, takeoffValue) else mapped
        return if (airborne) maxOf(out, floor) else out
    }

    // עלייה מוגבלת בקצב, ירידה מיידית. `fast` מחליף את המגביל בקצב החזרה
    // המהיר למרכז הריחוף - ראו fastHoverReturn().
    private fun slew(current: Float, target: Float, fast: Boolean = false): Float {
        val ratePerS = if (fast) (_hoverPercent.value / FAST_HOVER_RETURN_S)
                       else       THR_SLEW_UP_PER_S
        val maxRise = ratePerS * SEND_INTERVAL_MS / 1000f
        return if (target > current) minOf(target, current + maxRise) else target
    }

    // ═══════════════════════════════════════════════════════
    //   מתי מותרת החזרה המהירה
    //
    //   ארבעת התנאים ביחד, וכל אחד מהם הכרחי:
    //     · INAV **מבצע** טיסה ידנית. נגזר מ-navLast, כלומר מ-FMD, ולא מהמצב
    //       שנבחר בממשק. במצבי ניווט INAV שולט בגובה והמצערת היא פקודת קצב -
    //       האצה שלה שם היא התערבות בבקר שלו.
    //     · הרחפן באוויר לפי ה-ESP. על הקרקע שחרור המקל חייב להישאר 0%,
    //       ובשלב אישור ההמראה הרצפה הקיימת היא ששולטת - שניהם אינם AIRBORNE.
    //     · המקל שוחרר. כל עוד הטייס מחזיק אותו, הוא זה שקובע.
    //     · המצערת מתחת למרכז הריחוף. מלמעלה למטה הירידה ממילא מיידית,
    //       והאצה בכיוון ההוא הייתה הפלה של דחף.
    // ═══════════════════════════════════════════════════════
    private fun fastHoverReturn(throttleNow: Float): Boolean =
        !navLast &&
        _telemetry.value.phase == FlightPhase.AIRBORNE &&
        abs(stickYNorm) < RECENTER_EPS &&
        throttleNow < _hoverPercent.value

    private fun approach(cur: Float, target: Float, step: Float): Float =
        if (cur < target) minOf(target, cur + step) else maxOf(target, cur - step)

    // ═══════════════════════════════════════════════════════
    //   מרכז המצערת - שלושה מרכזים שונים שאסור לערבב
    //
    //   Motor Idle          (INAV)         מחומש על הקרקע. האפליקציה שולחת מינימום
    //                                      ו-INAV מסובב את המנועים בסרק בעצמו.
    //   Manual Hover Center (האפליקציה)    רק כשה-ESP קבע AIRBORNE ובטיסה ידנית.
    //   ALTHOLD Neutral     (INAV)         רק במצב שמירת גובה:
    //                                      STICK     - המצערת שנשלחה ברגע הכניסה
    //                                                  (נמוכה מדי ⇒ thr_mid, כמו INAV)
    //                                      MID_STICK - thr_mid
    //                                      HOVER     - HOVER_NEUTRAL_PCT, ועד שנסנכרן כמו STICK
    //
    //   ⚠️ המעבר נקבע לפי מה ש-INAV **מבצע** (שדה FMD), ולא לפי המצב שנבחר
    //      בממשק. בקשה שנדחתה - POSHOLD בלי GPS למשל - לא משנה כאן דבר,
    //      והמצערת ממשיכה להתנהג כמו בטיסה ידנית. המסכה NAV_THROTTLE_MASK
    //      מכסה ALTHOLD, POSHOLD, RTH, WP ו-Failsafe: בכולם INAV שולט בגובה.
    //
    //   שלב הטיסה מגיע מה-ESP (שדה PHS) ואינו מחושב כאן: רק ה-ESP מחזיק
    //   בו-זמנית את מצב החימוש, את חיישן DWN ואת אומדן הגובה של INAV.
    //
    //   ⚠️ TAKEOFF_CONFIRMING: מרגע העדות הראשונה להמראה ועד האישור, שחרור
    //      המקל **לא** מוריד למינימום אלא מחזיק את הערך שהטייס כבר נתן.
    //      בלי זה, שחרור מקל בתוך חלון האישור מפיל את הרחפן.
    //
    //   המקל מוחזק ⇒ המרכז לא זז, כדי שלא תהיה קפיצה ברגע המעבר והטייס ישמור
    //   שליטה מלאה. המרכז החדש נכנס לתוקף כשהמקל חוזר למרכזו.
    // ═══════════════════════════════════════════════════════
    private fun updateThrottleCenter() {
        val t         = _telemetry.value
        val nav       = if (t.inavModesValid) (t.inavModes and InavMode.NAV_THROTTLE_MASK) != 0 else navLast
        val stickHeld = abs(stickYNorm) >= RECENTER_EPS
        val phase     = t.phase

        // ⚠️ ההגנה האמיתית על המצערת בזמן אישור ההמראה נאכפת ב-ESP32, שם שלב
        //    הטיסה ידוע מיידית. מה שנעשה כאן הוא רק שיקוף שלה בממשק, כדי
        //    שהמחוון והערך שמוצג יתאימו למה שנשלח בפועל. אסור להסתמך על
        //    השכבה הזו לבטיחות: הטלמטריה מגיעה ב-10Hz, ובזמן הזה הרחפן כבר
        //    יכול להיות באוויר.
        if (phase != phaseLast) {
            // ═══ מעבר חלק: AIRBORNE → קרקע כשהמקל מוחזק ═══
            //   עצם החלפת השלב אסור שיעלה את המצערת. מיפוי האוויר נשמר עד השחרור.
            if (phaseLast == FlightPhase.AIRBORNE && stickHeld) airborneMapHeld = true
            if (phase == FlightPhase.TAKEOFF_CONFIRM) {
                takeoffHold  = true
                takeoffValue = maxOf(_control.value.throttle, takeoffValue)
            } else {
                takeoffHold  = false
                takeoffValue = 0f
            }
            phaseLast  = phase
            centerJump = true
        }

        if (nav != navLast) {
            val sent = _control.value.throttle
            if (nav) {
                val fixed: Float? = when (t.altHoldThrMode) {
                    AltHoldThrottle.MID_STICK -> INAV_THR_MID_PCT
                    AltHoldThrottle.HOVER     -> HOVER_NEUTRAL_PCT
                    else                      -> null
                }
                val neutral = fixed ?: if (sent < NAV_THR_LOW_PCT) INAV_THR_MID_PCT else sent
                _altHoldNeutral.value = neutral
            }
            holdValue   = sent
            holdOutput  = stickHeld
            navLast     = nav
            _navThrottle.value = nav
        }

        // ═══ E1022A: כניסה ל-SURFACE - מרכז המקל = הגובה הנוכחי (מעבר חלק) ═══
        //   ב-SURFACE ה-INAV מתרגם את המצערת ישירות לגובה. אם המרכז נשאר הנייטרל
        //   הקודם (למשל 58% = 55cm) הרחפן היה עולה / יורד אליו - ב-SITL: מ-91 ל-65cm.
        //   לכן בכניסה המרכז הופך לחצי האחוז שנותן את הגובה שה-DOWN מודד עכשיו.
        //   המקל מוחזק ⇒ המרכז החדש ממתין לשחרור (כמו בשאר המעברים).
        //   ביציאה - המרכז **נשאר**: INAV נכנס מחדש ל-POSHOLD ולוקח את המצערת הנוכחית
        //   כנייטרל (STICK), כך שאין קפיצה.
        //   E1022B: SURFACE נחשב פעיל רק כש-INAV מדווח (FMD) **וגם** ה-ESP עדיין מבקש (SRS = 1).
        //   ה-ESP מחליט על היציאה לפני ש-INAV מדווח עליה, ולכן SRS ≠ 1 מספיק כדי לצאת.
        val surfNow = nav && t.inavModesValid && (t.inavModes and InavMode.SURFACE) != 0 &&
                      t.surfState == SurfaceState.ACTIVE
        if (surfNow && !surfLast) {
            surfCenterPending = if (t.lidarDwnState == LidarState.VALID && t.lidarDwn in 1..SurfaceMap.TERRAIN_MAX_CM)
                SurfaceMap.halfForCm(t.lidarDwn) / 2f else null
            recorder.log('E', "SURFACE ON dwn=${t.lidarDwn} center=${surfCenterPending}")
        }
        // 25.10: נגיעה - רק לפי מגע מאומת ב-ESP (SRS שדה 5), ראו SurfTouchdown
        //   E1022I: גם בפריים שבו SURFACE יוצא (surfLast): ה-ESP קובע navTouch ביציאה שקטעה הוכחת
        //   מגע, והוא מגיע באותה שורת טלמטריה שבה SRS ≠ 1. הנעילה כאן קודמת ל-surfaceHandoff.
        if (!surfTouchdown && SurfTouchdown.latch(surfNow || surfLast, stickYNorm, t.surfTouch)) {
            surfTouchdown = true
            _notice.value = Notice("TOUCHDOWN - THROTTLE 0% UNTIL DISARM", true)
            recorder.log('E', "SURFACE TOUCHDOWN phase=${phase} dwn=${t.lidarDwn}/${t.lidarDwnState} - throttle 0 until disarm")
        }
        if (surfTouchdown && SurfTouchdown.release(t.motorsLive, stickYNorm, t.surfTouch)) {
            surfTouchdown = false
            recorder.log('E', "SURFACE TOUCHDOWN released (${when { !t.motorsLive -> "disarmed"; !t.surfTouch -> "ESP: airborne evidence"; else -> "stick up" }})")
        }
        // נגיעה אמיתית ואחריה יציאה מ-SURFACE על הקרקע: נשארים 0% - הדרך הבטוחה היא ניתוק
        if (surfTouchdown && !surfNow && surfLast) _notice.value = Notice("LANDED - DISARM NOW", false)
        if (!surfNow && surfLast) surfaceHandoff("srs=${t.surfState} fmd=${t.inavModes.toString(16)}")
        surfLast = surfNow
        if (surfHandoffHold && !stickHeld) {
            surfHandoffHold = false
            recorder.log('E', "SURFACE HANDOFF released (stick centered)")
        }
        surfCenterPending?.let { pc -> if (!stickHeld) { _altHoldNeutral.value = pc; surfCenterPending = null } }

        // באוויר, מקל מלא למטה משחרר את ההחזקה: אחרת הקלה קלה מהקצה הייתה
        // מקפיצה את המצערת חזרה לערך המוחזק
        if (holdOutput && (!stickHeld || (phase == FlightPhase.AIRBORNE && stickYNorm <= -1f))) holdOutput = false
        if (airborneMapHeld && (!stickHeld || phase == FlightPhase.AIRBORNE)) airborneMapHeld = false

        // המרכז הידני לפי שלב הטיסה: על הקרקע שחרור המקל = מצערת מינימלית
        val manualTarget = if (phase == FlightPhase.AIRBORNE) _hoverPercent.value else 0f

        centerNow = when {
            nav        -> _altHoldNeutral.value
            stickHeld  -> centerNow                      // מקל מוחזק - אין הזזת מרכז
            centerJump -> { centerJump = false; manualTarget }
            else       -> approach(centerNow, manualTarget,
                                   CENTER_SLEW_PER_S * SEND_INTERVAL_MS / 1000f)
        }
    }

    // ── פקודות ──

    // ═══ בקשת מצב טיסה ═══
    //   הבחירה נכנסת לשורת השליטה כשדה MODE, ה-FPGA ממיר אותה לערך AUX
    //   ו-INAV מחליט. היא **אינה** משנה כאן שום דבר: לא את המצערת, לא את
    //   המרכז ולא את הממשק מעבר להצגת הבקשה עצמה. מה שפעיל בפועל מגיע ב-FMD.
    //
    //   החסימה כאן זהה לזו שמכבה את הכפתור בממשק, והיא נבדקת שוב כדי שגם
    //   לחיצה שהספיקה להתקבל רגע אחרי שהתנאים השתנו לא תעבור.
    //
    //   ═══ WP = בחירת המשימה ═══
    //   מנוטרל ועל הקרקע: בחירת WP שולחת CMD:MPREP - ה-ESP קורא את המשימה מ-INAV
    //   ושומר תמונה שלה. ביטול הבחירה (כשמנוטרל) שולח CMD:MCANCEL. שום דבר מזה
    //   לא מניע מנועים: START נפרד, ורק אחרי ARM.
    fun setMode(mode: FlightMode) {
        val t = _telemetry.value
        if (mode.blockedBy(t) != null) return
        val prev = _control.value.mode
        if (mode != prev) startWaitCancel("mode ${mode.label}")
        // E1022A: SURFACE רק ב-POSHOLD - בחירת מצב אחר מבטלת את הבקשה
        _control.update { it.copy(mode = mode, surface = it.surface && mode == FlightMode.POSHOLD) }
        // E1022B: ל-ALTHOLD (גם הוא שומר גובה) - אותה מסירה. ל-ANGLE / RTH / WP - רק מפסיקים את
        //   מיפוי ה-SURFACE; המעבר עצמו מטופל כמו כל מעבר מצב (updateThrottleCenter).
        if (mode != FlightMode.POSHOLD && surfLast) {
            if (mode == FlightMode.ALTHOLD) surfaceHandoff("mode ALTHOLD")
            else { surfLast = false; surfCenterPending = null; recorder.log('E', "SURFACE OFF (mode ${mode.label})") }
        }
        if (mode == prev) return

        // צליל רק כש-INAV באמת מבצע את המצב (WP מאושר דרך MPREP / START)
        modeConfirmPending = if (mode != FlightMode.WP) mode else null

        if (!MissionConfig.ENABLED || t.mission.active) return
        val groundIdle = !t.motorsLive && !t.phase.offGround
        if (mode == FlightMode.WP && groundIdle) {
            _prepPending.value = true
            prepSentMs = System.currentTimeMillis()
            _notice.value = Notice("PREPARING MISSION...", true)
            viewModelScope.launch { udp.send("CMD:MPREP") }
        } else if (prev == FlightMode.WP && groundIdle) {
            _prepPending.value = false
            viewModelScope.launch { udp.send("CMD:MCANCEL") }
        }
    }

    // ═══ E1022B: מסירת המצערת ביציאה מ-SURFACE ═══
    //   ב-SURFACE מקל מלא למטה = 0%. ביציאה (אוטומטית: אובדן DOWN / פער / מצפן, או OFF של
    //   הטייס) INAV נכנס מחדש למצב ולוקח את המצערת של אותו רגע כנייטרל. לכן:
    //     · ה-FPGA (F1022B) שולח את המרכז (SNT ⇒ THS) 100ms לפני היציאה, ומחזיק אותו אחריה
    //       עד שהאפליקציה שולחת אותו ערך (±1%) - בלי טיימר;
    //     · כאן הרצפה חוזרת ל-20% מיד, והפלט = המרכז עד שהטייס משחרר את המקל.
    //   כך אחרי יציאה הרחפן מחזיק גובה, ופקודת ירידה דורשת דחיפה חדשה של המקל.
    //   STICK (ההגדרה אצלנו): המרכז נשאר. MID_STICK: אחרי השחרור המרכז חוזר ל-thr_mid.
    private fun surfaceHandoff(why: String) {
        if (!surfLast) return
        surfLast = false
        surfCenterPending = null
        // E1022C: אותו גבול כמו ה-THS שה-FPGA מחזיק (20%..90%), כדי שהערכים ייפגשו בדיוק
        surfHandoffValue = _altHoldNeutral.value.coerceIn(SURF_HOLD_MIN_PCT, SURF_HOLD_MAX_PCT)
        _altHoldNeutral.value = surfHandoffValue          // זה הנייטרל ש-INAV לקח בכניסה מחדש
        surfHandoffHold  = abs(stickYNorm) >= RECENTER_EPS
        if (_telemetry.value.altHoldThrMode == AltHoldThrottle.MID_STICK) surfCenterPending = INAV_THR_MID_PCT
        if (surfHandoffHold) _notice.value = Notice("SURFACE OFF - RELEASE THROTTLE STICK", false)
        recorder.log('E', "SURFACE OFF ($why) hold=${surfHandoffHold} center=${surfHandoffValue}")
    }

    // ═══ E1022A: SURFACE ידני ═══
    //   רק כש-POSHOLD נבחר. זו בקשה בלבד: ה-ESP בודק DOWN, rangefinder ב-INAV ומשימה,
    //   ה-FPGA בודק את CH7 היוצא ומוסיף את הכניסה מחדש. מה שקורה בפועל - FMD / SRS.
    //   לא נשמר בין הפעלות, ומתבטל בניתוק החימוש ובמעבר למצב אחר.
    fun setSurface(on: Boolean) {
        _control.update { it.copy(surface = on && it.mode == FlightMode.POSHOLD) }
        if (!_control.value.surface) surfaceHandoff("app OFF")
        recorder.log('E', "SURFACE REQ ${if (_control.value.surface) 1 else 0}")
    }

    // ═══ הגבלת Roll/Pitch ל-75% (ANGLE / ALTHOLD) - מתג, נשמר בין הפעלות ═══
    //   ה-FPGA מחיל אותה במעבר הדרגתי, ואינה חוסמת שום פקודת חירום.
    fun setRpLimit(on: Boolean) {
        _control.update { it.copy(rpLimit = on) }
        prefs.edit().putBoolean(KEY_RP_LIMIT, on).apply()
    }

    // ═══ הגדרות העקיפה ═══
    //   מותר רק כשמנוטרל, על הקרקע, והמשימה לא פעילה. הצלחה רק ב-MSET:OK.
    fun settingsBlockedBy(): String? {
        val t = _telemetry.value
        return when {
            t.motorsLive                          -> "DISARM FIRST"
            t.phase != FlightPhase.GROUND_DISARMED -> "ON THE GROUND ONLY"
            t.mission.active                      -> "MISSION RUNNING"
            !t.mission.present                    -> "NO ESP"
            else                                  -> null
        }
    }

    fun saveMissionSettings(s: MissionSettings) {
        settingsBlockedBy()?.let { fail("SETTINGS: $it"); return }
        s.problem(_telemetry.value.fwdMaxCm)?.let { fail("OUT OF RANGE: $it"); return }
        _settingsPending.value = true
        settingsSentMs = System.currentTimeMillis()
        _notice.value = Notice("SAVING SETTINGS...", true)
        viewModelScope.launch { udp.send(s.toCmd()) }
    }

    // ═══ SHORT / LONG לכל חיישן ═══
    //   ה-ESP מחיל בחיישן, קורא בחזרה, שומר ב-NVS ועונה. הממשק מציג רק את
    //   המצב שדווח בטלמטריה (FLM / DLM) - לא את מה שביקשנו.
    fun setLidarMode(front: Boolean, long: Boolean) {
        val t = _telemetry.value
        if (t.motorsLive || t.phase != FlightPhase.GROUND_DISARMED) { fail("LIDAR MODE: DISARM ON THE GROUND"); return }
        val w = if (front) 'F' else 'D'
        if (w in _lidarPending.value) return              // החלפה קודמת עוד לא הסתיימה
        _lidarPending.update { it + w }
        lidarSentMs[w] = System.currentTimeMillis()
        lidarWant[w]   = long
        lidarOkMs.remove(w)
        viewModelScope.launch { udp.send("LMODE:$w:${if (long) 1 else 0}") }
    }

    // משוב לחיצה לכל כפתור במסך (LocalTap)
    fun tap() = beeper.tap()

    private fun fail(text: String) {
        _notice.value = Notice(text, false)
        beeper.err()
    }

    private fun success(text: String) {
        _notice.value = Notice(text, true)
        beeper.ok()
    }

    fun systemStart() {
        viewModelScope.launch { udp.send("CMD:SELF_CHECK") }
        sendRtcNow()
    }

    fun armMotors() {
        viewModelScope.launch { udp.send("CMD:ARM") }
    }

    // ניתוק וחירום מותרים תמיד, גם כשהשליטה מושעית או שהקשר מוכרז אבוד
    fun disarmMotors() {
        startWaitCancel("DISARM")
        viewModelScope.launch { udp.send("CMD:DISARM") }
        unlatch()
    }

    fun abortFlight() {
        startWaitCancel("ABORT")
        viewModelScope.launch { udp.send("CMD:ABORT") }
        unlatch()
    }

    fun resetPreFlightCheck() { _preFlightState.value = PreFlightState.Idle }

    // ═══ Mission FSM ═══
    //   האפליקציה רק מבקשת. ה-FPGA בודק את 16 תנאי ה-SELF_CHECK ומחליט, וגם
    //   דחייה חוזרת בטלמטריה (MRS / MCK). כאן נחסמת רק לחיצה שבוודאות תידחה.
    //   STOP מותר תמיד, כמו ניתוק וחירום: הוא מחזיר מיד את השליטה לאפליקציה.
    //   25.10: STOPPED (הרחפן עומד, ‎≤15cm/s) לא חוסם את הכפתור: בריחוף ב-POSHOLD היא נכשלת
    //   ~40% מהזמן (blackbox), ולכן הכפתור הבהב ו-START נדחה. כאן START מחכה לרגע שבו גם היא
    //   עוברת (missionStartFullBlockedBy), עד START_WAIT_MS. ה-FPGA עדיין בודק הכל ומחליט.
    fun missionStartBlockedBy(): String? =
        _telemetry.value.mission.startBlockedBy(_control.value.mode, _controlAuthority.value.streaming,
                                                MissionCheck.ALL and MissionCheck.STOPPED.inv())

    private fun missionStartFullBlockedBy(): String? =
        _telemetry.value.mission.startBlockedBy(_control.value.mode, _controlAuthority.value.streaming)

    // START: רק אחרי ARM, ובלחיצה שנייה (בממשק). הצליל - רק כשהמשימה באמת התחילה.
    fun missionStart() {
        if (!MissionConfig.ENABLED || missionStartBlockedBy() != null) return
        startWaitUntilMs = System.currentTimeMillis() + START_WAIT_MS
        _startWaiting.value = true
        if (missionStartFullBlockedBy() == null) sendMissionStart()
        else {
            _notice.value = Notice("START: WAITING FOR STABLE HOVER", true)
            recorder.log('E', "START waiting for STOPPED")
        }
    }

    private fun sendMissionStart() {
        startPendingMs = System.currentTimeMillis()
        viewModelScope.launch { udp.send("CMD:MSTART") }
    }

    private fun startWaitEnd() { startWaitUntilMs = 0L; _startWaiting.value = false }

    // ביטול מיידי בפעולת משתמש / אובדן קשר, בלי לחכות לטלמטריה הבאה
    private fun startWaitCancel(why: String) {
        if (startWaitUntilMs == 0L) return
        startWaitEnd()
        recorder.log('E', "START wait cancelled ($why)")
    }

    fun missionStop() {
        if (!MissionConfig.ENABLED) return
        startWaitCancel("STOP")
        viewModelScope.launch { udp.send("CMD:MSTOP") }
    }

    // ניתוק יזום הוא סוף טיסה מוצהר, ולכן הבקשה חוזרת ל-ANGLE: החימוש הבא
    // יתחיל מטיסה ידנית, ולא ייכנס למצב ניווט ברגע שהמנועים יתעוררו.
    private fun unlatch() {
        startWaitCancel("unlatch")
        _latched.value = false
        stickYNorm     = 0f
        holdOutput     = false
        airborneMapHeld = false
        takeoffHold    = false
        centerNow      = 0f
        _control.update {
            it.copy(throttle = 0f, descentIntent = false, mode = FlightMode.ANGLE)
        }
    }

    // ═══════════════════════════════════════════════════════
    //   רשות השליטה
    //
    //   ⚠️ פתיחת זרם השליטה מחייבת עדות חיובית, לא היעדר בעיה. ברגע פתיחת
    //      האפליקציה ערכי השליטה שלה הם אפסים, וכל עוד לא הגיעה טלמטריה אין
    //      לה מושג אם הרחפן על הקרקע או באוויר. שידור במצב הזה - למשל אחרי
    //      קריסה והפעלה מחדש תוך כדי טיסה - פירושו מצערת אפס.
    //
    //   לכן מצב ההתחלה הוא WAITING, שבו נשלחת רק פעימת לב, ואת השליטה פותחת
    //   רק אחת משתי אלה:
    //     · טלמטריה שמראה מצב קרקע בטוח - מנועים מנוטרלים, שלב קרקע, וה-FPGA
    //       אינו נעול. שם אפסים הם בדיוק הערכים הנכונים.
    //     · אישור מפורש של הטייס (resumeControl), שמתחיל מהמצערת הידועה
    //       שה-ESP מדווח בשדה CTH - ולא מאפס.
    // ═══════════════════════════════════════════════════════
    private fun updateControlAuthority(t: TelemetryData) {
        val safeGround = !t.isArmed && !t.phase.offGround && !t.fpgaState.locked

        when (_controlAuthority.value) {
            ControlAuthority.ACTIVE -> { /* כבר פעיל - רק אובדן קשר עוצר אותו */ }
            else -> if (safeGround) activateOnGround()
                    else if (_controlAuthority.value == ControlAuthority.WAITING)
                        _controlAuthority.value = ControlAuthority.SUSPENDED
        }
    }

    // מצב קרקע בטוח: המנועים מנוטרלים והרחפן על הקרקע, ולכן אפסים הם הערכים
    // הנכונים. המצערת נשארת נעולה על 0 עד שהטייס דוחף את המקל מעבר לסף,
    // והבקשה מתחילה מ-ANGLE - נקודת הפתיחה של כל טיסה.
    private fun activateOnGround() {
        startWaitCancel("control reset")
        _latched.value = false
        stickYNorm     = 0f
        holdOutput     = false
        airborneMapHeld = false
        takeoffHold    = false
        takeoffValue   = 0f
        centerNow      = 0f
        _control.update {
            it.copy(throttle = 0f, yaw = 0f, pitch = 0f, roll = 0f,
                    descentIntent = false, mode = FlightMode.ANGLE)
        }
        _controlAuthority.value = ControlAuthority.ACTIVE
    }

    // ═══ חזרה מפורשת לשליטה על רחפן שכבר מחומש או באוויר ═══
    //
    //   ⚠️ **לא פותחים את זרם השליטה על מצב לא ידוע.** כל תנאי ב-
    //      `resumeBlockedBy` חייב להתקיים לפני שהרשות נפתחת: מצערת ידועה
    //      (CTH), דיווח מצבים תקף (FMV), אין Failsafe ב-INAV, ה-FPGA אינו
    //      ב-LINK_LOSS ואינו ב-ABORT, ומצב INAV הפעיל מזוהה כאחד מחמשת
    //      המצבים שלנו. חסר אחד מהם ⇒ יוצאים בלי לגעת בכלום.
    //
    //   ⚠️ **אין יציאה אוטומטית מ-LINK_LOSS.** ה-FPGA נשאר נעול שם עד פקודת
    //      ניתוק מפורשת, וזה בכוונת תכנון. חזרה לשליטה לא מנסה לעקוף את זה:
    //      היא פשוט חסומה, והפעולות היחידות שנשארות הן DISARM ו-ABORT.
    //
    //   נקודת ההתחלה היא המצערת שה-ESP באמת העביר ל-FPGA לאחרונה, כדי
    //   שהרגע שבו האפליקציה מתחילה לשדר שוב לא ישנה את המצערת בכלל. אותו
    //   עיקרון חל על המצב: הבקשה מסונכרנת למה ש-INAV מבצע, לא ל-ANGLE.
    fun resumeControl() {
        if (_controlAuthority.value != ControlAuthority.SUSPENDED) return
        val t = _telemetry.value
        if (t.resumeBlockedBy() != null) return

        // אותה בדיקה של resumeBlockedBy, וכאן היא גם מפיקה את הערך עצמו
        val inavMode = FlightMode.fromInav(t.inavModes, t.inavModesValid) ?: return
        // Mission FSM מופעל: WP הוא של ה-FSM בלבד, והבקשה שלנו בזמן משימה היא POSHOLD.
        // בקשת WP הייתה נחשבת השתלטות ועוצרת את המשימה.
        val actualMode = if (MissionConfig.ENABLED && inavMode == FlightMode.WP) FlightMode.POSHOLD else inavMode

        val knownF   = t.lastThrottlePct.coerceIn(0f, 100f)
        stickYNorm   = 0f
        holdOutput   = false
        airborneMapHeld = false
        centerNow    = knownF                 // שחרור המקל מחזיק את הערך הידוע
        centerJump   = false
        phaseLast    = t.phase
        takeoffHold  = t.phase == FlightPhase.TAKEOFF_CONFIRM
        takeoffValue = if (takeoffHold) knownF else 0f
        _latched.value = true
        _control.update {
            it.copy(throttle = knownF, yaw = 0f, pitch = 0f, roll = 0f,
                    descentIntent = false, mode = actualMode)
        }
        _controlAuthority.value = ControlAuthority.ACTIVE
    }

    // ═══ שעון בקר הטיסה ═══
    //   ל-Matek אין שעת אמת בלי GPS, ולכן קובצי ה-Blackbox נושאים תאריך חסר
    //   משמעות ואי אפשר להצליב לוג לטיסה. הטאבלט שולח את זמן ה-UTC שלו,
    //   וה-ESP כותב אותו לבקר הטיסה ב-MSP_SET_RTC.
    //
    //   ⚠️ זו הגדרה ולא שליטה: היא נשלחת גם כשאין רשות שליטה, אינה עוברת
    //      דרך ה-FPGA ואינה נוגעת בערוצים, במצבים או בחימוש.
    private fun sendRtcNow() {
        val now = System.currentTimeMillis()
        lastRtcMs = now
        viewModelScope.launch { udp.send("RTC:${now / 1000} MS:${now % 1000}") }
    }

    // התחברות, או חזרה אחרי נפילת קשר, היא נקודת הסנכרון הטבעית. מעבר לזה
    // רענון כל כמה דקות, כך שגם שינוי שעה בטאבלט יגיע לבקר הטיסה.
    private fun noteConnected() {
        val reconnected = _linkState.value != LinkState.CONNECTED
        _linkState.value = LinkState.CONNECTED
        val now = System.currentTimeMillis()
        if (reconnected || lastRtcMs == 0L || now - lastRtcMs >= RTC_RESEND_MS) sendRtcNow()
    }

    // ── לולאות ──
    private fun launchSendLoop() = viewModelScope.launch(Dispatchers.IO) {
        while (isActive) {
            if (!_controlAuthority.value.streaming) {
                // ═══ אין רשות שליטה: לא משדרים שורות שליטה בכלל ═══
                // זה נכון בשלושה מצבים שונים - לפני שהתקבלה טלמטריה, אחרי
                // אובדן קשר, ומול רחפן מחומש שטרם אושרה עליו חזרה לשליטה.
                // בכולם חבילה עם ערכי ברירת מחדל היא מצערת אפס.
                // פעימת הלב היחידה היא CMD:APP_PING: ה-ESP עונה עליה מקומית
                // ו**אינו** מעביר ל-FPGA, ולכן היא אינה מאפסת את מונה אובדן הקשר.
                val now = System.currentTimeMillis()
                if (now - lastPingMs >= APP_PING_MS) {
                    lastPingMs = now
                    udp.send("CMD:APP_PING")
                }
            } else {
                // ═══ משימה פעילה באוויר: המצערת = הנייטרל שה-FPGA מדווח (MNT) ═══
                //   כך המצערת של האפליקציה כבר שווה לנייטרל כשהשליטה חוזרת לטייס,
                //   והמסירה לא מזיזה את הגובה. תזוזת מקל עדיין עוברת - והיא השתלטות.
                val tm = _telemetry.value
                if (tm.mission.active && tm.phase == FlightPhase.AIRBORNE && tm.mission.neutralHalf in 0..200) {
                    _altHoldNeutral.value = tm.mission.neutralHalf / 2f
                    if (!_latched.value) _latched.value = true
                }
                updateThrottleCenter()
                _control.update { cur ->
                    val target = throttleTarget()
                    // מצב ניווט: המצערת היא פקודת קצב, לא כוח - בלי מגביל עלייה.
                    // מקל מוחזק: הטייס קובע ישירות - היעד נשלח כמו שהוא בעדכון
                    // ה-20Hz הבא, למעלה ולמטה, בלי מגביל הקצב. המגביל והחזרה
                    // המהירה לריחוף פועלים רק אחרי שחרור המקל.
                    val stickHeld = abs(stickYNorm) >= RECENTER_EPS
                    val next = if (navLast || stickHeld) target
                               else slew(cur.throttle, target, fastHoverReturn(cur.throttle))
                    cur.copy(
                        throttle      = next.coerceIn(0f, 100f),
                        descentIntent = stickYNorm <= -DESCENT_EPS,
                        // אישור השלב שהתקבל - ה-ESP משחרר לפיו את רצפת המצערת
                        phaseAck      = _telemetry.value.phase.code,
                        // E1022B: מרכז המקל ב-SURFACE - ה-ESP מחזיק אותו ביציאה
                        surfCenterHalf = if (surfLast) (_altHoldNeutral.value * 2f).roundToInt().coerceIn(0, 200) else -1
                    )
                }
                udp.send(_control.value.toUdpString())
            }
            delay(SEND_INTERVAL_MS)
        }
    }

    private fun launchReceiveLoop() = viewModelScope.launch {
        udp.incoming.collect { packet -> parsePacket(packet) }
    }

    private fun launchLinkWatchdog() = viewModelScope.launch {
        while (isActive) {
            delay(100L)
            val last = _telemetry.value.lastUpdateMs
            if (last > 0L && System.currentTimeMillis() - last > LINK_TIMEOUT_MS) {
                if (_linkState.value != LinkState.LOST) onLinkLost()
            }
        }
    }

    // ═══════════════════════════════════════════════════════
    //   אובדן קשר בין הטאבלט ל-ESP
    //
    //   מרגע ההכרזה, לולאת השליחה מפסיקה לשדר שורות שליטה. ה-FPGA מזהה שאין
    //   פקודות, עוצר את זרם ה-RC, ו-INAV מפעיל את ה-Failsafe שלו. פעימת הלב
    //   היחידה שממשיכה היא CMD:APP_PING, שה-ESP עונה עליה מקומית ואינו מעביר
    //   ל-FPGA - ולכן היא לא מאפסת את מונה אובדן הקשר.
    //
    //   ⚠️ ב-INAV מוגדר failsafe_procedure = LAND: אובדן קשר באוויר מפעיל נחיתת
    //      חירום של INAV (לא כיבוי מנועים). בדיקת אובדן קשר - קודם על השולחן
    //      בלי פרופלורים, ורק אחר כך בטיסה בגובה נמוך.
    //
    //   אין חזרה אוטומטית לשליטה: ה-FPGA נשאר נעול עד DISARM, ורשות השליטה
    //   נפתחת מחדש רק כשהטלמטריה מראה שהרחפן מנוטרל ועל הקרקע.
    // ═══════════════════════════════════════════════════════
    private fun onLinkLost() {
        recorder.log('E', "LINK LOST")
        startWaitCancel("link lost")
        _linkState.value        = LinkState.LOST
        _controlAuthority.value = ControlAuthority.LOST
        _systemStarted.value    = false
        stickYNorm = 0f
        _control.update { it.copy(yaw = 0f, pitch = 0f, roll = 0f, descentIntent = false) }
    }

    // ── פענוח חבילות ──
    private fun parsePacket(packet: String) {
        recorder.log('R', packet)
        when (packet) {
            "START:OK"  -> { _systemStarted.value = true;  return }
            "START:ERR" -> { _systemStarted.value = false; return }
            "CHECK:OK"  -> { _preFlightState.value = PreFlightState.Ready; return }
            // ⚠️ PONG מוכיח שהקישור חזר, אבל **אינו** פותח שליטה: הוא לא נושא
            //    שום מידע על מצב הרחפן. את השליטה פותחת רק טלמטריה.
            "APP:PONG"  -> { noteConnected(); return }
        }
        if (packet.startsWith("CHECK:ERR:")) {
            _preFlightState.value = PreFlightState.Fault(packet.removePrefix("CHECK:ERR:"))
            return
        }
        // תשובות לפקודות - אינן טלמטריה, ואסור שיעדכנו אותה
        if (handleReply(packet)) return

        val fields = packet.split(" ")
            .mapNotNull {
                val idx = it.indexOf(':')
                if (idx < 1) null else it.substring(0, idx) to it.substring(idx + 1)
            }
            .toMap()

        if (fields.isEmpty()) return

        noteConnected()

        val prevMission = _telemetry.value.mission
        val prevArmed   = _telemetry.value.isArmed
        _telemetry.update { cur ->
            val fwd = fields["FWD"]?.toIntOrNull() ?: cur.lidarFwd
            val dwn = fields["DWN"]?.toIntOrNull() ?: cur.lidarDwn
            cur.copy(
                batteryPercent = fields["BAT"]?.toIntOrNull()   ?: cur.batteryPercent,
                isArmed        = fields["ARM"]?.toIntOrNull()   == 1,
                lidarFwd       = fwd,
                lidarDwn       = dwn,
                lidarFwdState  = fields["FOK"]?.toIntOrNull()
                                 ?: if (fwd < 0) LidarState.FAULT else LidarState.VALID,
                lidarDwnState  = fields["DOK"]?.toIntOrNull()
                                 ?: if (dwn < 0) LidarState.FAULT else LidarState.VALID,
                fwdLong        = fields["FLM"]?.toIntOrNull()?.let { it == 1 } ?: cur.fwdLong,
                dwnLong        = fields["DLM"]?.toIntOrNull()?.let { it == 1 } ?: cur.dwnLong,
                fwdMaxCm       = fields["FMX"]?.toIntOrNull()   ?: cur.fwdMaxCm,
                dwnMaxCm       = fields["DMX"]?.toIntOrNull()   ?: cur.dwnMaxCm,
                lidarModeKnown = cur.lidarModeKnown || fields.containsKey("FLM"),
                phase          = fields["PHS"]?.toIntOrNull()?.let { FlightPhase.from(it) } ?: cur.phase,
                fwdActive      = fields["FWA"]?.toIntOrNull()   == 1,
                lastThrottlePct = fields["CTH"]?.toFloatOrNull() ?: cur.lastThrottlePct,
                inavModes      = fields["FMD"]?.toIntOrNull(16) ?: cur.inavModes,
                inavModesValid = fields["FMV"]?.toIntOrNull()   == 1,
                altHoldThrMode = fields["ATM"]?.toIntOrNull()   ?: cur.altHoldThrMode,
                surfState      = fields["SRS"]?.substringBefore(',')?.toIntOrNull() ?: cur.surfState,
                surfLosses     = fields["SRS"]?.split(',')?.getOrNull(1)?.toIntOrNull() ?: cur.surfLosses,
                surfHandoffs   = fields["SRS"]?.split(',')?.getOrNull(2)?.toIntOrNull() ?: cur.surfHandoffs,
                // E1022G: שדה 5 - חסר (ESP ישן) ⇒ false, כלומר אין נעילת מגע
                surfTouch      = fields["SRS"]?.let { SurfTouchdown.parseSrs(it) } ?: cur.surfTouch,
                // חבילת התצוגה (5Hz): ATR/ATP/HDG, GPS ו-MAG חסרים בחצי מהחבילות, ואז נשמר הערך האחרון
                attRoll        = fields["ATR"]?.toFloatOrNull() ?: cur.attRoll,
                attPitch       = fields["ATP"]?.toFloatOrNull() ?: cur.attPitch,
                heading        = fields["HDG"]?.toIntOrNull()   ?: cur.heading,
                fcLinkOK       = fields["FCL"]?.toIntOrNull()   == 1,
                failsafeWarn   = fields["FSW"]?.toIntOrNull()   == 1,
                checks         = fields["SYS"]?.let { SystemChecks.parse(it) } ?: cur.checks,
                gpsStatus      = fields["GPS"]                  ?: cur.gpsStatus,
                gpsLat         = fields["LAT"]?.toDoubleOrNull() ?: cur.gpsLat,
                gpsLon         = fields["LON"]?.toDoubleOrNull() ?: cur.gpsLon,
                gpsSats        = fields["SAT"]?.toIntOrNull()   ?: cur.gpsSats,
                gpsHdop        = fields["HDP"]?.toIntOrNull()   ?: cur.gpsHdop,
                gpsEph         = fields["EPH"]?.toIntOrNull()   ?: cur.gpsEph,
                gpsEpv         = fields["EPV"]?.toIntOrNull()   ?: cur.gpsEpv,
                magValid       = fields["MGV"]?.toIntOrNull()?.let { it == 1 } ?: cur.magValid,
                magX           = fields["MGX"]?.toIntOrNull()   ?: cur.magX,
                magY           = fields["MGY"]?.toIntOrNull()   ?: cur.magY,
                magZ           = fields["MGZ"]?.toIntOrNull()   ?: cur.magZ,
                magHealth      = fields["MGH"]?.toIntOrNull()   ?: cur.magHealth,
                fpgaState      = FpgaState.from(fields["FST"]?.toIntOrNull() ?: -1),
                fpgaDebug      = fields["FDB"]                  ?: cur.fpgaDebug,
                batteryVoltCv  = fields["BTV"]?.toIntOrNull()   ?: cur.batteryVoltCv,
                batteryState   = fields["BTS"]?.toIntOrNull()   ?: cur.batteryState,
                batteryCells   = fields["BTC"]?.toIntOrNull()   ?: cur.batteryCells,
                espVersion     = fields["FWV"]                  ?: cur.espVersion,
                fpgaVersion    = fields["FPV"]                  ?: cur.fpgaVersion,
                diagLoopMs     = dgn(fields, 0) ?: cur.diagLoopMs,
                diagCtrlGapMs  = dgn(fields, 1) ?: cur.diagCtrlGapMs,
                diagMspReq     = dgn(fields, 2) ?: cur.diagMspReq,
                diagMspFail    = dgn(fields, 3) ?: cur.diagMspFail,
                // שדות המשימה - חבילה בלי MSV משאירה את הערך הקודם
                mission        = MissionStatus.parse(fields, cur.mission),
                lastUpdateMs   = System.currentTimeMillis(),
            )
        }

        updateLidarView(_telemetry.value)

        // רשות השליטה נגזרת מהטלמטריה שהרגע התקבלה - אחרי שהיא נכתבה
        updateControlAuthority(_telemetry.value)
        noteMissionAndMode(prevMission, _telemetry.value)
        // E1022A: ניתוק חימוש (נחיתה / DISARM) מבטל את בקשת SURFACE
        if (prevArmed && !_telemetry.value.isArmed && _control.value.surface) setSurface(false)
    }

    // ═══ תצוגת הלידאר: הסיווג המפורט (FRC / DRC) כשקיים, אחרת FOK / DOK ═══
    private fun lidarClass(missionCls: Int, present: Boolean, basic: Int): Int = when {
        present -> missionCls
        else -> when (basic) {
            LidarState.VALID     -> MissionLidar.VALID
            LidarState.TOO_CLOSE -> MissionLidar.TOO_CLOSE
            LidarState.NO_RETURN -> MissionLidar.OUT_OF_RANGE
            LidarState.UNCERTAIN -> MissionLidar.UNCERTAIN
            else                 -> MissionLidar.FAULT
        }
    }

    private fun updateLidarView(t: TelemetryData) {
        val now = System.currentTimeMillis()
        val f = fwdSmoother.update(lidarClass(t.mission.frontCls, t.mission.present, t.lidarFwdState),
                                   t.lidarFwd, t.fwdMaxCm, now)
        val d = dwnSmoother.update(lidarClass(t.mission.downCls, t.mission.present, t.lidarDwnState),
                                   t.lidarDwn, t.dwnMaxCm, now)
        _lidarView.value = f to d
    }

    // ═══ תשובות ה-ESP: MSET / LMODE / MPREP / MCANCEL ═══
    private fun handleReply(p: String): Boolean {
        when {
            p == "MSET:OK" -> {
                _settingsPending.value = false
                success("SETTINGS SAVED + CONFIRMED BY FPGA")
            }
            p.startsWith("MSET:ERR:") -> {
                _settingsPending.value = false
                fail("SETTINGS NOT SAVED: " + p.removePrefix("MSET:ERR:"))
            }
            p.startsWith("LMODE:OK ") -> {
                // "LMODE:OK F:1"
                val w = p.getOrNull(9) ?: return true
                // לא משחררים כאן: הכפתור נשאר נעול עד שהטלמטריה מראה את המצב (noteLidarModes)
                if (w in _lidarPending.value) lidarOkMs[w] = System.currentTimeMillis()
                val name = if (w == 'F') "FRONT" else "DOWN"
                success("$name LIDAR " + if (p.endsWith(":1")) "LONG" else "SHORT")
            }
            p.startsWith("LMODE:ERR:") -> {
                // "LMODE:ERR:<סיבה> F"
                val w = p.lastOrNull()
                if (w == 'F' || w == 'D') _lidarPending.update { it - w } else _lidarPending.value = emptySet()
                fail("LIDAR MODE NOT CHANGED: " + p.removePrefix("LMODE:ERR:"))
            }
            p.startsWith("MPREP:OK") -> {
                _prepPending.value = false
                success("MISSION PREPARED" + p.removePrefix("MPREP:OK"))
            }
            p.startsWith("MPREP:ERR:") -> {
                _prepPending.value = false
                fail("MISSION NOT PREPARED: " + p.removePrefix("MPREP:ERR:"))
            }
            p == "MCANCEL:OK" || p.startsWith("MCANCEL:ERR:") -> {
                _notice.value = Notice(if (p == "MCANCEL:OK") "MISSION DESELECTED" else p, p == "MCANCEL:OK")
            }
            else -> return false
        }
        return true
    }

    private fun dgn(f: Map<String, String>, i: Int): Int? =
        f["DGN"]?.split(",")?.getOrNull(i)?.toIntOrNull()

    // ═══ מצב מבוקש מול מצב פעיל ═══
    //   מחומש, ומצב (לא WP) שביקשנו ו-INAV לא מפעיל יותר משנייה ⇒ התראה עם הסיבה.
    //   הסיבה נגזרת ממה שידוע: דיווח INAV, Failsafe, מצפן (שער ה-FPGA), GPS.
    private fun updateModeIssue(t: TelemetryData, now: Long) {
        val req = _control.value.mode
        val act = FlightMode.fromInav(t.inavModes, t.inavModesValid)
        val mismatch = t.isArmed && req != FlightMode.WP && act != req
        if (!mismatch) { modeMismatchSinceMs = 0L; setModeIssue(null); return }
        if (modeMismatchSinceMs == 0L) modeMismatchSinceMs = now
        if (now - modeMismatchSinceMs < MODE_ISSUE_MS) return
        val why = when {
            !t.inavModesValid                              -> "NO INAV MODE STATUS"
            (t.inavModes and InavMode.FAILSAFE) != 0       -> "INAV FAILSAFE"
            req.needsGps && t.magHealth != MagHealth.OK    -> "COMPASS NOT VERIFIED - FPGA HOLDS ${act?.label ?: "?"}"
            req.needsGps && !t.gpsFix3D                    -> "NO GPS 3D FIX"
            else                                           -> "INAV DID NOT ENGAGE"
        }
        setModeIssue("${req.label} NOT ACTIVE: $why")
    }

    private fun setModeIssue(v: String?) {
        if (_modeIssue.value == v) return
        _modeIssue.value = v
        recorder.log('E', "MODE ISSUE: ${v ?: "CLEARED"}")
    }

    // ═══ צלילים לפי מה שקרה בפועל: START שהתחיל / נדחה, ומצב טיסה ש-INAV אישר ═══
    private fun noteMissionAndMode(prev: MissionStatus, t: TelemetryData) {
        val m = t.mission
        val now = System.currentTimeMillis()
        updateModeIssue(t, now)

        if (startPendingMs != 0L) {
            when {
                m.active && !prev.active -> { startPendingMs = 0L; startWaitEnd(); success("MISSION STARTED") }
                m.fresh && m.reason == MissionReason.START_REJECTED && !m.active && now - startPendingMs > 300L -> {
                    startPendingMs = 0L
                    val failing = MissionCheck.failing(m.checks)
                    // 25.10: נדחה רק כי הרחפן זז ברגע ההגעה - ממשיכים לחכות בתוך חלון ההמתנה
                    if (StartWait.retryAfterReject(now, startWaitUntilMs, failing)) {
                        recorder.log('E', "START rejected (STOPPED) - waiting again")
                    } else {
                        startWaitEnd()
                        fail("START REJECTED: " + (failing.firstOrNull() ?: "?"))
                    }
                }
                now - startPendingMs > REPLY_TIMEOUT_MS -> { startPendingMs = 0L; startWaitEnd(); fail("START: NO RESPONSE") }
            }
        } else if (startWaitUntilMs != 0L) {
            val other = missionStartBlockedBy()
            when (StartWait.step(now, startWaitUntilMs, other, missionStartFullBlockedBy())) {
                StartWait.Step.TIMEOUT -> { startWaitEnd(); fail("START: NOT STABLE FOR 5s - TRY AGAIN") }
                StartWait.Step.BLOCKED -> { startWaitEnd(); fail("START CANCELLED: $other") }
                StartWait.Step.SEND    -> sendMissionStart()
                StartWait.Step.WAIT    -> {}
            }
        }

        modeConfirmPending?.let { want ->
            if (_control.value.mode != want) modeConfirmPending = null
            else if (FlightMode.fromInav(t.inavModes, t.inavModesValid) == want) {
                modeConfirmPending = null
                success("INAV: ${want.label} ACTIVE")
            }
        }

        // תשובות שלא הגיעו
        if (_settingsPending.value && now - settingsSentMs > REPLY_TIMEOUT_MS) {
            _settingsPending.value = false; fail("SETTINGS: NO RESPONSE")
        }
        if (_prepPending.value && now - prepSentMs > REPLY_TIMEOUT_MS + 4000L) {
            _prepPending.value = false; fail("PREPARE: NO RESPONSE")
        }
        noteLidarModes(t, now)
    }

    // SHORT / LONG: שחרור הכפתור רק כשהטלמטריה מראה את המצב שביקשנו, ולא לפני LIDAR_SETTLE_MS
    private fun noteLidarModes(t: TelemetryData, now: Long) {
        _lidarPending.value.forEach { w ->
            val okMs = lidarOkMs[w]
            val reported = if (w == 'F') t.fwdLong else t.dwnLong
            when {
                okMs == null -> if (now - (lidarSentMs[w] ?: now) > REPLY_TIMEOUT_MS) {
                    _lidarPending.update { it - w }; fail("LIDAR MODE: NO RESPONSE")
                }
                reported == lidarWant[w] && now - okMs >= LIDAR_SETTLE_MS -> {
                    _lidarPending.update { it - w }; lidarOkMs.remove(w)
                }
                reported != lidarWant[w] && now - okMs > REPLY_TIMEOUT_MS -> {
                    _lidarPending.update { it - w }; lidarOkMs.remove(w)
                    fail("LIDAR MODE: NOT REPORTED")
                }
            }
        }
    }

    override fun onCleared() { recorder.log('E', "APP STOP"); recorder.close(); udp.close(); beeper.release(); super.onCleared() }
}
