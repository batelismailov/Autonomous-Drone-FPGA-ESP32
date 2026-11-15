// בדיקות יחידה על המחשב לשני התיקונים מ-25.10: נעילת מגע ב-SURFACE והמתנת START.
// הקוד הנבדק הוא DroneModels.kt עצמו (SurfTouchdown / StartWait), בלי Android.
// בנוסף: בדיקת מקור ש-DroneViewModel.kt מבטל את ההמתנה בכל המסלולים הנדרשים.
//
//   kotlinc app/src/main/java/com/drone/gcs/model/DroneModels.kt tests/AppLogicTest.kt -include-runtime -d t.jar
//   java -cp t.jar AppLogicTestKt app/src/main/java/com/drone/gcs/viewmodel/DroneViewModel.kt
import com.drone.gcs.model.*
import java.io.File

private var ok = 0
private var bad = 0
private fun chk(c: Boolean, msg: String) { if (c) { ok++; println("ok   $msg") } else { bad++; println("FAIL $msg") } }

// ── מדמה את הבלוק ב-updateThrottleCenter + throttleTarget, פריים טלמטריה אחד בכל קריאה ──
private class TdSim {
    var latched = false
    var surfLast = false
    var handoffHold = false
    val notices = mutableListOf<String>()
    // מחזיר את המצערת שהאפליקציה תשלח: 0 נעול, HANDOFF במסירה, אחרת NORMAL
    fun frame(surfNow: Boolean, stick: Float, espTouch: Boolean, motorsLive: Boolean = true): String {
        if (!latched && SurfTouchdown.latch(surfNow || surfLast, stick, espTouch)) { latched = true; notices += "TOUCHDOWN" }
        if (latched && SurfTouchdown.release(motorsLive, stick, espTouch)) latched = false
        if (latched && !surfNow && surfLast) notices += "LANDED - DISARM NOW"
        if (!surfNow && surfLast) handoffHold = stick != 0f      // surfaceHandoff: מוחזק ⇒ מרכז עד שחרור
        surfLast = surfNow
        if (handoffHold && stick == 0f) handoffHold = false
        return when { latched -> "ZERO"; handoffHold -> "HANDOFF"; else -> "NORMAL" }
    }
}

private fun touchdownTests() {
    println("── SURFACE touchdown latch ──")
    // 1: נגיעה שגויה באוויר - SURFACE, מקל מלא למטה, DOWN מראה TOO_CLOSE אבל ה-ESP לא אימת מגע
    run { val s = TdSim(); var zero = 0
        repeat(50) { if (s.frame(true, -1f, espTouch = false) == "ZERO") zero++ }
        chk(zero == 0 && !s.latched, "A1 in the air, stick full down, TOO_CLOSE seen but no ESP touch -> never 0% ($zero frames)") }
    // 2: TOO_CLOSE ישן - האפליקציה כבר לא מסתמכת עליו; בלי navTouch אין נעילה גם אחרי 5 שנ'
    run { val s = TdSim()
        repeat(50) { s.frame(true, -1f, false) }
        chk(!s.latched, "A2 stale TOO_CLOSE (ESP E1022G T2 rejects it) -> no latch") }
    // 3: נגיעה אמיתית ואז שחרור המקל (24.10 23:48) - נשאר 0%
    run { val s = TdSim()
        repeat(5) { s.frame(true, -1f, false) }
        val a = s.frame(true, -1f, true)
        val b = (0 until 30).map { s.frame(true, 0f, true) }.all { it == "ZERO" }
        chk(a == "ZERO" && b, "A3 verified touchdown -> 0%, stick released for 3s -> still 0%") }
    // 4: אובדן DOWN אחרי הנעילה - ה-ESP משאיר navTouch (T6) ⇒ נשאר 0%
    run { val s = TdSim(); s.frame(true, -1f, true)
        val held = (0 until 30).map { s.frame(true, 0f, true) }.all { it == "ZERO" }
        chk(held, "A4 DOWN lost after the latch (navTouch kept by the ESP) -> stays 0%") }
    // 5: עדות לאוויר אחרי הנעילה (קפיצה / נגיעה שגויה) - ה-ESP מוחק ⇒ משתחרר מיד
    run { val s = TdSim(); s.frame(true, -1f, true); s.frame(true, -1f, true)
        val r = s.frame(true, -1f, false)
        chk(!s.latched && r != "ZERO", "A5 ESP clears navTouch (airborne evidence) -> released immediately ($r)") }
    // 6: יציאה מ-SURFACE על הקרקע לפני ניתוק - נשאר 0% + הודעה
    run { val s = TdSim(); s.frame(true, -1f, true); s.frame(true, 0f, true)
        val r = s.frame(false, 0f, true)
        chk(r == "ZERO" && "LANDED - DISARM NOW" in s.notices, "A6 SURFACE exit on the ground before DISARM -> stays 0%, LANDED - DISARM NOW") }
    // 7: יציאה מ-SURFACE באוויר עם המקל מוחזק - לא נעול ⇒ המסירה הבטוחה (HANDOFF) כרגיל
    run { val s = TdSim(); repeat(10) { s.frame(true, -0.5f, false) }
        val r = s.frame(false, -0.5f, false)
        chk(r == "HANDOFF", "A7 SURFACE exit in the air, stick held -> normal safe handoff, not 0% ($r)") }
    // 8: יציאה מ-SURFACE ובאותו פריים ה-ESP מוחק את המגע (באוויר) - מסירה, לא 0%
    run { val s = TdSim(); s.frame(true, -1f, true)
        val r = s.frame(false, -1f, false)
        chk(r == "HANDOFF" && !s.latched, "A8 SURFACE exit + ESP airborne evidence in the same frame -> handoff, not 0% ($r)") }
    // 9: ניתוק / דחיפה למעלה משחררים
    run { val s = TdSim(); s.frame(true, -1f, true); s.frame(true, 0f, true, motorsLive = false)
        val d = !s.latched
        val s2 = TdSim(); s2.frame(true, -1f, true); s2.frame(true, 0.29f, true); val stay = s2.latched
        s2.frame(true, 0.30f, true)
        chk(d && stay && !s2.latched, "A9 DISARM releases; stick 0.29 keeps, 0.30 releases") }
    // 10: לא ב-SURFACE - אין נעילה גם עם navTouch
    run { val s = TdSim(); repeat(10) { s.frame(false, -1f, true) }
        chk(!s.latched, "A10 not in SURFACE -> no latch even with navTouch") }
    // 12 (E1022I): navTouch נקבע ביציאה עצמה ומגיע באותו פריים שבו SURFACE כבר לא פעיל ⇒ נעילה, לא מסירה
    run { val s = TdSim(); repeat(5) { s.frame(true, -1f, false) }
        val r = s.frame(false, -1f, true)
        val held = (0 until 20).map { s.frame(false, 0f, true) }.all { it == "ZERO" }
        chk(r == "ZERO" && held && "LANDED - DISARM NOW" in s.notices, "A12 navTouch set at the exit itself (same frame as SRS != 1) -> latched 0%, not handoff, stays after stick release") }
    // 13 (E1022I): navTouch מגיע רק פריים אחרי היציאה ⇒ לא ננעל (הנעילה רק ב-SURFACE או בפריים של היציאה)
    run { val s = TdSim(); repeat(5) { s.frame(true, -0.5f, false) }
        s.frame(false, -0.5f, false); val r = s.frame(false, -0.5f, true)
        chk(r == "HANDOFF" && !s.latched, "A13 navTouch one frame after the exit -> no latch, normal handoff ($r)") }
    // 11: פענוח SRS - ESP ישן (4 שדות) ⇒ false
    chk(!SurfTouchdown.parseSrs("1,0,0,95") && SurfTouchdown.parseSrs("1,0,0,95,1") && !SurfTouchdown.parseSrs("1,0,0,95,0"),
        "A11 SRS field 5: old ESP (4 fields) -> false, 1 -> true, 0 -> false")
}

private fun startWaitTests() {
    println("── START wait ──")
    val until = 10_000L
    chk(StartWait.step(9_999, until, null, null) == StartWait.Step.SEND, "S1 inside the window, stable -> SEND")
    chk(StartWait.step(9_999, until, null, "STOPPED") == StartWait.Step.WAIT, "S2 inside the window, not stable -> WAIT")
    chk(StartWait.step(10_000, until, null, null) == StartWait.Step.TIMEOUT, "S3 timeout checked before sending: stable exactly at 5s -> TIMEOUT, no MSTART")
    chk(StartWait.step(12_000, until, "POSHOLD REQUIRED", null) == StartWait.Step.TIMEOUT, "S4 timeout before other blockers")
    chk(StartWait.step(9_000, until, "POSHOLD REQUIRED", null) == StartWait.Step.BLOCKED, "S5 other blocker inside the window -> BLOCKED (cancel), not SEND")
    chk(StartWait.retryAfterReject(9_000, until, listOf(MissionCheck.STOPPED_NAME)), "S6 FPGA reject only for STOPPED inside the window -> keep waiting")
    chk(!StartWait.retryAfterReject(10_000, until, listOf(MissionCheck.STOPPED_NAME)), "S7 reject after the window -> no retry")
    chk(!StartWait.retryAfterReject(9_000, 0L, listOf(MissionCheck.STOPPED_NAME)), "S8 reject after a cancel (until = 0) -> no retry")
    chk(!StartWait.retryAfterReject(9_000, until, listOf(MissionCheck.STOPPED_NAME, "GPS")), "S9 reject for STOPPED + another check -> no retry")
    // רצף שלם: 20Hz, יציב רק אחרי 6 שנ' ⇒ TIMEOUT ואף SEND
    run { var sent = 0; var timeout = 0; var t = 5_000L; val u = t + 5_000L
        while (t < 12_000L) { when (StartWait.step(t, u, null, if (t < 11_000L) "STOPPED" else null)) {
            StartWait.Step.SEND -> sent++; StartWait.Step.TIMEOUT -> { timeout++; break }; else -> {} }; t += 50 }
        chk(sent == 0 && timeout == 1, "S10 stable only after 6s -> one TIMEOUT, MSTART never sent (sent=$sent)") }
}

// בדיקת מקור: כל מסלול נדרש מבטל את ההמתנה מיד, לפני כל שליחה
private fun cancelPathTests(vmPath: String) {
    println("── START wait cancel paths (source check) ──")
    val src = File(vmPath).readText()
    fun body(sig: String): String {
        val i = src.indexOf(sig); if (i < 0) return ""
        var d = 0; var j = src.indexOf('{', i)
        val start = j
        while (j < src.length) { if (src[j] == '{') d++; if (src[j] == '}') { d--; if (d == 0) break }; j++ }
        return src.substring(start, j)
    }
    fun cancelsFirst(sig: String): Boolean {
        val b = body(sig); val c = b.indexOf("startWaitCancel("); val s = b.indexOf("udp.send")
        return c >= 0 && (s < 0 || c < s)
    }
    for ((name, sig) in listOf("DISARM" to "fun disarmMotors()", "ABORT" to "fun abortFlight()",
                               "STOP" to "fun missionStop()", "LINK LOST" to "private fun onLinkLost()",
                               "unlatch (mode -> ANGLE)" to "private fun unlatch()",
                               "control reset" to "private fun activateOnGround()"))
        chk(cancelsFirst(sig), "C $name: startWaitCancel before any send")
    val sm = body("fun setMode(mode: FlightMode)")
    chk(sm.contains("if (mode != prev) startWaitCancel("), "C mode change: setMode cancels when the mode changes")
    chk(body("private fun startWaitCancel(why: String)").contains("startWaitEnd()"), "C startWaitCancel clears the window (startWaitEnd)")
}

fun main(args: Array<String>) {
    touchdownTests()
    startWaitTests()
    if (args.isNotEmpty()) cancelPathTests(args[0]) else { bad++; println("FAIL no DroneViewModel.kt path") }
    println(if (bad == 0) "ALL OK  ($ok ok, 0 failed)" else "FAILED  ($ok ok, $bad failed)")
    if (bad != 0) kotlin.system.exitProcess(1)
}
