package com.drone.gcs.ui.screens

import androidx.compose.animation.*
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.drone.gcs.model.*
import com.drone.gcs.ui.components.*
import com.drone.gcs.ui.theme.*
import com.drone.gcs.viewmodel.DroneViewModel
import com.drone.gcs.viewmodel.Notice
import kotlin.math.roundToInt

// ═══════════════════════════════════════════════════════════════════════
//   מסך השליטה
//
//   שלוש עמודות שממלאות את כל גובה המסך:
//     שמאל  - בדיקת מערכת למעלה, מקל מצערת/סבסוב למטה (ליד האגודל)
//     מרכז  - מצב ה-FPGA ושלב הטיסה, חימוש/ניתוק, ABORT, טלמטריה
//     ימין  - בורר מצב הטיסה, המבוקש מול מה ש-INAV מבצע, ומקל גובה/גלגול למטה
//
//   כפתור חימוש/ניתוק אחד בלבד. ABORT הוא מפסק חירום נפרד.
//   אין כפתור LAND: הנחיתה היא באחריות INAV.
//
//   ⚠️ בורר המצבים שולח **בקשה** בלבד. INAV מחליט אם להפעיל אותה, ושתי
//      השורות REQUESTED ו-INAV ACTIVE מוצגות בנפרד בכוונה - כדי שבקשה
//      שנדחתה (POSHOLD בלי GPS, למשל) תיראה מיד כמו שהיא.
//
//   שלב הטיסה מגיע מה-ESP32 בשדה PHS ואינו מחושב כאן.
//
//   ⚠️ הבאנר העליון מציג את **רשות השליטה** ולא את מצב הקישור: קישור תקין
//      אינו אומר שמותר לשדר. ראו ControlAuthority.
// ═══════════════════════════════════════════════════════════════════════
@Composable
fun MainControlScreen(vm: DroneViewModel = viewModel()) {
    val control        by vm.control.collectAsStateWithLifecycle()
    val telemetry      by vm.telemetry.collectAsStateWithLifecycle()
    val linkState      by vm.linkState.collectAsStateWithLifecycle()
    val authority      by vm.controlAuthority.collectAsStateWithLifecycle()
    val hoverPercent   by vm.hoverPercent.collectAsStateWithLifecycle()
    val latched        by vm.latched.collectAsStateWithLifecycle()
    val systemStarted  by vm.systemStarted.collectAsStateWithLifecycle()
    val navThrottle    by vm.navThrottle.collectAsStateWithLifecycle()
    val altHoldNeutral by vm.altHoldNeutral.collectAsStateWithLifecycle()
    val notice         by vm.notice.collectAsStateWithLifecycle()
    val setPending     by vm.settingsPending.collectAsStateWithLifecycle()
    val lidarPending   by vm.lidarPending.collectAsStateWithLifecycle()
    val lidarView      by vm.lidarView.collectAsStateWithLifecycle()
    val prepPending    by vm.prepPending.collectAsStateWithLifecycle()
    val startWaiting   by vm.startWaiting.collectAsStateWithLifecycle()
    val modeIssue      by vm.modeIssue.collectAsStateWithLifecycle()
    var showSettings   by remember { mutableStateOf(false) }
    // גובה כרטיס FLIGHT MODE - SYSTEM CHECK מקבל בדיוק אותו גובה (סימטריה)
    var modeCardH      by remember { mutableStateOf(0.dp) }
    val density        = LocalDensity.current

    // המנועים חיים לפי INAV או לפי ה-FPGA ⇒ ניתוק דורש שתי לחיצות
    val motorsLive = telemetry.motorsLive

    val armMode = when {
        motorsLive                              -> ArmButtonMode.DISARM
        telemetry.fpgaState == FpgaState.ABORT  -> ArmButtonMode.CLEAR
        else                                    -> ArmButtonMode.ARM
    }

    // חסימת חימוש אמיתית - בלי GPS, שלא נדרש בטיסה ידנית
    val armBlock = when {
        !systemStarted                           -> "SYSTEM START"
        telemetry.fpgaState != FpgaState.READY   -> "FPGA READY"
        // כל חימוש מתחיל בטיסה ידנית. מצב ניווט מבוקש לפני החימוש היה נכנס
        // לתוקף ברגע שהמנועים מתעוררים, ו-INAV ממילא מסרב לחמש עם מצב ניווט פעיל.
        // WP (משימה נבחרה) מותר: ה-FPGA מוציא אותו ל-CH7 כ-ANGLE על הקרקע.
        control.mode != FlightMode.ANGLE &&
            control.mode != FlightMode.WP        -> "SELECT ANGLE OR WP"
        else                                     -> telemetry.checks.blockingReason()
    }

    // כל כפתור במסך (כולל הדיאלוג) משמיע צליל קצר + רטט דרך LocalTap
    val tap = remember(vm) { { vm.tap() } }
    CompositionLocalProvider(LocalTap provides tap) {
    Box(modifier = Modifier.fillMaxSize().background(HudBackground)) {
        Column(modifier = Modifier.fillMaxSize()) {

            TopStatusBar(
                linkState = linkState,
                authority = authority,
                telemetry = telemetry,
                control   = control
            )

            Row(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
                    .padding(start = 12.dp, end = 12.dp, top = 8.dp, bottom = 12.dp),
                horizontalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                // ── LEFT - בדיקות + מקל מצערת/סבסוב ──
                Column(
                    modifier = Modifier.weight(1f).fillMaxHeight(),
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.SpaceBetween
                ) {
                    // גרסת ה-ESP ואבחון: סבב מרבי · פער מרבי בין שורות שליטה · MSP שנשלחו/נכשלו
                    val t = telemetry
                    // כל ערך בתא נפרד ברוחב קבוע - מספר שגדל לא מזיז את התאים האחרים.
                    // הריפוד בתוך התא מינימלי: רק לטווח הסביר של כל מדידה.
                    fun num(v: Int, w: Int, max: Int): String =
                        if (v < 0) "--".padStart(w) else minOf(v, max).toString().padStart(w)
                    // (הגרסאות עברו לחלון המידע - כפתור i בכרטיס)
                    val diag = if (t.espVersion.isEmpty()) emptyList() else listOf(
                        "LOOP ${num(t.diagLoopMs, 3, 999)}ms",
                        "GAP ${num(t.diagCtrlGapMs, 4, 9999)}ms",
                        "MSP ${num(t.diagMspReq, 2, 99)}/${if (t.diagMspFail < 0) "--" else minOf(t.diagMspFail, 99).toString()}",
                        if (vm.recorderFile != null) "REC" else ""
                    )
                    SystemCheckPanel(
                        checks        = telemetry.checks,
                        magHealth     = telemetry.magHealth,
                        systemStarted = systemStarted,
                        onSystemStart = vm::systemStart,
                        diagCells     = diag,
                        diagWarn      = authority.streaming && (t.diagCtrlGapMs > 150 || t.diagMspFail > 3),
                        espVersion    = t.espVersion,
                        fpgaVersion   = t.fpgaVersion,
                        targetHeight  = if (modeCardH > 0.dp) modeCardH else null
                    )

                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        JoystickLabel("LEFT STICK")
                        Spacer(Modifier.height(6.dp))
                        ThrottleYawJoystick(
                            modifier        = Modifier.fillMaxWidth(0.95f),
                            throttlePercent = control.throttleSent,   // מה שנשלח בפועל, בחצאי אחוז
                            hoverPercent    = hoverPercent,
                            latched         = latched,
                            yawValue        = control.yaw,
                            // המקל מוחזר למרכז כשאין רשות שליטה - לא רק באובדן קשר
                            forceCenter     = !authority.streaming,
                            onStickChange   = { yNorm, yaw ->
                                vm.updateThrottleStick(yNorm)
                                vm.updateYaw(yaw)
                            },
                            onTrimHover     = vm::trimHover,
                            navThrottle     = navThrottle,
                            navNeutral      = altHoldNeutral,
                            navLabel        = if (telemetry.inavModesValid)
                                                  InavMode.label(telemetry.inavModes, true)
                                              else "NAV",
                        )
                    }
                }

                // ── CENTER - מצב, פקודות וטלמטריה ──
                Column(
                    modifier = Modifier.weight(1.25f).fillMaxHeight(),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    FlightStateBar(
                        state     = telemetry.fpgaState,
                        phase     = telemetry.phase,
                        fpgaDebug = telemetry.fpgaDebug
                    )

                    ArmDisarmButton(
                        mode        = armMode,
                        canArm      = armBlock == null,
                        blockReason = armBlock,
                        onArm       = vm::armMotors,
                        onDisarm    = vm::disarmMotors
                    )

                    EmergencyButtons(onAbort = vm::abortFlight)

                    if (MissionConfig.ENABLED) {
                        MissionPanel(
                            mission     = telemetry.mission,
                            selected    = control.mode == FlightMode.WP,
                            prepPending = prepPending,
                            startWaiting = startWaiting,
                            // אותה בדיקה בדיוק שחוסמת ב-ViewModel
                            startBlock  = vm.missionStartBlockedBy(),
                            notice      = notice,
                            onStart     = vm::missionStart,
                            onStop      = vm::missionStop,
                            onSettings  = { showSettings = true }
                        )
                    }

                    TelemetryPanel(
                        telemetry    = telemetry,
                        modifier     = Modifier.weight(1f),
                        lidarView    = lidarView,
                        lidarPending = lidarPending,
                        onLidarMode  = vm::setLidarMode
                    )
                }

                // ── RIGHT - בורר המצב, מבוקש מול בפועל + מקל גובה/גלגול ──
                Column(
                    modifier = Modifier.weight(1f).fillMaxHeight(),
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.SpaceBetween
                ) {
                    ModeStatusCard(
                        requested      = control.mode,
                        telemetry      = telemetry,
                        navThrottle    = navThrottle,
                        hoverPercent   = hoverPercent,
                        altHoldNeutral = altHoldNeutral,
                        onSelect       = vm::setMode,
                        rpLimit        = control.rpLimit,
                        onRpLimit      = vm::setRpLimit,
                        surface        = control.surface,
                        surfaceHalf    = control.throttleHalf,
                        onSurface      = vm::setSurface,
                        modeIssue      = modeIssue,
                        modifier       = Modifier.onSizeChanged {
                            modeCardH = with(density) { it.height.toDp() }
                        }
                    )

                    // אותו מבנה בדיוק כמו בצד השמאלי: כותרת, ריבוע המקל ברוחב 95%
                    // מהעמודה, ואזור תחתית בגובה המשותף. המקל ממורכז בתוך הריבוע,
                    // ולכן מרכז אזור התנועה שלו נמצא על אותו Y כמו מרכז המקל השמאלי.
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        JoystickLabel("RIGHT STICK")
                        Spacer(Modifier.height(6.dp))
                        Box(
                            modifier         = Modifier.fillMaxWidth(0.95f).aspectRatio(1f),
                            contentAlignment = Alignment.Center
                        ) {
                            VirtualJoystick(
                                modifier    = Modifier.fillMaxWidth(),
                                xLabel      = "RLL",
                                yLabel      = "PCH",
                                externalX   = control.roll,
                                externalY   = control.pitch,
                                xAutoCenter = true,
                                yAutoCenter = true,
                                yRange      = -100f..100f,
                                onXChange   = vm::updateRoll,
                                onYChange   = vm::updatePitch,
                            )
                        }
                        // אזור תחתית ריק בגובה זהה לזה של המקל השמאלי
                        Box(
                            Modifier
                                .padding(top = STICK_FOOTER_GAP)
                                .height(STICK_FOOTER_HEIGHT)
                        )
                    }
                }
            }
        }

        AnimatedVisibility(
            visible  = !authority.streaming,
            enter    = fadeIn() + slideInVertically(),
            exit     = fadeOut() + slideOutVertically(),
            modifier = Modifier.align(Alignment.TopCenter).padding(top = 48.dp)
        ) {
            ControlStatusBanner(
                authority = authority,
                telemetry = telemetry,
                onResume  = vm::resumeControl
            )
        }

        // הגדרות המשימה - שכבה מעל כל המסך (אחרונה ב-Box = מעל הכל)
        if (showSettings) {
            MissionSettingsDialog(
                current     = telemetry.mission.settings,
                confirmed   = telemetry.mission.settingsConfirmed,
                blockReason = vm.settingsBlockedBy(),
                fwdMaxCm    = telemetry.fwdMaxCm,
                pending     = setPending,
                onSave      = vm::saveMissionSettings,
                onDismiss   = { showSettings = false }
            )
        }
    }
    }   // CompositionLocalProvider
}

// ── מצב טיסה: מה ביקשנו מול מה ש-INAV מבצע ─────────────────────────
//
//   שתי צלעות נפרדות בכוונה. הכפתור מסמן את ה**בקשה**; הצבע מספר אם INAV
//   באמת מבצע אותה: ירוק = מבוקש ומבוצע · ענבר = מבוקש וטרם אושר.
//   שורת INAV ACTIVE ממשיכה להיות המקור היחיד למה שקורה בפועל.

@Composable
private fun ModeStatusCard(
    requested: FlightMode,
    telemetry: TelemetryData,
    navThrottle: Boolean,
    hoverPercent: Float,
    altHoldNeutral: Float,
    onSelect: (FlightMode) -> Unit,
    rpLimit: Boolean,
    onRpLimit: (Boolean) -> Unit,
    surface: Boolean,
    surfaceHalf: Int,
    onSurface: (Boolean) -> Unit,
    modeIssue: String?,
    modifier: Modifier = Modifier,
) {
    // מה ש-INAV מדווח, מתורגם בחזרה למצב מוכר. ACRO / FAILSAFE / "לא ידוע"
    // אינם אף אחד מהמצבים שאנחנו מבקשים, ולכן הם משאירים את הבקשה "טרם אושרה".
    val activeLabel = InavMode.label(telemetry.inavModes, telemetry.inavModesValid)
    val activeMode  = FlightMode.entries.firstOrNull { it.label == activeLabel }

    HudCard(title = "FLIGHT MODE", modifier = modifier) {
        Row(
            modifier = Modifier.fillMaxWidth().padding(bottom = 6.dp),
            horizontalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            FlightMode.entries.forEach { m ->
                ModeChip(
                    mode     = m,
                    selected = m == requested,
                    active   = m == activeMode,
                    enabled  = m.blockedBy(telemetry) == null,
                    onSelect = onSelect
                )
            }
        }

        // הגבלת Roll/Pitch ל-75% בטיסה ידנית (ANGLE / ALTHOLD). חלה מיד, נשמרת בטאבלט.
        Row(
            modifier = Modifier.fillMaxWidth().padding(bottom = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Text("R/P LIMIT 75%", fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                color = HudTextSecondary, maxLines = 1, softWrap = false, modifier = Modifier.weight(1f))
            ToggleChip("ON", rpLimit, HudGreen) { onRpLimit(true) }
            ToggleChip("OFF", !rpLimit, HudAmber) { onRpLimit(false) }
        }

        // ═══ E1022A: SURFACE ידני - רק כש-POSHOLD נבחר ═══
        //   ירוק = INAV מדווח SURFACE בפועל · ענבר = מבוקש וטרם פעיל · אדום = נחסם (הסיבה מימין)
        val surfActive = telemetry.inavModesValid && (telemetry.inavModes and InavMode.SURFACE) != 0
        val surfAllowed = requested == FlightMode.POSHOLD
        Row(
            modifier = Modifier.fillMaxWidth().padding(bottom = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Text("SURFACE", fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                color = if (surfAllowed) HudTextSecondary else HudTextDim,
                maxLines = 1, softWrap = false, modifier = Modifier.weight(1f))
            ToggleChip("ON", surface, if (surfActive) HudGreen else HudAmber) { if (surfAllowed) onSurface(true) }
            ToggleChip("OFF", !surface, HudTextSecondary) { onSurface(false) }
        }
        if (surface || surfActive || telemetry.surfState >= SurfaceState.LOST) {
            TelRow("SURFACE", when {
                surfActive -> "ON  TGT %dcm  DWN %dcm".format(SurfaceMap.targetCm(surfaceHalf), telemetry.lidarDwn)
                else       -> SurfaceState.label(telemetry.surfState)
            })
        }

        TelRow("REQUESTED", requested.label + when {
            activeMode == requested       -> ""
            requested == FlightMode.WP    -> "  (MISSION SELECTED)"
            else                          -> "  (PENDING)"
        })
        TelRow("INAV ACTIVE", activeLabel)

        // מצב שביקשנו ו-INAV לא מפעיל יותר משנייה - בולט, עם הסיבה
        modeIssue?.let {
            Text(it, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                fontSize = HudText.caption, color = HudRed, maxLines = 1, softWrap = false,
                overflow = TextOverflow.Ellipsis,
                modifier = Modifier.fillMaxWidth()
                    .background(HudRed.copy(alpha = 0.15f), RoundedCornerShape(4.dp))
                    .padding(horizontal = 6.dp, vertical = 3.dp))
        }

        // מה חוסם כרגע - כדי שכפתור כבוי לא יהיה חידה
        val gate = FlightMode.ALTHOLD.blockedBy(telemetry)?.let { "ALTHOLD NEEDS $it" }
            ?: FlightMode.POSHOLD.blockedBy(telemetry)?.let { "NAV NEEDS $it" }
            ?: "ALL MODES OPEN"
        TelRow("MODE GATE", gate)

        TelRow("ALT THR CFG", AltHoldThrottle.label(telemetry.altHoldThrMode))

        // המרכז שאליו המצערת חוזרת בשחרור המקל - לפי המצב שבו אנחנו באמת
        val centerText = when {
            navThrottle && surfActive -> "%.1f%%  SURF %dcm".format(altHoldNeutral, SurfaceMap.targetCm((altHoldNeutral * 2f).toInt()))
            navThrottle -> "%.1f%%  NAV".format(altHoldNeutral)
            telemetry.phase == FlightPhase.AIRBORNE -> "%.1f%%  HOVER".format(hoverPercent)
            telemetry.phase == FlightPhase.TAKEOFF_CONFIRM -> "HOLD  TAKEOFF"
            else -> "0%  GROUND"
        }
        TelRow("THR CENTER", centerText)
    }
}

@Composable
private fun RowScope.ModeChip(
    mode: FlightMode,
    selected: Boolean,
    active: Boolean,
    enabled: Boolean,
    onSelect: (FlightMode) -> Unit,
) {
    val tap = LocalTap.current
    val accent = when {
        selected && active -> HudGreen           // מבוקש, ו-INAV אכן מבצע
        selected           -> HudAmber           // מבוקש, טרם אושר
        enabled            -> HudTextSecondary
        else               -> HudTextDim         // התנאים לא מתקיימים כרגע
    }

    Box(
        modifier = Modifier
            .weight(1f)
            .height(34.dp)
            .clip(RoundedCornerShape(4.dp))
            .background(if (selected) accent.copy(alpha = 0.20f) else Color.Transparent)
            .border(1.dp, if (selected) accent else HudBorder, RoundedCornerShape(4.dp))
            .clickable(enabled = enabled) { tap(); onSelect(mode) },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = mode.label,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.caption,
            fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
            color = accent,
            maxLines = 1,
            softWrap = false
        )
    }
}

// ── Top status bar ─────────────────────────────────────────────────
//   שני מדדים נפרדים, ובכוונה: LINK הוא מצב הקישור, CTRL הוא האם אנחנו
//   באמת משדרים שליטה. קישור תקין בלי רשות שליטה הוא מצב תקף לחלוטין.

@Composable
private fun TopStatusBar(
    linkState: LinkState,
    authority: ControlAuthority,
    telemetry: TelemetryData,
    control: ControlValues,
) {
    val linkColor = when (linkState) {
        LinkState.CONNECTED  -> HudGreen
        LinkState.CONNECTING -> HudAmber
        LinkState.LOST       -> HudRed
    }
    val linkText = when (linkState) {
        LinkState.CONNECTED  -> "LINK OK"
        LinkState.CONNECTING -> "CONNECTING"
        LinkState.LOST       -> "LINK LOST"
    }
    val ctrlColor = authorityColor(authority)

    // פריסה בשלוש קבוצות, כל אחת עם מקום קבוע (נמדד בגופן עצמו לטקסט הארוך ביותר):
    //   שמאל  - DRONE GCS · LINK · CTRL
    //   מרכז  - REQ / INAV, ממורכז בדיוק במרכז המסך (לא תלוי ברוחב הקבוצות האחרות)
    //   ימין  - THR / YAW / PCH / RLL · סוללה (צמודה לימין)
    //   הרווח DRONE GCS→LINK שווה לרווח הנראה בין הזוויות לסוללה.
    //   כל ציר בתא משלו: המספר צמוד לשם הציר, והרווח הוא בין הצירים.
    //   כך החלפת מצב טיסה, זווית שגדלה בספרה או סוללה לא מזיזים שום דבר.
    val measurer  = rememberTextMeasurer()
    val density   = LocalDensity.current
    val mainStyle = TextStyle(fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel, fontWeight = FontWeight.Bold)
    val axisStyle = TextStyle(fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel)
    fun widthOf(text: String, style: TextStyle) =
        with(density) { (measurer.measure(text, style).size.width + 2).toDp() }
    val linkW = remember(measurer) { widthOf("CONNECTING", mainStyle) }
    val ctrlW = remember(measurer) { widthOf("CTRL OFF", mainStyle) }
    val reqW  = remember(measurer) { widthOf("REQ: ALTHOLD · INAV: FAILSAFE", axisStyle) }
    val thrW  = remember(measurer) { widthOf("THR:100.0", axisStyle) + AXIS_LABEL_GAP }
    val axisW = remember(measurer) { widthOf("YAW:-100", axisStyle) + AXIS_LABEL_GAP }
    val battW = remember(measurer) { widthOf("12.60V | 100%", mainStyle) }

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .background(HudSurface)
            .border(width = 1.dp, color = HudBorder)
            .padding(horizontal = 16.dp, vertical = 6.dp)
    ) {
        // ── שמאל ──
        Row(modifier = Modifier.align(Alignment.CenterStart), verticalAlignment = Alignment.CenterVertically) {
            Text("DRONE GCS", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                fontSize = HudText.rowLabel, letterSpacing = 3.sp, color = HudCyan,
                maxLines = 1, softWrap = false)
            Spacer(Modifier.width(SIDE_GAP))
            Text(linkText, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                fontWeight = FontWeight.Bold, color = linkColor, maxLines = 1, softWrap = false,
                modifier = Modifier.width(linkW))
            Spacer(Modifier.width(LINK_CTRL_GAP))
            Text(if (authority.streaming) "CTRL TX" else "CTRL OFF",
                fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                fontWeight = FontWeight.Bold, color = ctrlColor, maxLines = 1, softWrap = false,
                modifier = Modifier.width(ctrlW))
        }

        // ── מרכז: מה ביקשנו, ומה INAV מבצע בפועל - בדיוק במרכז המסך ──
        Text("REQ: ${control.mode.label} · INAV: " +
                InavMode.label(telemetry.inavModes, telemetry.inavModesValid),
            fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
            color = HudTextSecondary, maxLines = 1, softWrap = false,
            textAlign = TextAlign.Center,
            modifier = Modifier.align(Alignment.Center).width(reqW))

        // ── ימין ──
        Row(modifier = Modifier.align(Alignment.CenterEnd), verticalAlignment = Alignment.CenterVertically) {
            AxisCell("THR:", "%.1f".format(control.throttleSent), thrW)
            Spacer(Modifier.width(AXIS_GAP))
            AxisCell("YAW:", control.yaw.toInt().toString(), axisW)
            Spacer(Modifier.width(AXIS_GAP))
            AxisCell("PCH:", control.pitch.toInt().toString(), axisW)
            Spacer(Modifier.width(AXIS_GAP))
            AxisCell("RLL:", control.roll.toInt().toString(), axisW)
            Spacer(Modifier.width(AXES_BATT_GAP))

            // Battery - באותו גודל כמו DRONE GCS, צמודה לימין התא שלה
            Text(
                text = if (telemetry.batteryVoltCv > 0)
                    "%.2fV | %d%%".format(telemetry.batteryVoltCv / 100f, telemetry.batteryPercent)
                else "--.--V | --%",
                fontFamily = FontFamily.Monospace,
                fontSize = HudText.rowLabel,
                fontWeight = FontWeight.Bold,
                color = when (telemetry.batteryState) {
                    0    -> HudGreen        // OK
                    1    -> HudAmber        // WARNING
                    2    -> HudRed          // CRITICAL
                    else -> HudTextDim      // NOT PRESENT
                },
                maxLines = 1,
                softWrap = false,
                textAlign = TextAlign.End,
                modifier = Modifier.width(battW)
            )
        }
    }
}

// מרווחי השורה העליונה
private val SIDE_GAP       = 60.dp   // DRONE GCS → LINK
private val LINK_CTRL_GAP  = 38.dp   // סוף תא LINK → CTRL (רווח נראה ~65dp, חצי מקודם)
private val AXIS_GAP       = 14.dp   // בין תאי הצירים
private val AXIS_LABEL_GAP = 3.dp    // בין שם הציר למספר שלו
private val AXES_BATT_GAP  = 24.dp   // סוף תא RLL → תא הסוללה (רווח נראה ~60dp כשהערכים קטנים)

// ציר אחד: שם + מספר צמוד, בתא ברוחב קבוע
@Composable
private fun AxisCell(label: String, value: String, width: Dp) {
    Row(modifier = Modifier.width(width), verticalAlignment = Alignment.CenterVertically) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
            color = HudTextSecondary, maxLines = 1, softWrap = false)
        Spacer(Modifier.width(AXIS_LABEL_GAP))
        Text(value, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
            color = HudTextSecondary, maxLines = 1, softWrap = false)
    }
}

private fun authorityColor(a: ControlAuthority): Color = when (a) {
    ControlAuthority.ACTIVE    -> HudGreen
    ControlAuthority.WAITING   -> HudAmber
    ControlAuthority.SUSPENDED -> HudAmber
    ControlAuthority.LOST      -> HudRed
}

// ── באנר רשות השליטה ───────────────────────────────────────────────
//   מוצג בכל מצב שבו האפליקציה **אינה** משדרת שורות שליטה, ומתאר בדיוק
//   את מה שקורה בפועל. בשני מצבים הוא רק מידע, ובאחד - SUSPENDED - הוא
//   גם נקודת ההחלטה: חזרה לשליטה על רחפן שכבר מחומש היא פעולה מפורשת של
//   הטייס, ולא משהו שקורה מעצמו ברגע שהטלמטריה חוזרת.
//
//   ⚠️ החזרה מתחילה מהמצערת הידועה שה-ESP מדווח, ולא מאפס. בלי ערך ידוע
//      אין חזרה בכלל: אז הפעולות היחידות הן ניתוק וחירום.
//      גם מצב הטיסה המבוקש נטען ממה ש-INAV מבצע, כדי שהחזרה לא תבטל מצב
//      ניווט פעיל.

@Composable
private fun ControlStatusBanner(
    authority: ControlAuthority,
    telemetry: TelemetryData,
    onResume: () -> Unit,
) {
    val tap = LocalTap.current
    val accent           = authorityColor(authority)
    val knownThrottlePct = telemetry.lastThrottlePct
    // אותה בדיקה בדיוק שחוסמת ב-ViewModel - כדי שהכפתור לא יציע פעולה שתידחה
    val resumeBlock      = telemetry.resumeBlockedBy()
    val canResume        = authority == ControlAuthority.SUSPENDED && resumeBlock == null
    var resumeArmed by rememberPendingConfirm(active = canResume)

    Column(
        modifier = Modifier
            .clip(RoundedCornerShape(8.dp))
            .background(accent.copy(alpha = 0.18f))
            .border(2.dp, accent, RoundedCornerShape(8.dp))
            .padding(horizontal = 24.dp, vertical = 10.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Text(authority.label,
            fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
            fontSize = HudText.sensorName, letterSpacing = 1.sp, color = accent,
            maxLines = 1, softWrap = false)

        Text(authority.hint,
            fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
            letterSpacing = 1.sp, color = HudTextSecondary,
            maxLines = 1, softWrap = false)

        if (authority == ControlAuthority.SUSPENDED) {
            if (canResume) {
                Button(
                    onClick = {
                        tap()
                        if (resumeArmed) { onResume(); resumeArmed = false }
                        else             { resumeArmed = true }
                    },
                    shape  = RoundedCornerShape(6.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (resumeArmed) accent else accent.copy(alpha = 0.35f),
                        contentColor   = Color.White
                    ),
                    modifier = Modifier.height(40.dp)
                ) {
                    Text(
                        text = if (resumeArmed) "CONFIRM RESUME AT ${"%.1f".format(knownThrottlePct)}%"
                               else             "RESUME CONTROL AT ${"%.1f".format(knownThrottlePct)}%",
                        fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                        fontSize = HudText.caption, letterSpacing = 1.sp,
                        maxLines = 1, softWrap = false
                    )
                }
            } else {
                // הסיבה המדויקת ולא הודעה כללית: זה מה שצריך להשתנות כדי
                // שחזרה לשליטה תהיה אפשרית בכלל
                Text("RESUME BLOCKED - ${resumeBlock ?: "---"}",
                    fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                    fontSize = HudText.caption, letterSpacing = 1.sp, color = HudRed,
                    maxLines = 1, softWrap = false)

                Text("DISARM OR ABORT ONLY",
                    fontFamily = FontFamily.Monospace,
                    fontSize = HudText.caption, letterSpacing = 1.sp, color = HudTextSecondary,
                    maxLines = 1, softWrap = false)
            }
        }
    }
}

// ── Mission FSM ─────────────────────────────────────────────────────
//   האפליקציה רק מבקשת: ה-FPGA מחליט, ומה שמוצג כאן הוא מה שהוא מדווח בתשובת
//   MST - לא מה שביקשנו. מצבים נפרדים: לא נבחרה · נבחרה (ולמה לא מוכנה) ·
//   MISSION READY · התחילה · נעצרה למכשול · הושלמה · נעצרה (ABORT / STOP).
//   START בשתי לחיצות, רק אחרי ARM. STOP בלחיצה אחת, ופתוח תמיד.
//   גלגל השיניים פותח את הגדרות העקיפה.

@Composable
private fun MissionPanel(
    mission: MissionStatus,
    selected: Boolean,
    prepPending: Boolean,
    startWaiting: Boolean,
    startBlock: String?,
    notice: Notice?,
    onStart: () -> Unit,
    onStop: () -> Unit,
    onSettings: () -> Unit,
) {
    var startArmed by rememberPendingConfirm(active = startBlock == null)
    val phase = mission.phase(selected)

    val phaseColor = when (phase) {
        MissionStatus.Phase.NONE      -> HudTextDim
        MissionStatus.Phase.SELECTED  -> HudAmber
        MissionStatus.Phase.READY     -> HudGreen
        MissionStatus.Phase.STARTED   -> HudCyan
        MissionStatus.Phase.OBSTACLE  -> HudAmber
        MissionStatus.Phase.COMPLETED -> HudGreen
        MissionStatus.Phase.ABORTED   -> HudRed
    }
    // שורה ראשית: המצב, ומה חסר כשנבחרה ועדיין לא מוכנה
    val phaseText = when {
        prepPending && !mission.active          -> "PREPARING..."
        phase == MissionStatus.Phase.SELECTED   -> "NOT READY: " + (mission.prepBlockedBy() ?: "---")
        phase == MissionStatus.Phase.ABORTED    -> "STOPPED: " + MissionReason.label(mission.reason)
        else                                    -> phase.label
    }
    val stateText = when {
        !mission.present    -> "NO ESP"
        !mission.fresh      -> "NO FPGA REPLY"
        mission.reason != 0 && !mission.active -> mission.state.label + " · " + MissionReason.label(mission.reason)
        else                -> mission.state.label
    }
    // 25.10: STOPPED לא חוסם את הכפתור - START מחכה לרגע יציב (עד 5 שנ')
    val stoppedNow = (mission.checks and MissionCheck.STOPPED) != 0
    val startText = when {
        mission.active     -> "RUNNING"
        startWaiting       -> "WAITING FOR STABLE HOVER..."
        startBlock == null -> if (stoppedNow) "READY - PRESS TWICE" else "READY - PRESS TWICE (WAITS FOR STABLE)"
        else               -> startBlock
    }
    val s = mission.settings

    HudCard(title = "MISSION") {
        Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(phaseText, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                fontSize = HudText.rowLabel, color = phaseColor, maxLines = 1, softWrap = false,
                modifier = Modifier.weight(1f))
            // מורם מעט כדי שמרכזו יתיישר עם מרכז שורת המצב (offset לא משנה את הפריסה)
            Box(Modifier.offset(y = (-7).dp)) { GearButton(enabled = true, onClick = onSettings) }
        }
        MissionRow("FSM", stateText, HudTextPrimary)
        MissionRow("START", startText, if (startBlock == null || mission.active) HudTextPrimary else HudAmber)
        MissionRow("WP MISSION",
                   MissionVerify.label(mission.verify) + " · ${mission.wpCount} WP · ALT ${cmOrDash(mission.wp1AltCm.takeIf { it > 0 } ?: -1)}" +
                       " · ${if (mission.effSpeedCms >= 0) "${mission.effSpeedCms} cm/s" else "-- cm/s"}",
                   if (mission.verify == MissionVerify.OK) HudGreen else HudAmber)
        // קצר בכוונה - הפירוט המלא בגלגל השיניים. כתום רק כשה-FPGA עוד לא אישר.
        MissionRow("SETTINGS",
                   when {
                       s == null                  -> "NOT REPORTED"
                       !mission.settingsConfirmed -> "STAGE ${s.stage} · WAITING FOR FPGA"
                       else                       -> "STAGE ${s.stage} · DET ${s.stopCm} · CLR ${s.clearCm}" +
                                                     (if (s.takeoff) " · AUTO TO" else "")
                   },
                   if (s != null && mission.settingsConfirmed) HudTextPrimary else HudAmber)
        MissionRow("OBSTACLE", "DET ${cmOrDash(mission.detCm)} · STOP ${cmOrDash(mission.stopCm)} · WP ${mission.savedWp}",
                   HudTextPrimary)
        val b = mission.brake
        if (mission.detCm >= 0) {
            MissionRow("BRAKE", "${b.speedCms} cm/s · ACK ${if (b.ackMs < 0) "--" else "${b.ackMs}ms"} · " +
                       "STOP ${b.stopMs}ms · ${b.travelCm} cm · BACK ${b.reboundCm} cm", HudTextPrimary)
        }
        notice?.let { n ->
            MissionRow("LAST", n.text, if (n.ok) HudGreen else HudRed)
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            MissionButton(
                text    = if (startArmed) "CONFIRM START" else "START",
                color   = HudGreen,
                enabled = startBlock == null && !startWaiting,
                armed   = startArmed,
                onClick = {
                    if (startArmed) { onStart(); startArmed = false }
                    else            { startArmed = true }
                }
            )
            // STOP אדום ופעיל רק כשיש משימה פעילה. כשאין תשובה מה-FPGA לא יודעים אם יש
            // משימה - אז הוא נשאר לחיץ (בטיחות), אבל לא אדום.
            val stopEnabled = mission.active || !mission.fresh
            MissionButton(text = "STOP", color = if (mission.active) HudRed else HudTextSecondary,
                          enabled = stopEnabled, armed = mission.active, onClick = onStop)
        }
    }
}

private fun cmOrDash(cm: Int): String = if (cm < 0) "--" else "$cm cm"

@Composable
private fun MissionRow(label: String, value: String, color: Color) {
    // הערך לא נדבק לתווית, ואם הוא ארוך מדי - נחתך בשלוש נקודות ולא גולש
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(10.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
            color = HudTextSecondary, maxLines = 1, softWrap = false)
        Text(value, fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
            color = color, fontWeight = FontWeight.Bold, maxLines = 1, softWrap = false,
            overflow = TextOverflow.Ellipsis, textAlign = TextAlign.End, modifier = Modifier.weight(1f))
    }
}

@Composable
private fun RowScope.MissionButton(
    text: String,
    color: Color,
    enabled: Boolean,
    armed: Boolean,
    onClick: () -> Unit,
) {
    val tap = LocalTap.current
    val c = if (enabled) color else HudTextDim
    Box(
        modifier = Modifier
            .weight(1f)
            .height(38.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(c.copy(alpha = if (armed && enabled) 0.35f else 0.12f))
            .border(1.dp, c, RoundedCornerShape(6.dp))
            .clickable(enabled = enabled) { tap(); onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(text, fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
            fontWeight = FontWeight.Bold, color = c, maxLines = 1, softWrap = false)
    }
}

// גודל אחיד מ-HudText, כמו כל שאר המסך
@Composable
private fun JoystickLabel(text: String) {
    Text(text, fontFamily = FontFamily.Monospace, fontSize = HudText.stickLabel,
        letterSpacing = 2.sp, color = HudTextDim, maxLines = 1, softWrap = false)
}
