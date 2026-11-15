package com.drone.gcs.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.graphics.Color
import com.drone.gcs.model.CheckState
import com.drone.gcs.model.MagHealth
import com.drone.gcs.model.SystemChecks
import com.drone.gcs.ui.theme.*

// ═══════════════════════════════════════════════════════════════════════
//   בדיקת מערכת
//
//   SYSTEM START  - פותח READY ב-FPGA (CMD:SELF_CHECK)
//   SYSTEM READY  - מוצג רק כשה-FPGA ענה וכל בדיקות החובה עברו באמת
//   CHECKS FAILED - מה חוסם מוצג במפורש, בלי להסתיר כלום
//
//   GPS מוצג אבל אינו חוסם: בטיסה ידנית (ANGLE) אין בו צורך.
//   COMPASS (שדה MGH הקיים) - מידע בלבד, לא חוסם SYSTEM READY. רק התווית שונתה.
//   החסימה למצבי NAV (POSHOLD/RTH/WP) נשארת ב-FlightMode.blockedBy() (COMPASS).
//   ⚠️ בלי תווים גרפיים בכפתורים - בגופן של הטאבלט חלקם הופיעו כריבוע.
// ═══════════════════════════════════════════════════════════════════════
// רוחב יחסי לכל תא בשורת האבחון: LOOP · GAP · MSP · REC
//   ⚠️ התאים ברוחב קבוע - מספר שגדל בספרה לא מזיז את התאים שאחריו.
//      הגרסאות עברו לחלון המידע (כפתור i).
private val DIAG_CELL_WEIGHTS = listOf(1.1f, 1.2f, 1.0f, 0.5f)

@Composable
fun SystemCheckPanel(
    checks: SystemChecks,
    magHealth: Int,
    systemStarted: Boolean,
    onSystemStart: () -> Unit,
    modifier: Modifier = Modifier,
    diagCells: List<String> = emptyList(),   // אבחון התקשורת - תאים ברוחב קבוע
    diagWarn: Boolean = false,
    espVersion: String = "",                 // FWV
    fpgaVersion: String = "",                // FPV
    targetHeight: Dp? = null,                // גובה כרטיס FLIGHT MODE - לסימטריה. null = לפי התוכן
) {
    val tap = LocalTap.current
    var showInfo by remember { mutableStateOf(false) }
    val blocking = checks.blockingReason()
    val ready    = systemStarted && blocking == null

    val btnColor = when {
        !systemStarted -> HudCyan
        ready          -> HudGreen
        else           -> HudAmber
    }
    val btnText = when {
        !systemStarted -> "SYSTEM START"
        ready          -> "SYSTEM READY"
        else           -> "CHECKS FAILED"
    }

    // הכרטיס והחלון באותו Box: חלון המידע נפתח בדיוק מעל SYSTEM CHECK, לא באמצע המסך.
    // כשגובה FLIGHT MODE ידוע, הכרטיס מקבל בדיוק אותו גובה, והמרווחים מתחלקים:
    // כותרת → רכיבים, רכיבים → שורת האבחון (שווים), ומעט פחות עד הכפתור.
    val fixed = targetHeight != null
    Box(modifier = if (fixed) modifier.height(targetHeight!!) else modifier) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .then(if (fixed) Modifier.fillMaxHeight() else Modifier)
            .clip(RoundedCornerShape(6.dp))
            .background(HudCard)
            .border(1.dp, HudBorder, RoundedCornerShape(6.dp))
            .padding(horizontal = 8.dp, vertical = 8.dp)
    ) {
        Text(
            text = "SYSTEM CHECK",
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.cardTitle,
            fontWeight = FontWeight.Bold,
            color = HudCyan,
            letterSpacing = 2.sp,
            maxLines = 1,
            softWrap = false
        )

        FlexGap(fixed, 1f, 14.dp)

        // ⚠️ גופן כמו REQUESTED / INAV ACTIVE ב-FLIGHT MODE. השורה הארוכה ביותר
        //    (COMPASS · NOT VERIFIED) נמדדה על הטאבלט - נכנסת עם מרווח.
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(11.dp)
            ) {
                CheckRow("FPGA LINK", checks.fpga)
                CheckRow("FC LINK",   checks.fcLink)
                CheckRow("IMU / ATT", checks.imu)
                CheckRow("BATTERY",   checks.battery)
            }
            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(11.dp)
            ) {
                CheckRow("LIDAR FRONT", checks.lidarFwd)
                CheckRow("LIDAR DOWN",  checks.lidarDwn)
                // מידע בלבד - לא נכנס ל-blockingReason
                MagnetometerRow(magHealth)
                // מידע בלבד - לא תנאי לחימוש בטיסה ידנית
                CheckRow("GPS", checks.gps, optional = true)
            }
        }

        // ⚠️ כל ערך בתא משלו, ברוחב קבוע: מספר שגדל בספרה לא מזיז את שאר השורה
        // ⚠️ השורה תמיד תופסת את מקומה - גם לפני שה-ESP מחובר (ריקה).
        //    כך החיבור רק ממלא אותה, והרכיבים שמעליה לא קופצים.
        FlexGap(fixed, 1f, 14.dp)
        Row(modifier = Modifier.fillMaxWidth()) {
            diagCells.ifEmpty { List(DIAG_CELL_WEIGHTS.size) { "" } }.forEachIndexed { i, cell ->
                    Text(cell.ifEmpty { " " }, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                        color = if (diagWarn) HudAmber else HudTextDim, maxLines = 1, softWrap = false,
                        overflow = TextOverflow.Clip,
                        modifier = Modifier.weight(DIAG_CELL_WEIGHTS.getOrElse(i) { 1f }))
            }
        }

        FlexGap(fixed, 1f, 14.dp)

        Button(
            onClick = { tap(); onSystemStart() },
            shape   = RoundedCornerShape(6.dp),
            colors  = ButtonDefaults.buttonColors(
                containerColor = btnColor.copy(alpha = 0.18f),
                contentColor   = btnColor
            ),
            modifier = Modifier
                .fillMaxWidth()
                .height(40.dp)
                .border(1.dp, btnColor.copy(alpha = 0.6f), RoundedCornerShape(6.dp))
        ) {
            // צבע מ-LocalContentColor, לא מסגנון ברירת המחדל של הכפתור
            Text(
                text = btnText,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = HudText.rowLabel,
                letterSpacing = 1.sp,
                color = LocalContentColor.current,
                maxLines = 1,
                softWrap = false
            )
        }

        if (systemStarted && blocking != null) {
            Spacer(Modifier.height(4.dp))
            Text(
                text = "BLOCKED BY: $blocking",
                fontFamily = FontFamily.Monospace,
                fontSize = HudText.caption,
                letterSpacing = 1.sp,
                color = HudAmber,
                maxLines = 1,
                softWrap = false,
                modifier = Modifier.fillMaxWidth()
            )
        }
    }

        // כפתור המידע - בפינה העליונה של הכרטיס, בשורת הכותרת
        InfoButton(
            open = showInfo,
            modifier = Modifier.align(Alignment.TopEnd).padding(top = 5.dp, end = 5.dp)
        ) { tap(); showInfo = !showInfo }

        if (showInfo) {
            VersionsOverlay(
                espVersion  = espVersion,
                fpgaVersion = fpgaVersion,
                modifier    = Modifier.matchParentSize().padding(4.dp),
                onClose     = { tap(); showInfo = false }
            )
        }
    }   // Box
}

// מרווח: בגובה קבוע - חלק יחסי מהמקום הפנוי; בלי גובה קבוע - מרווח קבוע
@Composable
private fun ColumnScope.FlexGap(fixed: Boolean, weight: Float, min: Dp) {
    if (fixed) Spacer(Modifier.weight(weight)) else Spacer(Modifier.height(min))
}

// ── כפתור i ──────────────────────────────────────────────────────────
@Composable
private fun InfoButton(open: Boolean, modifier: Modifier = Modifier, onClick: () -> Unit) {
    val c = if (open) HudCyan else HudTextSecondary
    Box(
        modifier = modifier
            .size(24.dp)
            .clip(RoundedCornerShape(50))
            .background(c.copy(alpha = 0.15f))
            .border(1.5.dp, c, RoundedCornerShape(50))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center
    ) {
        Text("i", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
            fontSize = HudText.rowLabel, color = c, maxLines = 1, softWrap = false)
    }
}

// ── חלון הגרסאות - מכסה את SYSTEM CHECK בלבד ──────────────────────────
//   כתב לבן וגדול כמו בהגדרות המשימה. "NOT REPORTED" = הרכיב לא דיווח גרסה
//   (קוד ישן, או שאין קשר).
@Composable
private fun VersionsOverlay(
    espVersion: String,
    fpgaVersion: String,
    modifier: Modifier = Modifier,
    onClose: () -> Unit,
) {
    val ctx = LocalContext.current
    val appVersion = remember(ctx) {
        runCatching { ctx.packageManager.getPackageInfo(ctx.packageName, 0).versionName }.getOrNull() ?: "-"
    }
    fun shown(v: String) = if (v.isEmpty() || v == "-") "NOT REPORTED" else v

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(HudSurface)
            .border(2.dp, HudCyan, RoundedCornerShape(8.dp))
            // נגיעה בחלון לא עוברת לכפתורים שמתחתיו
            .clickable(interactionSource = remember { MutableInteractionSource() }, indication = null) { }
            .padding(horizontal = 14.dp, vertical = 10.dp)
    ) {
        Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween) {
            Text("VERSIONS", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                fontSize = HudText.sensorName, letterSpacing = 2.sp, color = HudCyan,
                maxLines = 1, softWrap = false)
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(HudTextSecondary.copy(alpha = 0.15f))
                    .border(1.dp, HudTextSecondary, RoundedCornerShape(6.dp))
                    .clickable(onClick = onClose)
                    .padding(horizontal = 12.dp, vertical = 4.dp)
            ) {
                Text("CLOSE", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                    fontSize = HudText.caption, color = HudTextPrimary, maxLines = 1, softWrap = false)
            }
        }
        // השורות במרכז החלון בגובה, בגופן גדול
        Column(
            modifier = Modifier.weight(1f).fillMaxWidth().padding(horizontal = 10.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp, Alignment.CenterVertically)
        ) {
            VersionRow("APP (TABLET)", appVersion)
            VersionRow("ESP32", shown(espVersion))
            VersionRow("FPGA", shown(fpgaVersion))
        }
    }
}

@Composable
private fun VersionRow(label: String, value: String) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = 19.sp,
            color = HudTextSecondary, maxLines = 1, softWrap = false, modifier = Modifier.weight(1f))
        Text(value, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
            fontSize = 22.sp,
            color = if (value == "NOT REPORTED") HudAmber else HudTextPrimary,
            maxLines = 1, softWrap = false)
    }
}

@Composable
private fun CheckRow(label: String, state: CheckState, optional: Boolean = false) {
    val color = when (state) {
        CheckState.OK      -> HudGreen
        CheckState.FAIL    -> if (optional) HudAmber else HudRed
        CheckState.UNKNOWN -> HudTextDim
    }
    val text = when (state) {
        CheckState.OK      -> "OK"
        CheckState.FAIL    -> if (optional) "NO FIX" else "FAIL"
        CheckState.UNKNOWN -> "-"
    }
    StatusRow(label, text, color)
}

// MGH: 0=לא אומת (צהוב/כתום), 1=OK (ירוק), 2=קפוא (אדום)
@Composable
private fun MagnetometerRow(magHealth: Int) {
    val (text, color) = when (magHealth) {
        MagHealth.OK     -> "OK" to HudGreen
        MagHealth.FROZEN -> "FROZEN" to HudRed
        else             -> "NOT VERIFIED" to HudAmber
    }
    StatusRow("COMPASS", text, color)
}

@Composable
private fun StatusRow(label: String, text: String, color: Color) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(7.dp)
    ) {
        Box(
            modifier = Modifier
                .size(11.dp)
                .clip(RoundedCornerShape(50))
                .background(color.copy(alpha = 0.25f))
                .border(1.5.dp, color, RoundedCornerShape(50))
        )
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.rowLabel,
            color = HudTextSecondary,
            maxLines = 1,
            softWrap = false,
            modifier = Modifier.weight(1f)
        )
        Text(
            text = text,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.rowLabel,
            fontWeight = FontWeight.Bold,
            color = color,
            maxLines = 1,
            softWrap = false
        )
    }
}
