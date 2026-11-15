package com.drone.gcs.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.Layout
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Constraints
import androidx.compose.ui.unit.Density
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.clickable
import com.drone.gcs.model.FlightPhase
import com.drone.gcs.model.LidarRange
import com.drone.gcs.model.LidarView
import com.drone.gcs.model.MagHealth
import com.drone.gcs.model.MissionLidar
import com.drone.gcs.model.TelemetryData
import com.drone.gcs.ui.theme.*

// ═══════════════════════════════════════════════════════════════════════
//   גדלי הטקסט של המסך - מוגדרים במקום אחד
//
//   כל הכרטיסים מושכים מכאן, כדי שהגדלה תהיה אחידה ולא תלויה בכרטיס.
//   אין להגדיר גודל טקסט ישירות בתוך כרטיס.
// ═══════════════════════════════════════════════════════════════════════
object HudText {
    val cardTitle   = 13.sp    // כותרת כרטיס: LIDAR / ATTITUDE / BATTERY / GPS
    val rowLabel    = 15.sp    // שם שורה: ROLL / PITCH / HDG / LAT / LON
    val rowValue    = 16.sp    // ערך שורה רגיל
    val rowValueBig = 19.sp    // ערך שורה מודגש
    val sensorName  = 17.sp    // FRONT / DOWN
    val sensorValue = 24.sp    // המרחק עצמו
    val sensorState = 17.sp    // טקסט מצב במקום מרחק
    val chip        = 13.sp    // תגיות מצב
    val caption     = 12.sp    // שורת המידע הקטנה בכרטיס הסוללה
    val stickLabel  = 12.sp    // LEFT STICK / RIGHT STICK
    val stickAxis   = 11.sp    // תוויות הצירים על המקל
    val stickValue  = 18.sp    // THR / YAW / PCH / RLL
}

// ═══════════════════════════════════════════════════════════════════════
//   לוח הטלמטריה - רשת של שתי עמודות
//     שורה 1: לידאר ומצב מרחבי - מה שצריך לראות בטיסה
//     שורה 2: סוללה ו-GPS
//   השורות נמתחות וממלאות את העמודה עד למטה (שורה 1 מקבלת יותר).
//   גלילה היא רק מוצא אחרון, כשאין מספיק מקום בכלל.
// ═══════════════════════════════════════════════════════════════════════
private val ROW_WEIGHTS = floatArrayOf(1.15f, 1.0f)

@Composable
fun TelemetryPanel(
    telemetry: TelemetryData,
    modifier: Modifier = Modifier,
    lidarView: Pair<LidarView, LidarView> = LidarView() to LidarView(),   // FRONT / DOWN יציבים
    lidarPending: Set<Char> = emptySet(),           // 'F' / 'D' - שינוי מצב ממתין לתשובה
    onLidarMode: (front: Boolean, long: Boolean) -> Unit = { _, _ -> },
) {
    FillOrScrollRows(
        weights  = ROW_WEIGHTS,
        gap      = 6.dp,
        modifier = modifier.verticalScroll(rememberScrollState())
    ) {
        CardRow(
            left  = { LidarCard(telemetry, lidarView, lidarPending, onLidarMode) },
            right = { AttitudeCard(telemetry) }
        )
        CardRow(
            left  = {
                BatteryPanel(
                    voltageCentivolts = telemetry.batteryVoltCv,
                    percent           = telemetry.batteryPercent,
                    state             = telemetry.batteryState,
                    cells             = telemetry.batteryCells,
                    modifier          = Modifier.fillMaxHeight(),
                )
            },
            right = { GpsCard(telemetry) }
        )
    }
}

// ═══ שורות שממלאות את הגובה הפנוי, או נגללות כשאין מספיק מקום ═══
@Composable
private fun FillOrScrollRows(
    weights: FloatArray,
    gap: Dp,
    modifier: Modifier = Modifier,
    content: @Composable () -> Unit
) {
    Layout(content = content, modifier = modifier) { measurables, constraints ->
        val width   = if (constraints.hasBoundedWidth) constraints.maxWidth else constraints.minWidth
        val gapPx   = gap.roundToPx()
        val gapsPx  = gapPx * (measurables.size - 1).coerceAtLeast(0)
        val natural = measurables.map { it.minIntrinsicHeight(width) }
        val extra   = (constraints.minHeight - natural.sum() - gapsPx).coerceAtLeast(0)
        val wSum    = measurables.indices.sumOf { i -> weights.getOrElse(i) { 1f }.toDouble() }.toFloat()

        val placeables = measurables.mapIndexed { i, m ->
            val add = if (wSum > 0f) (extra * weights.getOrElse(i) { 1f } / wSum).toInt() else 0
            m.measure(Constraints.fixed(width, natural[i] + add))
        }
        val height = maxOf(placeables.sumOf { it.height } + gapsPx, constraints.minHeight)

        layout(width, height) {
            var y = 0
            placeables.forEach { p ->
                p.placeRelative(0, y)
                y += p.height + gapPx
            }
        }
    }
}

// שני כרטיסים באותה שורה - תמיד באותו גובה
@Composable
private fun CardRow(left: @Composable () -> Unit, right: @Composable () -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Box(modifier = Modifier.weight(1f).fillMaxHeight()) { left() }
        Box(modifier = Modifier.weight(1f).fillMaxHeight()) { right() }
    }
}

// ═══ גוף כרטיס: השורות מתפזרות לגובה הכרטיס ═══
@Composable
fun CardBody(content: @Composable ColumnScope.() -> Unit) {
    Column(
        modifier = Modifier.fillMaxWidth().fillMaxHeight(),
        verticalArrangement = CardRowsArrangement,
        content = content
    )
}

internal val CardRowsArrangement: Arrangement.Vertical = SpreadRows(4.dp)

private class SpreadRows(private val minGap: Dp) : Arrangement.Vertical {
    override val spacing: Dp get() = minGap

    override fun Density.arrange(totalSize: Int, sizes: IntArray, outPositions: IntArray) {
        if (sizes.isEmpty()) return
        val gapMin = minGap.roundToPx()
        val free   = totalSize - sizes.sum()
        val gap    = maxOf(gapMin, free / (sizes.size + 1))
        var y      = ((free - gap * (sizes.size - 1)) / 2).coerceAtLeast(0)
        for (i in sizes.indices) {
            outPositions[i] = y
            y += sizes[i] + gap
        }
    }
}

// ── כרטיסים ──────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════════════
//   הלידארים - צבע לפי שלישים של הטווח השמיש, לכל חיישן ולפי המצב שלו
//
//     שליש קרוב  - אדום · שליש אמצעי - צהוב · שליש רחוק - ירוק
//   הטווח השמיש = הטווח של המצב (SHORT 130 / LONG 400, כפי שה-ESP דיווח)
//   פחות הקיזוז של החיישן. הקיזוז של FRONT (18cm עד קצה הפרופלור) כבר
//   הוחל פעם אחת ב-ESP - המספר כאן הוא המרחק מקצה הפרופלור.
//
//   ⚠️ רק מדידה VALID נצבעת לפי מרחק. OUT OF RANGE / WEAK / UNCERTAIN /
//      NO SIGNAL / STALE / FAULT לעולם אינם ירוקים. TOO CLOSE - אדום.
//   ⚠️ DOWN הוא מרחק למשטח שמתחת לרחפן - לא גובה. הגובה הוא של INAV.
//
//   FRONT על הקרקע ובגובה נמוך רואה רצפה: אז הוא מסומן INACTIVE, והמספר
//   מוצג באפור (לבדיקות קרקע) ולא בצבע.
//
//   SHORT / LONG: לחיצה מבקשת מה-ESP להחליף מצב (רק מנוטרל ועל הקרקע).
//   התג מציג רק את המצב שהחיישן דיווח בפועל.
// ═══════════════════════════════════════════════════════════════════════
@Composable
private fun LidarCard(t: TelemetryData, v: Pair<LidarView, LidarView>, pending: Set<Char>,
                      onMode: (Boolean, Boolean) -> Unit) {
    val canChange = !t.motorsLive && t.phase == FlightPhase.GROUND_DISARMED && t.lidarModeKnown
    HudCard(title = "LIDAR (cm)", modifier = Modifier.fillMaxHeight()) {
        CardBody {
            FrontRow(t, v.first)
            ModeRow("FRONT", t.fwdLong, t.fwdMaxCm, 'F' in pending, canChange && 'F' !in pending) {
                onMode(true, !t.fwdLong)
            }
            Box(modifier = Modifier.fillMaxWidth().height(1.dp).background(HudBorder))
            DownRow(t, v.second)
            ModeRow("DOWN", t.dwnLong, t.dwnMaxCm, 'D' in pending, canChange && 'D' !in pending) {
                onMode(false, !t.dwnLong)
            }
        }
    }
}

// שורת מצב המדידה: SHORT / LONG כפי שדווח, והטווח המרבי
@Composable
private fun ModeRow(name: String, long: Boolean, maxCm: Int, pending: Boolean, enabled: Boolean, onToggle: () -> Unit) {
    val tap = LocalTap.current
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text("$name MODE", fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
            color = HudTextSecondary, maxLines = 1, softWrap = false)
        val c = when {
            pending -> HudAmber
            enabled -> HudCyan
            else    -> HudTextDim
        }
        Box(
            modifier = Modifier
                .clip(RoundedCornerShape(4.dp))
                .background(c.copy(alpha = 0.15f))
                .border(1.dp, c, RoundedCornerShape(4.dp))
                .clickable(enabled = enabled) { tap(); onToggle() }
                .padding(horizontal = 8.dp, vertical = 3.dp)
        ) {
            Text(
                text = when {
                    pending -> "SWITCHING..."
                    long    -> "LONG $maxCm"
                    else    -> "SHORT $maxCm"
                },
                fontFamily = FontFamily.Monospace, fontSize = HudText.chip,
                fontWeight = FontWeight.Bold, color = c, maxLines = 1, softWrap = false
            )
        }
    }
}

// מצב שאינו מרחק: טקסט וצבע. אף אחד מהם אינו ירוק.
@Composable
private fun NonValidRow(name: String, cls: Int) {
    when (cls) {
        MissionLidar.TOO_CLOSE    -> SensorRow(name, "TOO CLOSE",    HudRed,           big = false)
        MissionLidar.OUT_OF_RANGE -> SensorRow(name, "OUT OF RANGE", HudTextSecondary, big = false)
        MissionLidar.WEAK         -> SensorRow(name, "WEAK TARGET",  HudAmber,         big = false)
        MissionLidar.UNCERTAIN    -> SensorRow(name, "UNSTABLE",     HudAmber,         big = false)
        // NO SIGNAL (status 2 של VL53L1X) בשטח פתוח = אין מטרה בטווח - אותה משמעות
        // כמו OUT OF RANGE, ולכן אותה תצוגה. תצוגה בלבד: הסיווג שנשלח ל-FPGA לא שונה.
        MissionLidar.NO_SIGNAL    -> SensorRow(name, "OUT OF RANGE", HudTextSecondary, big = false)
        else                      -> SensorRow(name, "SENSOR FAULT", HudRed,           big = false)
    }
}

// FRONT - מקצה דיסקת הפרופלור. v = התצוגה היציבה (LidarSmoother).
@Composable
private fun FrontRow(t: TelemetryData, v: LidarView) {
    val inactive = when {
        t.phase != FlightPhase.AIRBORNE -> "INACTIVE"
        !t.fwdActive                    -> "LOW ALT"
        else                            -> null
    }
    when {
        inactive != null && v.cls == MissionLidar.VALID && v.cm >= 0 ->
            SensorRow("FRONT", "$inactive ${v.cm}cm", HudTextSecondary, big = false)
        inactive != null && v.cls != MissionLidar.FAULT && v.cls != MissionLidar.TOO_CLOSE ->
            SensorRow("FRONT", inactive, HudTextSecondary, big = false)
        v.cls == MissionLidar.VALID && v.cm >= 0 && v.band != null ->
            SensorRow("FRONT", "${v.cm}cm", bandColor(v.band), big = true)
        else -> NonValidRow("FRONT", v.cls)
    }
}

// DOWN - מתחתית הרחפן אל המשטח שמתחתיו (לא גובה)
//   על הקרקע (ה-ESP לא אישר טיסה) TOO CLOSE הוא המצב הרגיל - הרחפן עומד על
//   הרצפה - ולכן מוצג GROUND באפור ולא אזהרה. הרמה ביד מציגה מרחקים כרגיל.
//   באוויר TOO CLOSE נשאר אזהרה אדומה.
@Composable
private fun DownRow(t: TelemetryData, v: LidarView) {
    when {
        v.cls == MissionLidar.VALID && v.cm >= 0 && v.band != null ->
            SensorRow("DOWN", "${v.cm}cm", bandColor(v.band), big = true)
        v.cls == MissionLidar.TOO_CLOSE && t.phase != FlightPhase.AIRBORNE ->
            SensorRow("DOWN", "GROUND", HudTextSecondary, big = false)
        else -> NonValidRow("DOWN", v.cls)
    }
}

@Composable
private fun AttitudeCard(t: TelemetryData) {
    HudCard(title = "ATTITUDE", modifier = Modifier.fillMaxHeight()) {
        CardBody {
            TelRow("ROLL",  "%.1f°".format(t.attRoll),  valueSize = HudText.rowValueBig)
            TelRow("PITCH", "%.1f°".format(t.attPitch), valueSize = HudText.rowValueBig)
            TelRow("HEADING", "${t.heading}°")
            // מגנטומטר חי מ-INAV - שורה קומפקטית אחת לשלושת הצירים
            TelGrid(
                columns = 3,
                cells = listOf(
                    "MAG X" to (if (t.magValid) "${t.magX}" else "---"),
                    "MAG Y" to (if (t.magValid) "${t.magY}" else "---"),
                    "MAG Z" to (if (t.magValid) "${t.magZ}" else "---"),
                )
            )
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                StatusChip("FC LINK", t.fcLinkOK, HudGreen, HudRed)
                // בריאות המצפן מה-ESP: לא מאומת = ענבר, קפוא = אדום
                MagHealth.warning(t.magHealth)?.let { w ->
                    val c = if (t.magHealth == MagHealth.FROZEN) HudRed else HudAmber
                    StatusChip(w, true, c, c)
                }
                if (t.failsafeWarn) {
                    StatusChip("UNEXPECTED DISARM", true, HudRed, HudRed)
                }
            }
        }
    }
}

// GPS - מידע בלבד. בטיסה ידנית (ANGLE) אין בו צורך, והוא לא חוסם חימוש.
@Composable
private fun GpsCard(t: TelemetryData) {
    HudCard(title = "GPS", modifier = Modifier.fillMaxHeight()) {
        CardBody {
            StatusChip(
                label = if (t.gpsStatus == "3D") "GPS 3D FIX" else "GPS NO FIX",
                active = t.gpsStatus == "3D",
                activeColor = HudGreen,
                inactiveColor = HudTextDim
            )
            TelRow("LAT", if (t.gpsLat != 0.0) "%.6f".format(t.gpsLat) else "---")
            TelRow("LON", if (t.gpsLon != 0.0) "%.6f".format(t.gpsLon) else "---")
            // תצוגה ואבחון בלבד. HDOP ללא יחידות, EPH / EPV במטרים - כמו ב-INAV
            TelGrid(
                columns = 2,
                cells = listOf(
                    "SATS" to (if (t.gpsSats >= 0) "${t.gpsSats}" else "---"),
                    "HDOP" to (if (t.gpsHdop >= 0) "%.2f".format(t.gpsHdop / 100f) else "---"),
                    "EPH"  to (if (t.gpsEph  >= 0) "%.2f m".format(t.gpsEph / 100f) else "---"),
                    "EPV"  to (if (t.gpsEpv  >= 0) "%.2f m".format(t.gpsEpv / 100f) else "---"),
                )
            )
        }
    }
}

// ── רשת קומפקטית: תווית קטנה מעל ערך ──
//   לנתוני תצוגה איטיים (GPS מורחב, מגנטומטר): כמה ערכים בשורה אחת, כדי
//   שהכרטיס לא יתארך בשורה נפרדת לכל ערך.
@Composable
private fun TelGrid(cells: List<Pair<String, String>>, columns: Int) {
    Column(
        modifier = Modifier.fillMaxWidth(),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        cells.chunked(columns).forEach { row ->
            Row(modifier = Modifier.fillMaxWidth()) {
                row.forEach { (label, value) ->
                    Column(
                        modifier = Modifier.weight(1f),
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
                            color = HudTextSecondary, maxLines = 1, softWrap = false)
                        Text(value, fontFamily = FontFamily.Monospace, fontSize = HudText.rowValue,
                            color = HudTextPrimary, fontWeight = FontWeight.Bold,
                            maxLines = 1, softWrap = false)
                    }
                }
            }
        }
    }
}

// ── Sub-components ─────────────────────────────────────────────────────────

@Composable
fun HudCard(
    title: String,
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(6.dp))
            .background(HudCard)
            .border(1.dp, HudBorder, RoundedCornerShape(6.dp))
            .padding(horizontal = 10.dp, vertical = 8.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Text(
            text = title,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.cardTitle,
            fontWeight = FontWeight.Bold,
            color = HudCyan,
            letterSpacing = 2.sp,
            maxLines = 1,
            softWrap = false
        )
        content()
    }
}

@Composable
fun TelRow(label: String, value: String, valueSize: TextUnit = HudText.rowValue) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
            color = HudTextSecondary, maxLines = 1, softWrap = false)
        Text(value, fontFamily = FontFamily.Monospace, fontSize = valueSize,
            color = HudTextPrimary, fontWeight = FontWeight.Bold,
            maxLines = 1, softWrap = false)
    }
}

@Composable
fun StatusChip(label: String, active: Boolean, activeColor: Color, inactiveColor: Color) {
    val color = if (active) activeColor else inactiveColor
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(4.dp))
            .background(color.copy(alpha = 0.15f))
            .border(1.dp, color.copy(alpha = 0.5f), RoundedCornerShape(4.dp))
            .padding(horizontal = 8.dp, vertical = 4.dp)
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.chip,
            fontWeight = FontWeight.Bold, color = color, maxLines = 1, softWrap = false)
    }
}

// ── שורת חיישן ────────────────────────────────────────────────────────────
//   שורה אחת בלבד: שם מצד אחד, ערך או מצב מצד שני. בלי מלל הסבר מתחת,
//   ובלי ירידת שורה - "NO READING" חייב להישאר בשורה אחת.
@Composable
private fun SensorRow(name: String, value: String, color: Color, big: Boolean) {
    // גובה קבוע: מעבר בין מרחק (גופן גדול) למצב (גופן קטן) לא מזיז את הכרטיס
    Row(
        modifier = Modifier.fillMaxWidth().height(SENSOR_ROW_HEIGHT),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = name,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.sensorName,
            fontWeight = FontWeight.Bold,
            color = HudTextPrimary,
            maxLines = 1,
            softWrap = false
        )
        Text(
            text = value,
            fontFamily = FontFamily.Monospace,
            fontSize = if (big) HudText.sensorValue else HudText.sensorState,
            fontWeight = FontWeight.Bold,
            color = color,
            maxLines = 1,
            softWrap = false
        )
    }
}

private val SENSOR_ROW_HEIGHT = 36.dp

// ── צבע לפי שליש הטווח - רק למדידה VALID ──
private fun bandColor(b: LidarRange.Band): Color = when (b) {
    LidarRange.Band.NEAR -> HudRed
    LidarRange.Band.MID  -> HudAmber
    LidarRange.Band.FAR  -> HudGreen
}
