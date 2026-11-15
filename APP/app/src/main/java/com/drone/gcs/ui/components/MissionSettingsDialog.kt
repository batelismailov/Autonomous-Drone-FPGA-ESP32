package com.drone.gcs.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.activity.compose.BackHandler
import androidx.compose.foundation.interaction.MutableInteractionSource
import com.drone.gcs.model.MissionSettings
import com.drone.gcs.ui.theme.*

// ═══════════════════════════════════════════════════════════════════════
//   כפתור גלגל שיניים - מצויר, לא תו גרפי (בגופן של הטאבלט חלק מהתווים
//   הופיעו כריבוע)
// ═══════════════════════════════════════════════════════════════════════
@Composable
fun GearButton(enabled: Boolean, onClick: () -> Unit, size: Dp = 34.dp) {
    val tap = LocalTap.current
    val c = if (enabled) HudCyan else HudTextDim
    Box(
        modifier = Modifier
            .size(size)
            .clip(RoundedCornerShape(6.dp))
            .background(c.copy(alpha = 0.12f))
            .border(1.dp, c, RoundedCornerShape(6.dp))
            .clickable(enabled = enabled) { tap(); onClick() },
        contentAlignment = Alignment.Center
    ) {
        Canvas(modifier = Modifier.size(size * 0.62f)) {
            val r      = this.size.minDimension / 2f
            val center = Offset(this.size.width / 2f, this.size.height / 2f)
            val tooth  = Size(r * 0.36f, r * 0.42f)
            for (i in 0 until 8) {
                rotate(degrees = i * 45f, pivot = center) {
                    drawRect(color = c, topLeft = Offset(center.x - tooth.width / 2f, center.y - r), size = tooth)
                }
            }
            drawCircle(color = c, radius = r * 0.66f, center = center, style = Stroke(width = r * 0.30f))
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//   הגדרות המשימה - Mission Settings
//
//   מקור האמת הוא ה-NVS של ה-ESP. הדיאלוג מציג את הסט שה-ESP מדווח (NOW),
//   ועורך עותק. SAVE שולח את כל הסט כיחידה אחת; הצלחה מוצגת רק אחרי שה-ESP
//   ענה MSET:OK - כלומר אחרי שה-FPGA החזיר את אותו checksum.
//   שינוי מותר רק כשמנוטרל, על הקרקע, והמשימה לא פעילה (ה-ESP בודק שוב).
//   ⚠️ אין כאן הגדרות INAV (מהירות WP, nav, PID) - הן נקבעות ב-INAV בלבד.
//   ⚠️ בלי מקלדת: צעדים של ±1 / ±10 - נוח במסך מגע ולא מסתיר את המסך.
// ═══════════════════════════════════════════════════════════════════════
@Composable
fun MissionSettingsDialog(
    current: MissionSettings?,          // הסט שב-ESP (MSE), null = לא דווח
    confirmed: Boolean,                 // MSK - ה-FPGA אישר את הסט הזה
    blockReason: String?,               // למה אי אפשר לשנות כרגע, או null
    fwdMaxCm: Int,                      // הטווח של FRONT במצב הנוכחי
    pending: Boolean,                   // שמירה ממתינה לתשובה
    onSave: (MissionSettings) -> Unit,
    onDismiss: () -> Unit,
) {
    val base = current ?: MissionSettings()
    var edit by remember(current) { mutableStateOf(base) }
    val problem = edit.problem(fwdMaxCm)
    val changed = edit != base
    val canSave = blockReason == null && problem == null && !pending && (changed || !confirmed)

    // ⚠️ שכבה בתוך המסך, לא Dialog: חלון Dialog נפרד קיבל בפריים הראשון את כיוון
    //    השפה של הטאבלט (עברית = ימין-לשמאל) והציג לשבריר שנייה את כל החלון בהיפוך.
    //    כאן הכל באותו חלון, באותו כיוון - בלי היפוך ובלי אנימציית פתיחה.
    BackHandler(onBack = onDismiss)
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.6f))
            // נגיעה מחוץ לחלון סוגרת (כמו קודם); לא עוברת לכפתורים שמתחת
            .clickable(interactionSource = remember { MutableInteractionSource() }, indication = null,
                onClick = onDismiss),
        contentAlignment = Alignment.Center
    ) {
        Column(
            // גובה לפי התוכן (לא כל המסך), והחלון עצמו ממורכז במסך
            modifier = Modifier
                .fillMaxWidth(0.62f)
                .wrapContentHeight()
                // נגיעה בתוך החלון לא סוגרת אותו
                .clickable(interactionSource = remember { MutableInteractionSource() }, indication = null) { }
                .clip(RoundedCornerShape(10.dp))
                .background(HudSurface)
                .border(2.dp, HudCyan, RoundedCornerShape(10.dp))
                .padding(horizontal = 18.dp, vertical = 16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically) {
                Text("MISSION SETTINGS", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                    fontSize = HudText.sensorName, letterSpacing = 2.sp, color = HudCyan, maxLines = 1, softWrap = false)
                val (st, sc) = when {
                    current == null -> "NOT REPORTED BY ESP" to HudRed
                    confirmed       -> "ESP + FPGA CONFIRMED" to HudGreen
                    else            -> "NOT CONFIRMED BY FPGA" to HudAmber
                }
                Text(st, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                    fontSize = HudText.caption, color = sc, maxLines = 1, softWrap = false)
            }

            Column(
                modifier = Modifier.fillMaxWidth().heightIn(max = 600.dp).verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(10.dp)
            ) {
                CmField(MissionSettings.Field.STOP,  edit.stopCm,  base.stopCm)  { edit = edit.copy(stopCm = it) }
                CmField(MissionSettings.Field.CLEAR, edit.clearCm, base.clearCm) { edit = edit.copy(clearCm = it) }
                CmField(MissionSettings.Field.VCLR,  edit.vclrCm,  base.vclrCm)  { edit = edit.copy(vclrCm = it) }
                CmField(MissionSettings.Field.FCLR,  edit.fclrCm,  base.fclrCm)  { edit = edit.copy(fclrCm = it) }
                CmField(MissionSettings.Field.CEIL,  edit.ceilCm,  base.ceilCm)  { edit = edit.copy(ceilCm = it) }
                CmField(MissionSettings.Field.DEPTH, edit.depthCm, base.depthCm) { edit = edit.copy(depthCm = it) }

                Text("FRONT RANGE NOW: $fwdMaxCm cm  ·  DISTANCES FROM THE FRONT PROP TIP  ·  CEILING RELATIVE TO INAV HOME",
                    fontFamily = FontFamily.Monospace, fontSize = HudText.caption, color = HudTextDim,
                    maxLines = 1, softWrap = false)

                SectionDivider()

                // שלב הבדיקה - מאפשר להתקדם בהדרגה בלי צריבה
                Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("TEST STAGE", fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                        color = HudTextSecondary, modifier = Modifier.width(170.dp), maxLines = 1, softWrap = false)
                    listOf(2 to "2 STOP", 3 to "3 +CLIMB", 4 to "4 +CROSS", 5 to "5 FULL").forEach { (v, l) ->
                        ToggleChip(l, edit.stage == v, if (v == 5) HudAmber else HudCyan) { edit = edit.copy(stage = v) }
                    }
                    Text("NOW ${base.stage}", fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
                        color = HudTextDim, maxLines = 1, softWrap = false)
                }

                Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("AUTO TAKEOFF", fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                        color = HudTextSecondary, modifier = Modifier.width(170.dp), maxLines = 1, softWrap = false)
                    ToggleChip("OFF", !edit.takeoff, HudCyan) { edit = edit.copy(takeoff = false) }
                    ToggleChip("ON", edit.takeoff, HudRed) { edit = edit.copy(takeoff = true) }
                    Text(if (edit.takeoff) "ONLY AFTER GROUND TESTS WITHOUT PROPS" else "START IN THE AIR ONLY",
                        fontFamily = FontFamily.Monospace, fontSize = HudText.caption,
                        color = if (edit.takeoff) HudRed else HudTextDim, maxLines = 1, softWrap = false)
                }

            }

            // למה אי אפשר לשמור - בלי חידות
            val status = when {
                pending              -> "SAVING - WAITING FOR ESP + FPGA..."
                blockReason != null  -> "LOCKED: $blockReason"
                problem != null      -> "OUT OF RANGE: $problem"
                !changed && confirmed -> "NO CHANGES"
                else                 -> "READY TO SAVE"
            }
            Text(status, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold, fontSize = HudText.caption,
                color = if (problem != null || blockReason != null) HudAmber else HudTextSecondary,
                maxLines = 1, softWrap = false)

            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                DialogButton("CLOSE", HudTextSecondary, true, Modifier.weight(1f), onDismiss)
                DialogButton("DEFAULTS", HudAmber, blockReason == null && !pending, Modifier.weight(1f)) {
                    edit = MissionSettings()
                }
                DialogButton(if (pending) "SAVING..." else "SAVE", HudGreen, canSave, Modifier.weight(1.4f)) {
                    onSave(edit)
                }
            }
        }
    }
}

// קו מפריד בין קבוצות, עם מרווח קטן מעליו ומתחתיו
@Composable
private fun SectionDivider() {
    Box(modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp).height(1.dp).background(HudBorder))
}

// שדה מרחק: שם, טווח, הערך שב-ESP כעת, וצעדים
@Composable
private fun CmField(f: MissionSettings.Field, value: Int, now: Int, onChange: (Int) -> Unit) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        Column(modifier = Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(3.dp)) {
            Text(f.label, fontFamily = FontFamily.Monospace, fontSize = HudText.rowLabel,
                color = HudTextSecondary, maxLines = 1, softWrap = false)
            Text("${f.range.first}-${f.range.last} cm  ·  NOW $now cm", fontFamily = FontFamily.Monospace,
                fontSize = HudText.caption, color = HudTextDim, maxLines = 1, softWrap = false)
        }
        listOf(-10, -1).forEach { d -> StepButton(d) { onChange((value + d).coerceIn(f.range)) } }
        Text("$value cm", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
            fontSize = HudText.rowValueBig, color = if (value != now) HudAmber else HudTextPrimary,
            modifier = Modifier.width(96.dp), maxLines = 1, softWrap = false)
        listOf(1, 10).forEach { d -> StepButton(d) { onChange((value + d).coerceIn(f.range)) } }
    }
}

@Composable
private fun StepButton(delta: Int, onClick: () -> Unit) {
    val tap = LocalTap.current
    Box(
        modifier = Modifier
            .size(width = 52.dp, height = 38.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(HudCyan.copy(alpha = 0.12f))
            .border(1.dp, HudCyan, RoundedCornerShape(6.dp))
            .clickable { tap(); onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(if (delta > 0) "+$delta" else "$delta", fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold, fontSize = HudText.chip, color = HudCyan, maxLines = 1, softWrap = false)
    }
}

@Composable
fun ToggleChip(label: String, selected: Boolean, color: Color, onClick: () -> Unit) {
    val tap = LocalTap.current
    val c = if (selected) color else HudTextDim
    Box(
        modifier = Modifier
            .height(34.dp)
            .clip(RoundedCornerShape(5.dp))
            .background(if (selected) c.copy(alpha = 0.22f) else Color.Transparent)
            .border(1.dp, c, RoundedCornerShape(5.dp))
            .clickable { tap(); onClick() }
            .padding(horizontal = 10.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
            fontSize = HudText.chip, color = c, maxLines = 1, softWrap = false)
    }
}

@Composable
private fun DialogButton(text: String, color: Color, enabled: Boolean, modifier: Modifier, onClick: () -> Unit) {
    val tap = LocalTap.current
    val c = if (enabled) color else HudTextDim
    Box(
        modifier = modifier
            .height(44.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(c.copy(alpha = 0.18f))
            .border(1.dp, c, RoundedCornerShape(6.dp))
            .clickable(enabled = enabled) { tap(); onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(text, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
            fontSize = HudText.caption, color = c, maxLines = 1, softWrap = false)
    }
}
