package com.drone.gcs.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*

/**
 * סוללה - כל הנתונים מגיעים מ-MSP2_INAV_ANALOG.
 * שום דבר כאן לא מחושב מחדש. אין זרם ואין mAh - אין חיישן זרם ברחפן.
 * modifier עובר לכרטיס, כדי שבלוח הטלמטריה הכרטיס יימתח לגובה השורה.
 *
 * גדלי הטקסט נלקחים מ-HudText ולא נקבעים כאן, כדי שהגדלה תהיה אחידה בכל המסך.
 */
@Composable
fun BatteryPanel(
    voltageCentivolts: Int,   // 1142 → 11.42V
    percent: Int,             // החישוב של iNav
    state: Int,               // 0=OK 1=WARNING 2=CRITICAL 3=NOT_PRESENT
    cells: Int,
    modifier: Modifier = Modifier,
) {
    val present = state != 3 && voltageCentivolts > 0
    val color = when {
        !present   -> HudTextDim
        state == 2 -> HudRed
        state == 1 -> HudAmber
        else       -> HudGreen
    }
    val stateLabel = when (state) {
        0 -> "OK"; 1 -> "WARN"; 2 -> "CRIT"; else -> "NO BATT"
    }
    val volts = voltageCentivolts / 100f

    HudCard(title = "BATTERY", modifier = modifier) {
        CardBody {

            // ── 11.42V | 38% ──
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    text = if (present) "%.2fV".format(volts) else "--.--V",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = HudText.sensorValue,
                    color = color,
                    maxLines = 1,
                    softWrap = false
                )
                Text(
                    text = " | ",
                    fontFamily = FontFamily.Monospace,
                    fontSize = HudText.rowValue,
                    color = HudTextDim
                )
                Text(
                    text = if (present) "$percent%" else "--%",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = HudText.sensorValue,
                    color = color,
                    maxLines = 1,
                    softWrap = false
                )
            }

            // ── פס דק ──
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(8.dp)
                    .clip(RoundedCornerShape(2.dp))
                    .background(HudBorder)
            ) {
                if (present) {
                    Box(
                        modifier = Modifier
                            .fillMaxHeight()
                            .fillMaxWidth((percent.coerceIn(0, 100)) / 100f)
                            .clip(RoundedCornerShape(2.dp))
                            .background(color)
                    )
                }
            }

            // ── שורה אחת:  OK · 3S · 3.81V/cell ──
            Text(
                text = buildString {
                    append(stateLabel)
                    if (cells > 0) {
                        append("  ·  ${cells}S")
                        if (present) append("  ·  %.2fV/cell".format(volts / cells))
                    }
                },
                fontFamily = FontFamily.Monospace,
                fontSize = HudText.caption,
                letterSpacing = 0.5.sp,
                color = HudTextSecondary,
                maxLines = 1,
                softWrap = false
            )
        }
    }
}
