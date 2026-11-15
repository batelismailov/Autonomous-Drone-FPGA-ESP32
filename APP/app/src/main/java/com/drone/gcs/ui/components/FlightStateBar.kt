package com.drone.gcs.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.model.FlightPhase
import com.drone.gcs.model.FpgaState
import com.drone.gcs.ui.theme.*
import kotlinx.coroutines.delay

// ═══════════════════════════════════════════════════════════════════════
//   מצב ה-FSM של ה-FPGA + שלב הטיסה + אבחון
//
//   שתי שורות מידע שונות לגמרי:
//     FPGA  - שכבת הבטיחות: האם זרם ה-RC משודר, ומה עצר אותו לאחרונה
//     PHASE - קרקע או אוויר, כפי שה-ESP32 קבע מחיישן DWN ומגובה INAV.
//             חימוש הוא לא המראה: ARMED אינו AIRBORNE.
// ═══════════════════════════════════════════════════════════════════════
@Composable
fun FlightStateBar(
    state: FpgaState,
    phase: FlightPhase,
    fpgaDebug: String,
    modifier: Modifier = Modifier,
) {
    val color = when (state) {
        FpgaState.DISARMED  -> HudTextSecondary
        FpgaState.READY     -> HudCyan
        FpgaState.ARMED     -> HudGreen
        FpgaState.LINK_LOSS -> HudRed
        FpgaState.ABORT     -> HudRed
        FpgaState.RESERVED  -> HudTextDim
        FpgaState.UNKNOWN   -> HudTextDim
    }
    val phaseColor = when (phase) {
        FlightPhase.AIRBORNE        -> HudAmber
        FlightPhase.TAKEOFF_CONFIRM -> HudAmber
        FlightPhase.GROUND_ARMED    -> HudGreen
        FlightPhase.GROUND_DISARMED -> HudTextSecondary
    }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(6.dp))
            .background(color.copy(alpha = 0.12f))
            .border(1.5.dp, color, RoundedCornerShape(6.dp))
            .padding(horizontal = 10.dp, vertical = 6.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(3.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "FPGA  ${state.label}",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = HudText.sensorName,
                letterSpacing = 2.sp,
                color = color,
                maxLines = 1,
                softWrap = false
            )
            StatusChip(phase.label, true, phaseColor, phaseColor)
        }

        Text(
            text = state.hint,
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.caption,
            letterSpacing = 1.sp,
            color = HudTextSecondary,
            maxLines = 1,
            softWrap = false,
            modifier = Modifier.fillMaxWidth()
        )

        Text(
            text = phase.hint + "  ·  " + fpgaDebugText(fpgaDebug),
            fontFamily = FontFamily.Monospace,
            fontSize = HudText.caption,
            color = HudTextDim,
            maxLines = 1,
            softWrap = false,
            modifier = Modifier.fillMaxWidth()
        )
    }
}

// ── אבחון ה-FPGA: ספרת דגלים (hex) + סיבת העצירה האחרונה ───────────────────
//   bit3 זרם ה-RC עצור · bit2 דגימת FWD טרייה · bit1 דגימת DWN טרייה
//   bit0 פקודות שליטה טריות
//   סיבה: L אין פקודות שליטה · E ה-ESP מת · - אין
private fun fpgaDebugText(dbg: String): String {
    val flags = dbg.getOrNull(0)?.digitToIntOrNull(16) ?: return "FPGA ---"
    val parts = mutableListOf<String>()
    parts += if ((flags and 0x8) != 0) "RC STOPPED" else "RC OK"
    if ((flags and 0x4) != 0) parts += "FWD"
    if ((flags and 0x2) != 0) parts += "DWN"
    if ((flags and 0x1) != 0) parts += "CTRL"
    val end = when (dbg.getOrNull(1)) {
        'L'  -> "CONTROL LOST"
        'E'  -> "ESP LOST"
        '-'  -> "-"
        else -> "?"
    }
    return parts.joinToString(" ") + "  ·  LAST STOP " + end
}

// ── אישור בשתי לחיצות ─────────────────────────────────────────────────────
// לחיצה ראשונה מבקשת אישור, השנייה מבצעת. בלי לחיצה שנייה תוך 3 שניות
// הבקשה מתבטלת, כדי שלחיצה מקרית מאוחרת לא תבצע פעולה מסוכנת.
// active=false (למשל המנועים כבר כבויים) מבטל בקשה פתוחה מיד.
const val CONFIRM_TIMEOUT_MS = 3000L

@Composable
fun rememberPendingConfirm(active: Boolean = true): MutableState<Boolean> {
    val pending = remember { mutableStateOf(false) }
    LaunchedEffect(pending.value) {
        if (pending.value) {
            delay(CONFIRM_TIMEOUT_MS)
            pending.value = false
        }
    }
    LaunchedEffect(active) {
        if (!active) pending.value = false
    }
    return pending
}
