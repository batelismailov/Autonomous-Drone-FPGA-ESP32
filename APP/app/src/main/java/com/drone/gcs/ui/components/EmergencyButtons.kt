package com.drone.gcs.ui.components

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*

/**
 * ABORT - כיבוי מנועים מיידי (kill switch): ה-FPGA מוריד את החימוש ומשדר
 *         AUX1=1000, כלומר INAV מנתק. הרחפן נופל. שתי לחיצות תוך 3 שניות.
 *
 * כפתור LAND הוסר: אין נחיתה שממומשת ב-FPGA. נחיתה תחזור כשתוגדר ותיבדק
 * נחיתה מבוססת INAV (ALTHOLD מכויל → LAND → ניתוק בנחיתה).
 */
@Composable
fun EmergencyButtons(
    onAbort: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val tap = LocalTap.current
    var abortArmed by rememberPendingConfirm()

    Button(
        onClick = {
            tap()
            if (abortArmed) { onAbort(); abortArmed = false }
            else            { abortArmed = true }
        },
        shape  = RoundedCornerShape(8.dp),
        colors = ButtonDefaults.buttonColors(
            containerColor = if (abortArmed) HudRed else HudRedDark,
            contentColor   = Color.White
        ),
        modifier = modifier
            .fillMaxWidth()
            .height(48.dp)
            .border(
                width = if (abortArmed) 2.dp else 1.dp,
                color = HudRed,
                shape = RoundedCornerShape(8.dp)
            )
    ) {
        // צבע מ-LocalContentColor, לא מסגנון ברירת המחדל של הכפתור
        Text(
            text = if (abortArmed) "CONFIRM ABORT - MOTORS OFF" else "ABORT",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = HudText.rowLabel,
            letterSpacing = 1.sp,
            color = LocalContentColor.current,
            maxLines = 1,
            softWrap = false
        )
    }
}
