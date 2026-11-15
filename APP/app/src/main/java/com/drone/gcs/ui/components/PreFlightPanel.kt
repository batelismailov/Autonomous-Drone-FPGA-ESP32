package com.drone.gcs.ui.components

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.LocalContentColor
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*

// ═══════════════════════════════════════════════════════════════════════
//   כפתור חימוש / ניתוק אחד ויחיד
//
//   ARM     - זמין רק כשבדיקות החובה עברו. חסום ⇒ כתובה הסיבה.
//   DISARM  - זמין תמיד כשהמנועים חיים, ודורש שתי לחיצות תוך 3 שניות
//             (ניתוק באוויר עוצר את המנועים מיד).
//   CLEAR   - ניקוי ABORT או עצירת זרם RC אחרי אובדן קשר. המנועים כבר
//             כבויים, ולכן לחיצה אחת מספיקה.
//
//   ⚠️ בלי תווים גרפיים - בגופן של הטאבלט חלקם הופיעו כריבוע.
//   ⚠️ צבע הטקסט מגיע מ-LocalContentColor, כדי שלא יגבר עליו סגנון ברירת המחדל.
//
//   הלוח הישן PRE-FLIGHT CHECK הוסר והבדיקות עברו ל-SystemCheckPanel.
//   בורר מצבי הטיסה יושב בכרטיס FLIGHT MODE שבעמודה הימנית של מסך השליטה:
//   ANGLE / ALTHOLD / POSHOLD / RTH / WP, לפי טווחי ה-AUX שהוגדרו ב-INAV.
//   חימוש נחסם כאן כל עוד המצב המבוקש אינו ANGLE.
// ═══════════════════════════════════════════════════════════════════════
enum class ArmButtonMode { ARM, DISARM, CLEAR }

@Composable
fun ArmDisarmButton(
    mode: ArmButtonMode,
    canArm: Boolean,
    blockReason: String?,
    onArm: () -> Unit,
    onDisarm: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val tap = LocalTap.current
    var confirm by rememberPendingConfirm(active = mode == ArmButtonMode.DISARM)

    val enabled = mode != ArmButtonMode.ARM || canArm
    val color = when (mode) {
        ArmButtonMode.ARM    -> HudGreen
        ArmButtonMode.DISARM -> HudRed
        ArmButtonMode.CLEAR  -> HudAmber
    }
    val text = when (mode) {
        ArmButtonMode.ARM    -> if (canArm) "ARM MOTORS" else "ARM BLOCKED"
        ArmButtonMode.DISARM -> if (confirm) "CONFIRM DISARM" else "DISARM MOTORS"
        ArmButtonMode.CLEAR  -> "CLEAR - DISARM"
    }

    Column(
        modifier = modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(2.dp)
    ) {
        Button(
            onClick = {
                tap()
                when (mode) {
                    ArmButtonMode.ARM    -> onArm()
                    ArmButtonMode.CLEAR  -> onDisarm()
                    ArmButtonMode.DISARM ->
                        if (confirm) { confirm = false; onDisarm() } else { confirm = true }
                }
            },
            enabled = enabled,
            shape   = RoundedCornerShape(8.dp),
            colors  = ButtonDefaults.buttonColors(
                containerColor         = if (mode == ArmButtonMode.ARM) color else color.copy(alpha = 0.22f),
                // טקסט כהה על ירוק - לבן לא קריא
                contentColor           = if (mode == ArmButtonMode.ARM) HudBackground else color,
                disabledContainerColor = HudTextDim.copy(alpha = 0.15f),
                disabledContentColor   = HudTextDim
            ),
            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp)
                .border(
                    width = if (confirm) 2.dp else 1.5.dp,
                    color = if (confirm) Color.White else if (enabled) color else HudBorder,
                    shape = RoundedCornerShape(8.dp)
                )
        ) {
            Text(
                text          = text,
                fontFamily    = FontFamily.Monospace,
                fontWeight    = FontWeight.Bold,
                fontSize      = HudText.sensorName,
                letterSpacing = 1.sp,
                color         = LocalContentColor.current,
                maxLines      = 1,
                softWrap      = false
            )
        }

        if (mode == ArmButtonMode.ARM && !canArm && blockReason != null) {
            Text(
                text = "WAITING FOR: $blockReason",
                fontFamily = FontFamily.Monospace,
                fontSize = HudText.caption,
                letterSpacing = 1.sp,
                color = HudAmber,
                textAlign = TextAlign.Center,
                maxLines = 1,
                softWrap = false,
                modifier = Modifier.fillMaxWidth()
            )
        }
    }
}
