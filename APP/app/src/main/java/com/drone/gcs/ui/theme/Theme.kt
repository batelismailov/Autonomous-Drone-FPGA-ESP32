package com.drone.gcs.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

// משוב לחיצה (צליל קצר + רטט קצר) לכל כפתור. MainControlScreen מספק אותו מה-ViewModel.
// ⚠️ לא בכפתורי +/- של המצערת הקבועה ב-ANGLE (HoverTrimButton) - בכוונה.
val LocalTap = staticCompositionLocalOf<() -> Unit> { {} }

private val DroneColorScheme = darkColorScheme(
    primary          = HudCyan,
    onPrimary        = HudBackground,
    secondary        = HudGreen,
    onSecondary      = HudBackground,
    error            = HudRed,
    onError          = Color.White,
    background       = HudBackground,
    onBackground     = HudTextPrimary,
    surface          = HudSurface,
    onSurface        = HudTextPrimary,
    surfaceVariant   = HudCard,
    onSurfaceVariant = HudTextSecondary,
    outline          = HudBorder,
)

@Composable
fun DroneGCSTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = DroneColorScheme,
        typography  = MaterialTheme.typography.copy(
            bodyLarge  = TextStyle(
                fontFamily = FontFamily.Monospace,
                fontSize   = 14.sp,
                color      = HudTextPrimary
            ),
            bodyMedium = TextStyle(
                fontFamily = FontFamily.Monospace,
                fontSize   = 12.sp,
                color      = HudTextSecondary
            ),
            // ⚠️ בלי color כאן!  labelLarge הוא סגנון ברירת המחדל של Text בתוך Button,
            //    וצבע שנקבע בסגנון גובר על ה-contentColor של הכפתור. כשהיה כאן צבע
            //    קבוע, כל הכפתורים הופיעו בטורקיז - גם ARM MOTORS על רקע ירוק.
            labelLarge = TextStyle(
                fontFamily = FontFamily.Monospace,
                fontSize   = 13.sp,
                fontWeight = FontWeight.Bold
            ),
            titleLarge = TextStyle(
                fontFamily = FontFamily.Monospace,
                fontSize   = 20.sp,
                fontWeight = FontWeight.Bold,
                color      = HudTextPrimary
            ),
        ),
        content = content
    )
}
