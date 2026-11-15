package com.drone.gcs.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.*
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*
import kotlin.math.abs

@Composable
fun YawSlider(
    yaw:         Float,   // -100f – 100f, auto-centers on release
    onYawChange: (Float) -> Unit,
    modifier:    Modifier = Modifier,
) {
    // norm: -1f (שמאל) → 0f (מרכז) → +1f (ימין)
    var norm by remember { mutableFloatStateOf(yaw / 100f) }

    // Snap back to center on LINK LOST
    LaunchedEffect(yaw) {
        val expected = yaw / 100f
        if (abs(expected - norm) > 0.02f) norm = expected
    }

    Column(
        modifier            = modifier,
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Text("YAW", fontFamily = FontFamily.Monospace,
            fontSize = 11.sp, fontWeight = FontWeight.Bold, color = HudTextDim)

        Canvas(
            modifier = Modifier
                .fillMaxWidth()
                .height(56.dp)
                .pointerInput(Unit) {
                    val trackW = size.width.toFloat()
                    awaitEachGesture {
                        val down = awaitFirstDown(requireUnconsumed = false)
                        down.consume()
                        while (true) {
                            val event  = awaitPointerEvent()
                            val change = event.changes.find { it.id == down.id } ?: break
                            change.consume()
                            if (!change.pressed) {
                                // Auto-center on release
                                norm = 0f
                                onYawChange(0f)
                                break
                            }
                            val newNorm = ((change.position.x / trackW) * 2f - 1f)
                                .coerceIn(-1f, 1f)
                            norm = newNorm
                            onYawChange(newNorm * 100f)
                        }
                    }
                }
        ) {
            val trackH    = 16.dp.toPx()
            val trackTop  = (size.height - trackH) / 2f
            val trackW    = size.width
            val thumbR    = 14.dp.toPx()
            val centerX   = trackW / 2f
            val thumbX    = centerX + norm * (centerX - thumbR)

            // Track background
            drawRoundRect(
                color        = HudBorder,
                topLeft      = Offset(0f, trackTop),
                size         = Size(trackW, trackH),
                cornerRadius = CornerRadius(trackH / 2f)
            )

            // Fill — center to thumb
            val fillStart = if (norm >= 0) centerX else thumbX
            val fillWidth = abs(norm) * (centerX - thumbR)
            if (fillWidth > 0f) {
                drawRoundRect(
                    color        = HudCyan.copy(alpha = 0.6f),
                    topLeft      = Offset(fillStart, trackTop),
                    size         = Size(fillWidth, trackH),
                    cornerRadius = CornerRadius(trackH / 2f)
                )
            }

            // Center notch
            drawLine(HudTextDim, Offset(centerX, trackTop - 4.dp.toPx()),
                Offset(centerX, trackTop + trackH + 4.dp.toPx()), 1.dp.toPx())

            // Thumb
            drawCircle(HudCyan.copy(alpha = 0.18f), thumbR + 5.dp.toPx(), Offset(thumbX, size.height / 2f))
            drawCircle(HudCyan, thumbR, Offset(thumbX, size.height / 2f))
            drawCircle(HudBackground, 5.dp.toPx(), Offset(thumbX, size.height / 2f))
        }

        Text(
            text       = "${(norm * 100).toInt()}",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize   = 13.sp,
            color      = if (abs(norm) < 0.05f) HudTextDim else HudCyan
        )
    }
}