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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*
import kotlin.math.abs

@Composable
fun ThrottleSlider(
    throttle:         Float,
    onThrottleChange: (Float) -> Unit,
    modifier:         Modifier = Modifier,
) {
    var norm by remember { mutableFloatStateOf(throttle / 100f) }

    LaunchedEffect(throttle) {
        val expected = throttle / 100f
        if (abs(expected - norm) > 0.02f) norm = expected
    }

    LaunchedEffect(Unit) {
        onThrottleChange(norm * 100f)
    }

    Column(
        modifier            = modifier,
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Text(
            text       = "THR",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize   = 11.sp,
            color      = HudTextDim
        )

        Box(
            modifier = Modifier
                .weight(1f)
                .width(64.dp),
            contentAlignment = Alignment.Center
        ) {
            Canvas(
                modifier = Modifier
                    .fillMaxHeight()
                    .width(64.dp)
                    .pointerInput(Unit) {
                        val trackH = size.height.toFloat()

                        awaitEachGesture {
                            val down = awaitFirstDown(requireUnconsumed = false)
                            down.consume()

                            while (true) {
                                val event  = awaitPointerEvent()
                                val change = event.changes.find { it.id == down.id } ?: break
                                change.consume()

                                if (!change.pressed) break

                                val newNorm = (1f - change.position.y / trackH).coerceIn(0f, 1f)
                                norm = newNorm
                                onThrottleChange(newNorm * 100f)
                            }
                        }
                    }
            ) {
                val trackW      = 18.dp.toPx()
                val trackLeft   = (size.width - trackW) / 2f
                val trackTop    = 20.dp.toPx()
                val trackBottom = size.height - 20.dp.toPx()
                val trackH      = trackBottom - trackTop
                val thumbR      = 14.dp.toPx()

                drawRoundRect(
                    color        = HudBorder,
                    topLeft      = Offset(trackLeft, trackTop),
                    size         = Size(trackW, trackH),
                    cornerRadius = CornerRadius(trackW / 2f)
                )

                val fillH   = trackH * norm
                val fillTop = trackBottom - fillH
                drawRoundRect(
                    brush = Brush.verticalGradient(
                        colors = listOf(HudCyan, HudCyanDim),
                        startY = fillTop,
                        endY   = trackBottom
                    ),
                    topLeft      = Offset(trackLeft, fillTop),
                    size         = Size(trackW, fillH),
                    cornerRadius = CornerRadius(trackW / 2f)
                )

                listOf(0.25f, 0.5f, 0.75f).forEach { tick ->
                    val tickY = trackBottom - trackH * tick
                    drawLine(
                        color       = HudTextDim,
                        start       = Offset(trackLeft - 6.dp.toPx(), tickY),
                        end         = Offset(trackLeft, tickY),
                        strokeWidth = 1.dp.toPx()
                    )
                }

                val thumbY = trackBottom - trackH * norm
                drawCircle(
                    color  = HudCyan.copy(alpha = 0.20f),
                    radius = thumbR + 6.dp.toPx(),
                    center = Offset(size.width / 2f, thumbY)
                )
                drawCircle(
                    color  = HudCyan,
                    radius = thumbR,
                    center = Offset(size.width / 2f, thumbY)
                )
                drawCircle(
                    color  = HudBackground,
                    radius = 5.dp.toPx(),
                    center = Offset(size.width / 2f, thumbY)
                )
            }
        }

        Text(
            text       = "${(norm * 100).toInt()}%",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize   = 16.sp,
            color      = HudCyan
        )

        Text(
            text       = "0%",
            fontFamily = FontFamily.Monospace,
            fontSize   = 10.sp,
            color      = HudTextDim
        )
    }
}