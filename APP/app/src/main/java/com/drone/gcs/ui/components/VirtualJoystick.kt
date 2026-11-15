package com.drone.gcs.ui.components

import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.spring
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.*
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*
import kotlinx.coroutines.launch
import kotlin.math.abs

/**
 * Generic virtual joystick — covers both left and right sticks.
 *
 * RC coordinate convention (matches drone firmware):
 *   normX  -1.0 = full left    /  +1.0 = full right
 *   normY  -1.0 = full down    /  +1.0 = full up   (screen Y is inverted here)
 *
 * Left stick  (THR + YAW):
 *   xAutoCenter = true   -> YAW springs back to 0 on release
 *   yAutoCenter = false  -> THR stays wherever the pilot left it
 *   yRange      = 0f..100f
 *
 * Right stick (PCH + RLL):
 *   xAutoCenter = true
 *   yAutoCenter = true
 *   yRange      = -100f..100f
 */
@Composable
fun VirtualJoystick(
    modifier: Modifier = Modifier,
    xLabel: String,
    yLabel: String,
    externalX: Float,   // current value from ViewModel — used to sync forced resets
    externalY: Float,
    xAutoCenter: Boolean = true,
    yAutoCenter: Boolean = true,
    yRange: ClosedFloatingPointRange<Float> = -100f..100f,
    onXChange: (Float) -> Unit,
    onYChange: (Float) -> Unit,
) {
    // ── Normalised thumb position in [-1, 1] ──────────────────────────────
    // For THR (yAutoCenter = false) the initial norm is -1 so the thumb
    // sits at the bottom of the joystick (0 % throttle) on first launch.
    val initNormY = if (!yAutoCenter) normFromValue(externalY, yRange) else 0f
    val normX     = remember { Animatable(0f) }
    val normY     = remember { Animatable(initNormY) }
    val scope     = rememberCoroutineScope()

    // Sync initial visual state to ViewModel on first launch (prevents 50% THR jump)
    LaunchedEffect(Unit) {
        if (!yAutoCenter) {
            onYChange(valueFromNorm(normY.value, yRange))
        }
    }

    // Sync forced external resets (e.g. LINK LOST zeros all axes in ViewModel)
    LaunchedEffect(externalX) {
        val expected = externalX / 100f
        if (abs(expected - normX.value) > 0.05f) normX.snapTo(expected)
    }
    LaunchedEffect(externalY) {
        val expected = normFromValue(externalY, yRange)
        if (abs(expected - normY.value) > 0.05f) normY.snapTo(expected)
    }

    var radius by remember { mutableFloatStateOf(200f) }

    Column(
        modifier             = modifier,
        horizontalAlignment  = Alignment.CenterHorizontally,
        verticalArrangement  = Arrangement.Center
    ) {

        // Top axis label (Y) - גודל אחיד מ-HudText
        Text(
            text       = yLabel,
            fontFamily = FontFamily.Monospace,
            fontSize   = HudText.stickAxis,
            fontWeight = FontWeight.Bold,
            color      = HudTextDim,
            maxLines   = 1,
            softWrap   = false
        )
        Spacer(Modifier.height(4.dp))

        Row(verticalAlignment = Alignment.CenterVertically) {

            // Left axis label (X)
            Text(
                text       = xLabel,
                fontFamily = FontFamily.Monospace,
                fontSize   = HudText.stickAxis,
                fontWeight = FontWeight.Bold,
                color      = HudTextDim,
                maxLines   = 1,
                softWrap   = false,
                modifier   = Modifier.width(36.dp)
            )

            // ── Joystick canvas ───────────────────────────────────────────
            Box(
                modifier = Modifier
                    .weight(1f)
                    .aspectRatio(1f)
                    .pointerInput(Unit) {
                        // Pre-compute geometry once
                        val center = Offset(size.width / 2f, size.height / 2f)
                        radius     = minOf(size.width, size.height) / 2f * 0.82f

                        awaitEachGesture {
                            // Wait for the first finger down
                            val down = awaitFirstDown(requireUnconsumed = false)
                            down.consume()

                            while (true) {
                                val event  = awaitPointerEvent()
                                val change = event.changes.find { it.id == down.id } ?: break
                                change.consume()

                                if (!change.pressed) {
                                    // ── Finger lifted ─────────────────────
                                    if (xAutoCenter) {
                                        scope.launch {
                                            normX.animateTo(0f, spring(stiffness = 400f))
                                        }
                                        onXChange(0f)
                                    }
                                    if (yAutoCenter) {
                                        scope.launch {
                                            normY.animateTo(0f, spring(stiffness = 400f))
                                        }
                                        onYChange(valueFromNorm(0f, yRange))
                                    }
                                    break
                                }

                                // ── Finger moving ─────────────────────────
                                // Clamp touch point to circular boundary
                                val delta   = change.position - center
                                val dist    = delta.getDistance()
                                val clamped = if (dist > radius) delta / dist * radius else delta

                                val nx =  clamped.x / radius          // right = positive
                                val ny = -clamped.y / radius          // up    = positive (inverted)

                                scope.launch { normX.snapTo(nx) }
                                scope.launch { normY.snapTo(ny) }

                                onXChange(nx * 100f)
                                onYChange(valueFromNorm(ny, yRange))
                            }
                        }
                    }
            ) {
                Canvas(modifier = Modifier.fillMaxSize()) {
                    val c = Offset(size.width / 2f, size.height / 2f)
                    val r = minOf(size.width, size.height) / 2f * 0.82f
                    radius = r

                    // Outer ring
                    drawCircle(
                        color  = JoystickRing,
                        radius = r,
                        style  = Stroke(width = 2.dp.toPx())
                    )

                    // Crosshair
                    drawLine(JoystickCross, Offset(c.x - r, c.y), Offset(c.x + r, c.y), 1.dp.toPx())
                    drawLine(JoystickCross, Offset(c.x, c.y - r), Offset(c.x, c.y + r), 1.dp.toPx())

                    // Throttle progress bar (left side — only on non-auto-center Y)
                    if (!yAutoCenter) {
                        val fillFraction = (normY.value + 1f) / 2f   // 0 = bottom, 1 = top
                        val barH         = r * 2f * fillFraction
                        val barTop       = c.y + r - barH
                        drawRect(
                            color   = HudCyanDim.copy(alpha = 0.22f),
                            topLeft = Offset(c.x - r + 4.dp.toPx(), barTop),
                            size    = Size(8.dp.toPx(), barH)
                        )
                    }

                    // Thumb — glow ring + solid dot + centre pip
                    val tx = c.x + normX.value * r
                    val ty = c.y - normY.value * r    // re-invert for screen coords
                    val tp = Offset(tx, ty)

                    drawCircle(JoystickThumb.copy(alpha = 0.18f), radius = 30.dp.toPx(), center = tp)
                    drawCircle(JoystickThumb,                     radius = 14.dp.toPx(), center = tp)
                    drawCircle(HudBackground,                     radius =  5.dp.toPx(), center = tp)
                }
            }

            // Right axis label (X) - אותו ציר כמו משמאל, ברוחב זהה
            Text(
                text       = xLabel,
                fontFamily = FontFamily.Monospace,
                fontSize   = HudText.stickAxis,
                fontWeight = FontWeight.Bold,
                color      = HudTextDim,
                maxLines   = 1,
                softWrap   = false,
                textAlign  = TextAlign.End,
                modifier   = Modifier.width(36.dp)
            )
        }

        Spacer(Modifier.height(4.dp))

        // Bottom axis label (Y) - אותו ציר כמו למעלה
        Text(
            text       = yLabel,
            fontFamily = FontFamily.Monospace,
            fontSize   = HudText.stickAxis,
            fontWeight = FontWeight.Bold,
            color      = HudTextDim,
            maxLines   = 1,
            softWrap   = false
        )
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/** Converts a real value into a normalised [-1, 1] position within [range]. */
private fun normFromValue(value: Float, range: ClosedFloatingPointRange<Float>): Float {
    val mid  = (range.start + range.endInclusive) / 2f
    val half = (range.endInclusive - range.start) / 2f
    return if (half == 0f) 0f else (value - mid) / half
}

/** Converts a normalised [-1, 1] position back to a real value within [range]. */
private fun valueFromNorm(norm: Float, range: ClosedFloatingPointRange<Float>): Float {
    val mid  = (range.start + range.endInclusive) / 2f
    val half = (range.endInclusive - range.start) / 2f
    return (mid + norm * half).coerceIn(range.start, range.endInclusive)
}