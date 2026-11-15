package com.drone.gcs.ui.components

import androidx.compose.animation.core.Animatable
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChange
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.gcs.ui.theme.*
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

// אזור התחתית שמתחת לריבוע המקל (קריאה חיה + HOVER) - גובה קבוע ומשותף לשני
// המקלות, כדי שמרכזי אזורי התנועה של המקל השמאלי והימני ייפלו על אותו Y.
val STICK_FOOTER_GAP    = 8.dp
val STICK_FOOTER_HEIGHT = 96.dp

private const val AXIS_NONE = 0
private const val AXIS_YAW  = 1   // אופקי
private const val AXIS_THR  = 2   // אנכי

/**
 * מקל שמאלי - מצערת (אנכי) + סבסוב (אופקי), בצורת פלוס.
 *
 * נעילת ציר: בתחילת כל מגע נקבע הציר הדומיננטי, ומאותו רגע
 * הציר השני מאולץ לאפס עד השחרור. אי אפשר לקבל סבסוב בטעות
 * תוך כדי שינוי מצערת, ולהפך.
 *
 * מתחת למקל: במצב ידני - כפתורי +/- של מרכז המצערת (HOVER).
 * כש-INAV במצב שבו המצערת היא פקודת גובה/ניווט (navThrottle) -
 * ה-+/- מוסתרים, ומוצג הנייטרל שנשלח כשהמקל במרכז.
 */
@Composable
fun ThrottleYawJoystick(
    throttlePercent: Float,
    hoverPercent: Float,
    latched: Boolean,
    yawValue: Float,
    forceCenter: Boolean,
    onStickChange: (yNorm: Float, yaw: Float) -> Unit,
    onTrimHover: (Float) -> Unit,
    modifier: Modifier = Modifier,
    navThrottle: Boolean = false,
    navNeutral: Float = 0f,
    navLabel: String = "ALTHOLD",
) {
    val scope = rememberCoroutineScope()
    val normX = remember { Animatable(0f) }    // -1..1 → YAW
    val normY = remember { Animatable(0f) }    // -1..1 → THR

    // איזה ציר נעול כרגע - לתצוגה בלבד
    var lockedAxis by remember { mutableIntStateOf(AXIS_NONE) }

    // LINK LOST / DISARM → החזרה למרכז
    LaunchedEffect(forceCenter) {
        if (forceCenter) {
            normX.snapTo(0f)
            normY.snapTo(0f)
            lockedAxis = AXIS_NONE
            onStickChange(0f, 0f)
        }
    }

    Column(
        modifier = modifier,
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(STICK_FOOTER_GAP)
    ) {
        BoxWithConstraints(modifier = Modifier.fillMaxWidth().aspectRatio(1f)) {
            val sizePx = with(LocalDensity.current) { maxWidth.toPx() }
            val radius = sizePx / 2f
            val thumbR = radius * 0.22f
            val travel = radius - thumbR
            val armHalf = thumbR * 1.30f          // חצי רוחב זרוע הפלוס
            val lockZone = travel * 0.14f         // אזור מת לפני החלטת ציר

            Canvas(
                modifier = Modifier
                    .fillMaxSize()
                    .pointerInput(Unit) {
                        awaitEachGesture {
                            val down = awaitFirstDown()

                            // חובה "לתפוס" את הידית - נגיעה מרוחקת לא מזיזה כלום
                            val gx = down.position.x - radius
                            val gy = down.position.y - radius
                            if (hypot(gx, gy) > travel * 0.45f) return@awaitEachGesture

                            var axis = AXIS_NONE
                            lockedAxis = AXIS_NONE

                            fun report(px: Float, py: Float) {
                                val rx = px - radius
                                val ry = py - radius

                                // קביעת הציר פעם אחת, אחרי שיצאנו מהאזור המת
                                if (axis == AXIS_NONE && hypot(rx, ry) > lockZone) {
                                    axis = if (abs(rx) > abs(ry)) AXIS_YAW else AXIS_THR
                                    lockedAxis = axis
                                }

                                val dx = if (axis == AXIS_YAW) (rx / travel).coerceIn(-1f, 1f) else 0f
                                val dy = if (axis == AXIS_THR) (ry / travel).coerceIn(-1f, 1f) else 0f
                                val yUp = -dy                // מסך למטה = ערך שלילי

                                scope.launch { normX.snapTo(dx); normY.snapTo(yUp) }
                                onStickChange(yUp, dx * 100f)
                            }
                            report(down.position.x, down.position.y)

                            var pos = down.position
                            while (true) {
                                val event  = awaitPointerEvent()
                                val change = event.changes.firstOrNull() ?: break
                                if (!change.pressed) break
                                pos += change.positionChange()
                                report(pos.x, pos.y)
                                change.consume()
                            }

                            // שחרור - חזרה למרכז בשני הצירים
                            lockedAxis = AXIS_NONE
                            scope.launch { normX.animateTo(0f) }
                            scope.launch { normY.animateTo(0f) }
                            onStickChange(0f, 0f)
                        }
                    }
            ) {
                val c = Offset(radius, radius)
                val armLen = travel + thumbR

                // ── מסלול הפלוס ──
                val idle   = HudCard
                val hot    = HudCyan.copy(alpha = 0.16f)
                val vFill  = if (lockedAxis == AXIS_THR) hot else idle
                val hFill  = if (lockedAxis == AXIS_YAW) hot else idle

                // זרוע אנכית - מצערת
                drawRoundRect(
                    color = vFill,
                    topLeft = Offset(radius - armHalf, radius - armLen),
                    size = Size(armHalf * 2f, armLen * 2f),
                    cornerRadius = CornerRadius(armHalf, armHalf)
                )
                // זרוע אופקית - סבסוב
                drawRoundRect(
                    color = hFill,
                    topLeft = Offset(radius - armLen, radius - armHalf),
                    size = Size(armLen * 2f, armHalf * 2f),
                    cornerRadius = CornerRadius(armHalf, armHalf)
                )
                drawRoundRect(
                    color = HudBorder,
                    topLeft = Offset(radius - armHalf, radius - armLen),
                    size = Size(armHalf * 2f, armLen * 2f),
                    cornerRadius = CornerRadius(armHalf, armHalf),
                    style = Stroke(width = 2f)
                )
                drawRoundRect(
                    color = HudBorder,
                    topLeft = Offset(radius - armLen, radius - armHalf),
                    size = Size(armLen * 2f, armHalf * 2f),
                    cornerRadius = CornerRadius(armHalf, armHalf),
                    style = Stroke(width = 2f)
                )

                // קווי מרכז
                drawLine(HudBorder, Offset(radius, radius - travel),
                    Offset(radius, radius + travel), 1.5f)
                drawLine(HudBorder, Offset(radius - travel, radius),
                    Offset(radius + travel, radius), 1.5f)

                // טבעת "מרכז = ריחוף / שמור גובה" - נראית רק כשהנעילה פתוחה
                if (latched) {
                    val ring = if (navThrottle) HudCyan else HudGreen
                    drawCircle(ring.copy(alpha = 0.5f), thumbR * 1.45f, c,
                        style = Stroke(width = 2f))
                }

                val tx = radius + normX.value * travel
                val ty = radius - normY.value * travel
                val thumbColor = if (latched) HudGreen else HudCyan
                drawCircle(thumbColor.copy(alpha = 0.22f), thumbR, Offset(tx, ty))
                drawCircle(thumbColor, thumbR, Offset(tx, ty), style = Stroke(width = 3f))
            }

            StickAxisLabel("THR ▲", Alignment.TopCenter)
            StickAxisLabel("▼ THR", Alignment.BottomCenter)
            StickAxisLabel("◀ YAW", Alignment.CenterStart)
            StickAxisLabel("YAW ▶", Alignment.CenterEnd)
        }

        // ── אזור התחתית: גובה קבוע STICK_FOOTER_HEIGHT, התוכן מיושר למעלה ──
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .height(STICK_FOOTER_HEIGHT)
                .wrapContentHeight(Alignment.Top, unbounded = true),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // ── קריאה חיה ──
            Row(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(HudCard)
                    .border(1.dp, HudBorder, RoundedCornerShape(4.dp))
                    .padding(horizontal = 12.dp, vertical = 5.dp),
                horizontalArrangement = Arrangement.spacedBy(14.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                StickReadout("THR", "%.1f%%".format(throttlePercent),
                    if (latched) HudGreen else HudCyan)
                StickReadout("YAW", yawValue.roundToInt().toString(), HudCyan)
            }

            // ── מרכז המצערת: +/- במצב ידני, נייטרל קבוע במצב ניווט ──
            //    שני המצבים באותו גובה, כדי שהמסך לא יקפוץ במעבר
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                if (!navThrottle) {
                    HoverTrimButton("−") { onTrimHover(-0.5f) }
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("HOVER", fontFamily = FontFamily.Monospace,
                            fontSize = HudText.stickAxis, letterSpacing = 1.5.sp,
                            color = HudTextDim, maxLines = 1, softWrap = false)
                        Text("%.1f%%".format(hoverPercent), fontFamily = FontFamily.Monospace,
                            fontSize = HudText.stickValue, fontWeight = FontWeight.Bold,
                            color = HudAmber, maxLines = 1, softWrap = false)
                    }
                    HoverTrimButton("+") { onTrimHover(0.5f) }
                } else {
                    Spacer(Modifier.size(38.dp))
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text(navLabel, fontFamily = FontFamily.Monospace,
                            fontSize = HudText.stickAxis, letterSpacing = 1.5.sp,
                            color = HudTextDim, maxLines = 1, softWrap = false)
                        Text("%.1f%%".format(navNeutral), fontFamily = FontFamily.Monospace,
                            fontSize = HudText.stickValue, fontWeight = FontWeight.Bold,
                            color = HudCyan, maxLines = 1, softWrap = false)
                    }
                    Spacer(Modifier.size(38.dp))
                }
            }
        }
    }
}

// ── רכיבי עזר פרטיים לקובץ הזה ─────────────────────────────────────

// גדלי הטקסט נלקחים מ-HudText, כדי שהגדלה תהיה אחידה בכל המסך
@Composable
private fun BoxScope.StickAxisLabel(text: String, align: Alignment) {
    Text(
        text = text,
        fontFamily = FontFamily.Monospace,
        fontSize = HudText.stickAxis,
        letterSpacing = 1.sp,
        color = HudTextDim,
        maxLines = 1,
        softWrap = false,
        modifier = Modifier.align(align).padding(4.dp)
    )
}

@Composable
private fun StickReadout(label: String, value: String, color: Color) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(5.dp)
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = HudText.stickAxis,
            color = HudTextDim, maxLines = 1, softWrap = false)
        Text(value, fontFamily = FontFamily.Monospace, fontSize = HudText.stickValue,
            fontWeight = FontWeight.Bold, color = color, maxLines = 1, softWrap = false)
    }
}

/** לחיצה = צעד אחד. לחיצה ארוכה = חזרה מהירה. */
@Composable
private fun HoverTrimButton(label: String, onStep: () -> Unit) {
    val interaction = remember { MutableInteractionSource() }
    val pressed by interaction.collectIsPressedAsState()

    LaunchedEffect(pressed) {
        if (pressed) {
            onStep()
            delay(450)
            while (true) { onStep(); delay(80) }
        }
    }

    Box(
        contentAlignment = Alignment.Center,
        modifier = Modifier
            .size(38.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(HudAmber.copy(alpha = 0.12f))
            .border(1.dp, HudAmber.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
            .clickable(interactionSource = interaction, indication = null) { }
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = 20.sp,
            fontWeight = FontWeight.Bold, color = HudAmber)
    }
}
