package com.drone.gcs

import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.drone.gcs.ui.screens.MainControlScreen
import com.drone.gcs.ui.theme.DroneGCSTheme

class MainActivity : ComponentActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // המסך נשאר דלוק לאורך כל הטיסה
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // מצב מסך מלא — מסתיר את סרגלי המערכת
        enableEdgeToEdge()
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }

        setContent {
            DroneGCSTheme {
                MainControlScreen()
            }
        }
    }
}