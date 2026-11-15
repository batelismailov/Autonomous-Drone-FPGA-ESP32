package com.drone.gcs.network

import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketTimeoutException

private const val TAG         = "UdpManager"
private const val TARGET_IP   = "192.168.4.1"
private const val TARGET_PORT = 4210
private const val LISTEN_PORT = 4210
private const val SOCKET_TIMEOUT_MS = 600
private const val ERR_REPEAT_MS     = 5000L   // סיכום כל 5 שניות

class UdpManager {

    private val _incoming = MutableSharedFlow<String>(extraBufferCapacity = 128)
    val incoming: SharedFlow<String> = _incoming

    // מקליט הטיסה: כל חבילה שיוצאת (נקרא לפני השליחה, מכל מקום שמשדר)
    @Volatile var onTx: ((String) -> Unit)? = null

    private var socket: DatagramSocket? = null
    private var receiveJob: Job? = null

    private var sendCount    = 0L
    private var errCount     = 0L
    private var lastErrMsg   = ""
    private var lastErrLogMs = 0L

    // ── לוג שגיאות מווסת: trace מלא פעם אחת לכל סוג שגיאה,
    //    ואז שורה אחת מסכמת כל 5 שניות ──
    private fun logError(where: String, e: Throwable) {
        errCount++
        val msg = e.cause?.message ?: e.message ?: e.toString()
        val now = System.currentTimeMillis()

        if (msg != lastErrMsg) {
            Log.w(TAG, "$where FAILED (#$errCount) — $msg", e)   // סוג חדש → מלא
            lastErrMsg   = msg
            lastErrLogMs = now
        } else if (now - lastErrLogMs >= ERR_REPEAT_MS) {
            Log.w(TAG, "$where still failing — $errCount total — $msg")
            lastErrLogMs = now
        }
    }

    fun startReceiving(scope: CoroutineScope) {
        receiveJob?.cancel()
        receiveJob = scope.launch(Dispatchers.IO) {
            runCatching {
                val s = DatagramSocket(LISTEN_PORT).also {
                    it.soTimeout = SOCKET_TIMEOUT_MS
                    socket = it
                }
                Log.i(TAG, "socket bound to port $LISTEN_PORT")

                val buf = ByteArray(1500)          // טלמטריה מלאה עם שדות המשימה: עד ~800 תווים
                val pkt = DatagramPacket(buf, buf.size)

                while (isActive) {
                    pkt.setLength(buf.size)          // חובה לפני כל receive
                    runCatching { s.receive(pkt) }
                        .onSuccess {
                            val msg = String(pkt.data, 0, pkt.length).trim()
                            if (msg.isNotEmpty()) {
                                Log.d(TAG, "RX ${pkt.address}: $msg")
                                _incoming.tryEmit(msg)
                            }
                        }
                        .onFailure { e ->
                            if (e !is SocketTimeoutException) logError("receive", e)
                        }
                }
            }.onFailure { e ->
                Log.e(TAG, "socket bind FAILED on port $LISTEN_PORT", e)
            }
        }
    }

    suspend fun send(message: String) = withContext(Dispatchers.IO) {
        onTx?.invoke(message)
        runCatching {
            val bytes   = message.toByteArray(Charsets.US_ASCII)
            val address = InetAddress.getByName(TARGET_IP)
            val pkt     = DatagramPacket(bytes, bytes.size, address, TARGET_PORT)
            val s = socket ?: DatagramSocket()
            s.send(pkt)
            if (++sendCount % 40L == 0L) Log.i(TAG, "TX ok #$sendCount -> $TARGET_IP")
        }.onFailure { e ->
            logError("send", e)
        }
    }

    fun close() {
        receiveJob?.cancel()
        socket?.close()
        socket = null
    }
}