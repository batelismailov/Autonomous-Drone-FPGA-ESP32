#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include "config.h"
#include "lidar.h"
#include "msp.h"
#include "batt.h"          // חייב להיות אחרי msp.h
#include "phase.h"
#include "fpga_comm.h"
#if MISSION_ENABLE
#include "mission.h"       // Mission FSM - כלול תמיד, רדום עד START מפורש (config.h)

// E1022P: האם משימה אוטונומית **פעילה בפועל** - רק אז נחסם המעבר המהיר ל-ANGLE בנגיעה (fastTd).
//   עד E1022O נחסם גם כשהמשימה נבחרה באפליקציה (MODE:WP) אבל כבר הסתיימה (MST=COMPLETE) והטייס נחת ידנית ב-POSHOLD:
//   ביומן 01.11 כך נחסמו 4 מתוך 4 הנחיתות אחרי משימה. עכשיו: ה-FSM ב-FPGA פעיל, או שמשימה נבחרה ואין מצב FPGA טרי.
//   (WP / RTH / FAILSAFE של INAV עצמו נחסמים בנפרד ב-phase.h.)
// E1022Q: גם כש-MST התיישן והמצב האחרון שה-FPGA דיווח היה פעיל - נשארים חסומים (מצב המשימה לא ידוע),
//   עד שמגיע MST טרי. MST טרי עם IDLE / COMPLETE / SUSPENDED משחרר את החסימה מעצמו (mission.stat.st מתעדכן).
//   statMs==0 = לא התקבל MST מעולם (למשל אחרי איתחול) - לא חוסם.
#if MISSION_ENABLE
static bool fastTdMissionBlock() {
    bool lastActiveStale = mission.statMs != 0 && !missionStatFresh() && missionStateActive(mission.stat.st);
    return missionFsmActive() || lastActiveStale || (missionEngaged() && !missionStatFresh());
}
#endif
#endif

// ═══════════════════════════════════════════════════════════
//   חלוקת האחריות
//     INAV  - ייצוב, אומדן גובה, ALTHOLD, POSHOLD, RTH, WP, LAND,
//             Failsafe וניתוק בנחיתה. הוא גם המחליט אילו מצבים פעילים.
//     FPGA  - בטיחות ובוררות: בניית זרם ה-RC והחלטה מתי להפסיק אותו
//     ESP32 - WiFi, תקשורת עם האפליקציה, קריאת טלמטריה מ-INAV, לידארים,
//             מכונת שלבי הטיסה, אכיפת רצפת המצערת בזמן אישור המראה,
//             וכיוון שעון בקר הטיסה מזמן הטאבלט
//     App   - ממשק, בקשות ולוגיקת המצערת הידנית
//
//   ⚠️ שדה MODE בשורת השליטה עובר מכאן ל-FPGA כמו שהוא. ה-ESP אינו מחליט
//      על מצבי טיסה ואינו חוסם אותם: ה-FPGA ממיר את הבקשה לערך AUX, ו-INAV
//      הוא שמחליט אם המצב מופעל. מה שפעיל בפועל חוזר לאפליקציה בשדה FMD.
//
//   ⚠️ הגובה של INAV (MSP_ALTITUDE) נקרא כאן לשימוש פנימי בלבד: בדיקת סבירות
//      של חיישן DWN, גיבוי לזיהוי המראה, ואישור החיישן הקדמי. הוא לא נשלח
//      ל-FPGA ולא לאפליקציה.
// ═══════════════════════════════════════════════════════════

WiFiUDP   udp;
IPAddress clientIP;
bool      clientKnown = false;

FCStatus   fc;
LidarData  lidar;
BattData   batt;
PhaseState flight;

unsigned long lastTelemMs  = 0;
unsigned long lastSensTxMs = 0;
unsigned long lastSlowTelemMs = 0;   // שדות הטלמטריה האיטיים (GPS מורחב + MAG)

// ── Debug דרך Type-C UART ──
// מחבר ה-UART משתמש בממיר USB-to-UART, לכן ה-ESP לא יכול לדעת פיזית
// אם המחשב/Serial Monitor מחוברים. לכן Debug כבוי אחרי כל Reset ומופעל ידנית:
//   D = ON, Q = OFF.  בזמן OFF אין printf/print של Debug.
bool debugEnabled = false;

// ═══ אבחון (DGN): חלון של שנייה ═══
//   loop - משך סבב הלולאה המרבי · gap - הפער המרבי בין שתי שורות שליטה שהועברו
//   ל-FPGA · req / fail - שאילתות MSP ל-Matek ואלה שלא נענו. הערכים של החלון
//   הקודם נשלחים בכל חבילת טלמטריה.
static unsigned long dgWinMs = 0, dgLoopStartUs = 0, dgLastCtrlFwdMs = 0;
static uint16_t dgLoopMax = 0, dgGapMax = 0;
static uint16_t dgOutLoop = 0, dgOutGap = 0, dgOutReq = 0, dgOutFail = 0;

static void diagLoopTick() {
    unsigned long nowUs = micros();
    if (dgLoopStartUs != 0) {
        unsigned long ms = (nowUs - dgLoopStartUs) / 1000UL;
        if (ms > dgLoopMax) dgLoopMax = (uint16_t)(ms > 65000 ? 65000 : ms);
    }
    dgLoopStartUs = nowUs;
    unsigned long now = millis();
    if (now - dgWinMs >= 1000) {
        dgWinMs  = now;
        dgOutLoop = dgLoopMax;  dgOutGap = dgGapMax;
        dgOutReq  = dgMspReq;   dgOutFail = dgMspFail;
        dgLoopMax = 0; dgGapMax = 0; dgMspReq = 0; dgMspFail = 0;
    }
}

static void diagCtrlForwarded() {
    unsigned long now = millis();
    if (dgLastCtrlFwdMs != 0) {
        unsigned long g = now - dgLastCtrlFwdMs;
        if (g > dgGapMax) dgGapMax = (uint16_t)(g > 65000 ? 65000 : g);
    }
    dgLastCtrlFwdMs = now;
}

static void updateDebugControl() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == 'D' || c == 'd') {
            if (!debugEnabled) {
                debugEnabled = true;
                Serial.println("[DBG] ON - send Q to disable");
            }
        } else if (c == 'S' || c == 's') {
            // אבחון קריאת הגדרות INAV - רק כשמנוטרל
            if (fc.armFresh && !fc.armed) {
                fcSettingProbe("nav_mc_althold_throttle");
                fcSettingProbe("thr_mid");
                fcSettingProbe("nav_wp_radius");
            } else Serial.println("[PROBE] skipped - FC armed or arm state unknown");
        } else if (c == 'Q' || c == 'q') {
            if (debugEnabled) Serial.println("[DBG] OFF");
            debugEnabled = false;
        }
    }
}

// ── לידאר ──
bool          lidarsReady     = false;
unsigned long lastLidarInitMs = 0;

// ── גובה בזמן החימוש - לבדיקת סבירות של DWN ולגיבוי זיהוי ההמראה ──
bool    prevFcArmed = false;
bool    altRefValid = false;
int32_t altAtArm    = 0;

// ── מצב האפליקציה ──
bool          appDescentIntent = false;   // ביט מפורש: מקל המצערת נדחף מטה
int           lastThrPct       = 0;       // ערך המצערת האחרון שהתקבל, באחוזים שלמים
// מה שנשלח בפועל ל-FPGA אחרי הרצפה, ביחידות של **חצי אחוז** (0..200, 131 = 65.5%),
// -1 = טרם התקבל. מדווח לאפליקציה ב-CTH.
int           lastSentThrHalf  = -1;
unsigned long lastCtrlMs       = 0;       // שורת שליטה אחרונה
unsigned long lastAppMs        = 0;       // כל חבילה מהאפליקציה, כולל APP_PING
bool          appSilentPrinted = false;
ThrottleTrace thrTrace;                   // שיא מצערת קצר - לרצפת ההמראה

// ── אישור השלב מהאפליקציה (שדה PHA בשורת השליטה) ──
//    האפליקציה מחזירה את השלב האחרון שהיא **קיבלה**. זו ההוכחה היחידה שהיא
//    כבר יודעת שאנחנו באוויר, ולכן היא התנאי לשחרור רצפת המצערת.
int           appPhaseAck = -1;
unsigned long lastAckMs   = 0;

// ── שעון בקר הטיסה, מהאפליקציה ──
//    נשמר עד שיש קישור ל-FC ומרווח פנוי מול שאילתות הסקר. הזמן שחלף מאז
//    שהחבילה התקבלה מתווסף לערך, כך שגם המתנה ארוכה לא מזיזה את השעון.
bool          rtcPending   = false;
uint32_t      rtcEpochS    = 0;
uint16_t      rtcMs        = 0;
unsigned long rtcRxMs      = 0;
unsigned long lastRtcSetMs = 0;

// ── חיות ה-FPGA ──
bool          fpgaAlive      = false;
unsigned long lastFpgaOkMs   = 0;
unsigned long lastFpgaPollMs = 0;
int8_t        prevFpgaState  = -2;     // להדפסה רק כשמשהו משתנה
char          prevFpgaEnd    = 0;

// ── אבחון (FC_DEBUG) - להדפסה רק כשמשהו משתנה ──
uint16_t      prevModeFlags  = 0xFFFF;
bool          prevModesFresh = false;
uint8_t       prevPhase      = 0xFF;
bool          prevFwdActive  = false;
int           prevThrFloor   = -2;
bool          prevAckWait    = false;

// ─── כיבוי הנורית המובנית ─────────────────────
void killBuiltinLed() {
#if defined(RGB_BUILTIN)
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
#else
    neopixelWrite(48, 0, 0, 0);   // ESP32-S3 DevKit - RGB על GPIO48
#endif
}

// ═══════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);
    killBuiltinLed();
    DBG_PRINTLN("\n[DRONE] Booting... (stage-1 firmware)");

    thrTraceClear(thrTrace);

    WiFi.mode(WIFI_AP);
    bool apOK = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
    bool bwOK = WiFi.softAPbandwidth(WIFI_BW_HT20);

    DBG_PRINTF("[WiFi] softAP()  -> %s\n", apOK ? "OK" : "FAILED");
    DBG_PRINT("[WiFi] SSID:     "); DBG_PRINTLN(WIFI_SSID);
    DBG_PRINT("[WiFi] AP IP:    "); DBG_PRINTLN(WiFi.softAPIP());
    DBG_PRINTF("[WiFi] Channel:  %d\n", WIFI_CHANNEL);
    DBG_PRINTF("[WiFi] BW:       %s\n", bwOK ? "20MHz" : "SET FAILED");

    udp.begin(UDP_PORT);
    DBG_PRINTF("[UDP]  Listening on port %d\n", UDP_PORT);

    Wire.begin(I2C_SDA, I2C_SCL);
    DBG_PRINTF("[LiDAR] budget %dms · DWN offset %dcm · FWD offset %dcm · mode from NVS\n",
                  LIDAR_TIMING_BUDGET_MS, LIDAR_DWN_MOUNT_CM, LIDAR_FWD_MOUNT_CM);
    lidarsReady = initLidars();

#if MISSION_ENABLE
    // הגדרות העקיפה מה-NVS, נבדקות מול הטווח של FRONT במצב השמור
    missionInit(lidarWantLong(chFwd) ? LIDAR_MAX_CM_LONG : LIDAR_MAX_CM_SHORT);
#endif

    initFPGA();                                     // GPIO17/18

    SerialFC.begin(BAUD_FC, SERIAL_8N1, UART_FC_RX, UART_FC_TX);   // GPIO4/5
    DBG_PRINTLN("[FC]   MSP UART ready (telemetry + clock)");

    DBG_PRINTLN("[DRONE] Ready to fly!\n");
}

// ═══════════════════════════════════════════════
void loop() {
    diagLoopTick();

    // בדיקת D/Q מה-Serial Monitor. כשה-Debug כבוי זו רק בדיקת UART קצרה.
    updateDebugControl();

    // ניסיון חוזר ללידאר - רק על הקרקע, initLidars חוסם ~100ms
    if (!lidarsReady && !fc.armed && millis() - lastLidarInitMs >= 3000) {
        lastLidarInitMs = millis();
        lidarsReady = initLidars();
    }

    handleUDP();

    // ── סקר MSP - שאילתה אחת בכל סבב, לפי חותמת הזמן של כל סוג ──
    updateFCFlags(fc);
#if MISSION_ENABLE
    missionPollsOn = missionEngaged();     // POSE / NAV רק כשמשימה נבחרה
#endif
    // E1021B: מיד אחרי מסגרת מרחק ל-INAV לא שולחים שאילתה (rfmsp.h, rfMspHold)
    const bool mspFree = !rfMspHold();
    int what = mspFree ? mspDue(fc) : -1;
    if (what >= 0) {
        mspNotePoll((uint8_t)what);
        if (what == POLL_ANALOG) pollBattery(batt);       // MSP2_INAV_ANALOG
        else                     pollFCOnce(fc, (uint8_t)what);
    }
    // BOXIDS / nav_mc_althold_throttle - רק כשעוד לא ידועים (התחברות או אתחול קישור)
    else if (mspFree && fcConfigDue(fc)) {
        lastAnyPollMs = millis();
        pollFCConfig(fc);
    }
#if MISSION_ENABLE
    // אימות משימת ה-WP והגדרות INAV ל-Mission FSM - שאילתה אחת, רק בסבב פנוי
    else if (mspFree && missionVerifyDue(fc)) {
        missionVerifyStep();
    }
#endif
    // שעון: פעולה נדירה, נכנסת רק כשאין שאילתה בשלה ויש מרווח פנוי
    else if (mspFree && rtcPending && fc.linkOK && millis() - lastAnyPollMs >= MSP_MIN_GAP_MS) {
        sendPendingRtc();
    }

#if FC_DEBUG
    // ── אבחון: מצבי הטיסה של INAV, מודפס רק כשמשהו משתנה ──
    if (debugEnabled && (fc.modeFlags != prevModeFlags || fc.modesFresh != prevModesFresh)) {
        prevModeFlags  = fc.modeFlags;
        prevModesFresh = fc.modesFresh;
        printInavModes(fc);
    }
#endif

    // ── נקודת ייחוס לגובה: הדגימה הטרייה הראשונה אחרי חימוש ──
    if (!fc.armed || !prevFcArmed) altRefValid = false;
    if (fc.armed && !altRefValid && fc.altFresh) {
        altAtArm    = fc.altCm;
        altRefValid = true;
    }
    // ניתוק מנועים מאפס גם את היסטוריית המצערת, כדי שערך מהטיסה הקודמת
    // לא ייחשב בטעות כשיא בהמראה הבאה
    if (!fc.armed && prevFcArmed) thrTraceClear(thrTrace);
    prevFcArmed = fc.armed;

    // כוונת ירידה פגה כשהשליטה נפסקת - אחרת ביט ישן היה נשאר תקוע
    if (millis() - lastCtrlMs > 500) appDescentIntent = false;

    // ── לידאר: דגימה בקצב של החיישן ──
    //    גובה גדול מהטווח של מצב ה-DWN ⇒ קריאת DWN "תקינה" אינה סבירה
    bool relAltValid = fc.armed && fc.altFresh && altRefValid;
    int32_t relAltCm = relAltValid ? (fc.altCm - altAtArm) : 0;
#if MISSION_ENABLE
    lidarDwnObstMode = missionDwnOverObstacle();   // E1022F: ראש המכשול לא נחסם לפי גובה INAV
#endif
    // E1022N: בנחיתה (כוונת ירידה) אליאס DOWN ≤ LIDAR_DWN_OBST_MAX_MM לא נחסם עד DWN_ALIAS_RELAX_ALT_CM
    lidarDwnRelaxAlias = appDescentIntent && relAltValid && relAltCm <= DWN_ALIAS_RELAX_ALT_CM;
    updateLidars(relAltValid && relAltCm > lidarDwnAliasCm());
    // שלב 0ב: הדגימה האחרונה של DOWN ל-INAV - כאן, כשאין שאילתת MSP פתוחה (rfmsp.h)
    rfMspService();

    // ── דגימת חיות FPGA כל 2 שניות ──
    if (millis() - lastFpgaPollMs >= FPGA_POLL_MS) {
        lastFpgaPollMs = millis();
        if (pingFPGA()) lastFpgaOkMs = millis();
        fpgaAlive = (lastFpgaOkMs != 0) && (millis() - lastFpgaOkMs < FPGA_STALE_MS);
        if (!fpgaAlive) { fpgaState = -1; fpgaFlags = '?'; fpgaEnd = '?'; fpgaVer = "-"; }

        if (debugEnabled && (fpgaState != prevFpgaState || fpgaEnd != prevFpgaEnd)) {
            DBG_PRINTF("[FPGA] state=%d flags=%c end=%c\n", fpgaState, fpgaFlags, fpgaEnd);
            prevFpgaState = fpgaState;
            prevFpgaEnd   = fpgaEnd;
        }
    }

    // ── תמונת חיישנים, שלב הטיסה, ושליחה ל-FPGA, 20Hz ──
    if (millis() - lastSensTxMs >= SENS_TO_FPGA_MS) {
        lastSensTxMs = millis();
        lidar = readLidars();

        // שיא המצערת בחלון הקצר האחרון, ואישור השלב מהאפליקציה.
        // אישור ישן אינו אישור: אם השליטה נפסקה, הערך לא ממשיך לתקף.
        int  thrPeak = thrTracePeak(thrTrace, millis(), PHASE_THR_PEAK_MS);
        bool ackFresh = (lastAckMs != 0) && (millis() - lastAckMs <= PHASE_ACK_FRESH_MS);
        bool appAirborneAck = ackFresh && (appPhaseAck == PH_AIRBORNE);

        bool fastTdOk = true;                        // E1022N: מעבר מהיר ל-ANGLE בנגיעה - לא בזמן משימה אוטונומית
#if MISSION_ENABLE
        fastTdOk = !fastTdMissionBlock();
#endif
        updatePhase(flight, fc, lidar, relAltValid, relAltCm, appDescentIntent,
                    lastThrPct, thrPeak, appAirborneAck, fastTdOk);

        // נעילת TOO_CLOSE של FRONT מהקרקע (דשא / קרקע מול החיישן) משתחררת בהמראה.
        // ראו lidarFwdReleaseGroundLatch ב-lidar.h. phase.h לא שונה.
        static uint8_t latchPrevPhase = PH_GROUND_DISARMED;
        if (flight.phase == PH_AIRBORNE && latchPrevPhase != PH_AIRBORNE) lidarFwdReleaseGroundLatch();
        latchPrevPhase = flight.phase;
        rfAirborne = (flight.phase == PH_AIRBORNE);      // E1022A: rfmsp.h
        sendSensorsToFPGA();

#if FC_DEBUG
        if (debugEnabled && (flight.phase != prevPhase || flight.fwdActive != prevFwdActive ||
            flight.thrFloor != prevThrFloor || flight.floorAckWait != prevAckWait)) {
            DBG_PRINTF("[PHASE] %s  fwd_active=%d  run=%u/%u  thr_floor=%d  peak=%d%s%s%s\n",
                          phaseName(flight.phase), flight.fwdActive ? 1 : 0,
                          (unsigned)flight.confirmRun, (unsigned)flight.groundRun,
                          flight.thrFloor, thrPeak,
                          flight.floorAckWait  ? "  (floor held - waiting for app PHA ack)" : "",
                          flight.floorAckByTmo ? "  (floor released by timeout - app did not ack)" : "",
                          flight.altFallbackUsed ? "  (alt fallback)" : "");
            prevPhase     = flight.phase;
            prevFwdActive = flight.fwdActive;
            prevThrFloor  = flight.thrFloor;
            prevAckWait   = flight.floorAckWait;
        }
#endif
    }

#if MISSION_ENABLE
    // ── Mission FSM: שורות INAV והמשימה ל-FPGA, CMD:MSTAT וקליטת תשובת MST ──
    missionLoop(fc, batt);
#endif

    // ── האפליקציה שתקה - לאבחון בלבד, ה-FPGA הוא זה שמטפל באובדן שליטה ──
    bool appSilent = (lastAppMs != 0) && (millis() - lastAppMs > APP_PING_STALE_MS);
    if (debugEnabled && appSilent != appSilentPrinted) {
        DBG_PRINTF("[APP] %s\n", appSilent ? "silent" : "back");
        appSilentPrinted = appSilent;
    }

    // ── טלמטריה לאפליקציה, 10Hz ──
    if (millis() - lastTelemMs >= TELEMETRY_INTERVAL) {
        lastTelemMs = millis();
        if (clientKnown) sendTelemetry();
    }
}

// ─── קריאת שדה מספרי משורת טקסט, -1 אם אינו קיים ─────
int parseField(const String& s, const char* key) {
    int i = s.indexOf(key);
    if (i < 0) return -1;
    return s.substring(i + strlen(key)).toInt();
}

// ─── מותר לשנות הגדרות: מנוטרל ועל הקרקע (INAV, שלב הטיסה וה-FPGA) ───
bool groundDisarmed() {
    return !fc.armed && flight.phase == PH_GROUND_DISARMED && fpgaState <= 1;
}

// ─── "LMODE:F:1" / "LMODE:D:0" - מצב המדידה של חיישן ───
//   תשובה: "LMODE:OK F:1" אחרי שהמצב אומת בחיישן ונשמר, או "LMODE:ERR:<סיבה> F"
void handleLmode(const String& pkt) {
    if (pkt.length() < 9 || pkt.charAt(7) != ':') { respond("LMODE:ERR:FORMAT"); return; }
    char w = pkt.charAt(6), m = pkt.charAt(8);
    if ((w != 'F' && w != 'D') || (m != '0' && m != '1')) { respond("LMODE:ERR:FORMAT"); return; }
    bool front = (w == 'F'), lng = (m == '1');
    String who = String(" ") + w;
    if (!groundDisarmed()) { respond("LMODE:ERR:ARMED" + who); return; }
#if MISSION_ENABLE
    if (missionFsmActive()) { respond("LMODE:ERR:ACTIVE" + who); return; }
    // FRONT ב-SHORT מקצר את הטווח - מרחקי הזיהוי וה"פנוי" חייבים להיכנס בו
    if (front && !lng && !missionSetFitsFront(LIDAR_MAX_CM_SHORT)) { respond("LMODE:ERR:SETTINGS" + who); return; }
#endif
    const char* e = lidarSetMode(front, lng);
    if (e) respond(String("LMODE:ERR:") + e + who);
    else   respond(String("LMODE:OK ") + w + ":" + m);
}

// ─── קבלת פקודה מהאפליקציה ────────────────────
// ═══════════════════════════════════════════════
//   פקודות מהאפליקציה
//
//   ⚠️ עדיפות לשליטה ולחירום: שאילתת MSP ל-Matek או המתנה לתשובת FPGA
//      חוסמות עד 30-150ms. בזמן ההמתנה ioIdleHook() קורא חבילות חדשות:
//      שורת שליטה (THR), DISARM, ABORT, MSTOP ו-PING מטופלות **מיד**; כל השאר נכנס
//      לתור קטן ומטופל בסבב הבא של handleUDP - באותו סדר שבו הגיע.
//      DISARM / ABORT / MSTOP מוחקים מהתור ARM ו-MSTART שהגיעו לפניהם, כדי שלא
//      יבוצעו אחריהם.
// ═══════════════════════════════════════════════
#define APP_Q_N 6
static String        appQ[APP_Q_N];
static uint8_t       appQHead = 0, appQCount = 0;
static bool          inAppHandler = false, inIdleHook = false;
static unsigned long lastHookUs = 0;

static String readAppPacket() {
    clientIP    = udp.remoteIP();
    clientKnown = true;
    lastAppMs   = millis();
    char buf[256] = {0};
    udp.read(buf, sizeof(buf) - 1);
    String pkt = String(buf); pkt.trim();
    return pkt;
}

static bool appFastPacket(const String& pkt) {
    return pkt.startsWith("THR:") || pkt == "CMD:DISARM" || pkt == "CMD:ABORT" ||
           pkt == "CMD:MSTOP" || pkt == "CMD:APP_PING";
}

// DISARM / ABORT: פקודות ARM ו-MSTART ישנות שבתור לא יבוצעו אחריהם
static void appQueuePurgeArming() {
    uint8_t n = appQCount, w = 0;
    String keep[APP_Q_N];
    for (uint8_t k = 0; k < n; k++) {
        const String& q = appQ[(appQHead + k) % APP_Q_N];
        if (q == "CMD:ARM" || q.startsWith("CMD:MSTART")) continue;
        keep[w++] = q;
    }
    for (uint8_t k = 0; k < w; k++) appQ[k] = keep[k];
    appQHead = 0; appQCount = w;
}

void processAppPacket(const String& pkt) {

    // פעימת לב של האפליקציה: מטופלת כאן ו**לא** מועברת ל-FPGA, כדי שלא
    // תיחשב שם כפקודת שליטה ותאפס את מונה אובדן הקשר.
    if (pkt == "CMD:APP_PING") { respond("APP:PONG"); return; }

    // שעון מהטאבלט: "RTC:<שניות UTC> MS:<מילישניות>". מטופל כאן בלבד -
    // זו הגדרה של בקר הטיסה ואין לה מה לחפש אצל ה-FPGA.
    if (pkt.startsWith("RTC:")) { noteRtcFromApp(pkt); return; }

    if (pkt == "CMD:ARM")        { fpgaSend("CMD:ARM");    lastFpgaPollMs = 0; return; }
    if (pkt == "CMD:DISARM")     { fpgaSend("CMD:DISARM"); noteDisarmCommanded(); lastFpgaPollMs = 0; return; }
    if (pkt == "CMD:ABORT")      { fpgaSend("CMD:ABORT");  noteDisarmCommanded(); lastFpgaPollMs = 0; return; }
    if (pkt == "CMD:SELF_CHECK") { doSystemStart();        return; }

    // מצב מדידה של לידאר - מטופל כאן בלבד
    if (pkt.startsWith("LMODE:")) { handleLmode(pkt); return; }
#if MISSION_ENABLE
    // הכנת המשימה / ביטול / הגדרות העקיפה - מטופלים כאן. MSTART / MSTOP עוברים ל-FPGA.
    if (pkt == "CMD:MPREP")      { missionHandlePrep(groundDisarmed()); return; }
    if (pkt == "CMD:MCANCEL")    { missionHandleCancel();               return; }
    if (pkt.startsWith("MSET:")) { missionHandleSet(pkt, groundDisarmed(), lidarFwdMaxCm()); return; }
#endif

    // ═══ שורת שליטה ═══
    //   נקראים ממנה ערך המצערת, כוונת הירידה ואישור השלב, ועליה נאכפת
    //   רצפת המצערת של שלב אישור ההמראה. האכיפה נעשית כאן ולא באפליקציה,
    //   משום שכאן שלב הטיסה ידוע מיידית ואינו תלוי בזמן החזרה של הטלמטריה.
    //
    //   ⚠️ שדה MODE אינו נוגע לכאן והוא עובר ל-FPGA כמו שהוא.
    //   ═══ מצערת בחצאי אחוז ═══
    //   THH - המצערת מהאפליקציה ביחידות של חצי אחוז (0..200). בלי השדה
    //   (אפליקציה ישנה) - THR באחוזים שלמים, כפול 2.
    //   מכונת השלבים, ספי ההמראה/הנחיתה וחישוב הרצפה ממשיכים לעבוד
    //   באחוזים שלמים בדיוק כמו קודם (thrH / 2). רק אכיפת הרצפה והערך
    //   שיוצא ל-FPGA (TH2) הם בחצאי אחוז.
    if (pkt.startsWith("THR:")) {
        lastCtrlMs = millis();
        int thr  = parseField(pkt, "THR:");
        int thrH = parseField(pkt, "THH:");
        if (thrH < 0 && thr >= 0) thrH = thr * 2;
        if (thrH > 200) thrH = 200;
        if (thrH >= 0) {
            thr = thrH / 2;
            lastThrPct = thr;
            thrTraceNote(thrTrace, thr, lastCtrlMs);   // לשיא הקצר
        }
        appDescentIntent = (parseField(pkt, "DSC:") == 1);
        // E1022A: בקשת SURFACE מהאפליקציה. ההחלטה ב-surface.h, והיא יוצאת ל-FPGA בשורת החיישנים.
        //   SRF לא עובר ל-FPGA בשורת השליטה: מוסר מהשורה (ה-FPGA מקבל רק את ההחלטה של ה-ESP).
        bool appPoshold = pkt.indexOf("MODE:POSHOLD") >= 0;
        surfNoteApp(parseField(pkt, "SRF:") == 1, appPoshold, parseField(pkt, "SNT:"));
#if MISSION_ENABLE
        missionNoteAppMode(pkt.indexOf("MODE:WP") >= 0);   // משימה נבחרה באפליקציה
#endif

        // PHA - השלב שהאפליקציה קיבלה לאחרונה. שדה אופציונלי: גרסה ישנה
        // של האפליקציה לא תשלח אותו, ואז הרצפה משתחררת בגיבוי הזמן.
        int ack = parseField(pkt, "PHA:");
        if (ack >= 0) { appPhaseAck = ack; lastAckMs = lastCtrlMs; }

        // THR השלם נאכף כמו קודם - בשביל FPGA בגרסה ישנה, שאינו מכיר TH2
        String out = applyThrottleFloor(pkt, flight.thrFloor);
        {   // E1022A: SRF של האפליקציה לא מגיע ל-FPGA (ה-FPGA מפענח SRF בכל שורה)
            int k = out.indexOf(" SRF:");
            if (k >= 0) {
                int e = out.indexOf(' ', k + 1);
                out = out.substring(0, k) + (e >= 0 ? out.substring(e) : String(""));
            }
        }

        // הערך הסופי בחצאי אחוז, אחרי הרצפה. נשלח ל-FPGA ב-TH2 בסוף השורה,
        // וה-FPGA נועל אותו כמצערת (הוא גובר על THR ועל THH).
        int sentH = thrH;
        if (flight.thrFloor >= 0 && thrH >= 0 && thrH < flight.thrFloor * 2)
            sentH = flight.thrFloor * 2;
        // E1022N: נגיעה מאומתת מהר (phase.h fastTd) במצב ניווט ⇒ ANGLE + סרק, כדי ש-INAV לא יוסיף דחף
        //   אחרי צניחת הברומטר ליד הקרקע (קפיצה). לא בזמן משימה אוטונומית. ה-FPGA מקבל את ANGLE מיד.
        bool fastTdOv = flight.fastTd;
#if MISSION_ENABLE
        if (fastTdMissionBlock()) fastTdOv = false;
#endif
        if (fastTdOv) {
            out   = phaseForceAngleIdle(out);
            sentH = 0;
        }
        if (sentH >= 0) out += " TH2:" + String(sentH);
#if FC_DEBUG
        if (sentH != thrH) {
            DBG_PRINTF("[PHASE] throttle floor applied: %d.%d -> %d\n",
                          thrH / 2, (thrH & 1) ? 5 : 0, flight.thrFloor);
        }
#endif
        // מה שנשלח בפועל - מדווח לאפליקציה בשדה CTH כדי שתוכל לחזור
        // לשליטה עם הערך הידוע ולא עם אפס
        lastSentThrHalf = sentH;

        fpgaSend(out);
        diagCtrlForwarded();
        return;
    }

    fpgaSend(pkt);
}

void handleUDP() {
    inAppHandler = true;
    while (appQCount > 0) {                       // קודם מה שנאסף בזמן ההמתנות
        String pkt = appQ[appQHead];
        appQHead = (appQHead + 1) % APP_Q_N;
        appQCount--;
        processAppPacket(pkt);
    }
    while (udp.parsePacket() > 0) processAppPacket(readAppPacket());
    inAppHandler = false;
}

// נקרא מתוך לולאות ההמתנה (MSP, FPGA) - לכל היותר פעם במילישנייה
void ioIdleHook() {
    if (inAppHandler || inIdleHook) return;
    unsigned long nowUs = micros();
    if (nowUs - lastHookUs < 1000) return;
    lastHookUs = nowUs;
    inIdleHook = true;
    while (appQCount < APP_Q_N && udp.parsePacket() > 0) {
        String pkt = readAppPacket();
        if (appFastPacket(pkt)) {
            if (pkt == "CMD:DISARM" || pkt == "CMD:ABORT" || pkt == "CMD:MSTOP") appQueuePurgeArming();
            processAppPacket(pkt);
            continue;
        }
        appQ[(appQHead + appQCount) % APP_Q_N] = pkt;
        appQCount++;
    }
    inIdleHook = false;
}

// ─── שעון: קליטה מהאפליקציה ───────────────────
//    רק בדיקת סבירות ושמירה. השליחה עצמה נעשית בלולאה, כדי שלא נתקע כאן
//    על ה-UART של בקר הטיסה בזמן טיפול בחבילה נכנסת.
void noteRtcFromApp(const String& pkt) {
    long s  = parseField(pkt, "RTC:");
    long ms = parseField(pkt, "MS:");

    if (s < (long)RTC_MIN_EPOCH_S || ms < 0 || ms > 999) {
        DBG_PRINTF("[RTC] ignored, not a sane time: %s\n", pkt.c_str());
        return;
    }
    // הגבלת קצב - השעון לא צריך יותר מעדכון אחד לכמה שניות
    if (lastRtcSetMs != 0 && millis() - lastRtcSetMs < RTC_MIN_SET_GAP_MS) return;

    rtcEpochS  = (uint32_t)s;
    rtcMs      = (uint16_t)ms;
    rtcRxMs    = millis();
    rtcPending = true;
}

// ─── שעון: כתיבה לבקר הטיסה ───────────────────
void sendPendingRtc() {
    unsigned long elapsed = millis() - rtcRxMs;        // מאז שהאפליקציה שלחה
    uint64_t totalMs = (uint64_t)rtcEpochS * 1000ULL + (uint64_t)rtcMs + (uint64_t)elapsed;
    uint32_t s  = (uint32_t)(totalMs / 1000ULL);
    uint16_t ms = (uint16_t)(totalMs % 1000ULL);

    bool ack     = setFcRtc(s, ms);
    rtcPending   = false;
    lastRtcSetMs = millis();

    DBG_PRINTF("[RTC] MSP_SET_RTC %lu.%03u -> %s\n",
                  (unsigned long)s, (unsigned)ms, ack ? "ack" : "no answer");
#if FC_DEBUG
    // קריאה חוזרת לאימות - רק על הקרקע, כדי לא להוסיף שאילתה בזמן טיסה
    if (debugEnabled && ack && !fc.armed) {
        uint32_t back = readFcRtc();
        if (back) DBG_PRINTF("[RTC] FC clock now reads %lu (delta %ld s)\n",
                                (unsigned long)back, (long)back - (long)s);
        else      DBG_PRINTLN("[RTC] read-back: no answer");
    }
#endif
}

// ─── SYSTEM START - פותח READY ב-FPGA ─────────
void doSystemStart() {
    DBG_PRINTLN("[SYS] System start requested");
    if (startFPGA()) {
        lastFpgaOkMs = millis();
        fpgaAlive    = true;
        respond("START:OK");
        DBG_PRINTF("[SYS] FPGA state = %d\n", fpgaState);
    } else {
        respond("START:ERR");
        DBG_PRINTLN("[SYS] FPGA did not answer");
    }
}

// ─── מצב הלידארים ושלב הטיסה ל-FPGA ───────────
// "FWD:70 FOK:1 DWN:0 DOK:4 FSEQ:17 DSEQ:203 PHS:3 FWA:1 MGH:1"
// FOK/DOK:   0=FAULT 1=VALID 2=NO_RETURN 3=UNCERTAIN 4=TOO_CLOSE
// FSEQ/DSEQ: מונה הדגימות השימושיות של כל חיישן (0..255, מתגלגל).
//            השורה נשלחת כל 50ms גם כשלא הגיעה דגימה חדשה (החיישן דוגם כל 55ms),
//            ולכן ה-FPGA סופר דגימה רק כשהמונה השתנה.
// PHS:       שלב הטיסה - 0 קרקע מנוטרל · 1 קרקע מחומש · 2 אישור המראה · 3 באוויר
// FWA:       1 = מותר להשתמש ב-FWD להחלטות מכשול (באוויר ובגובה מספיק)
// THS:       E1022C - מרכז המקל של האפליקציה ב-SURFACE (חצאי אחוז), ‎-1 = אין. ה-FPGA מחזיק אותו
//            ב-CH3 ביציאה (100ms לפני, ואחרי - עד שהאפליקציה מתיישרת).
//            E1022H: אחרי מגע מאומת (navTouch) ⇒ ‎-1. האפליקציה נעולה אז על 0% ולא תתיישר לעולם,
//            ולכן היציאה חייבת להיות מיידית (ה-FPGA: בלי THS תקין - בלי הקדמה ובלי החזקה), כך ש-CH3
//            נשאר 0% ו-INAV נכנס מחדש ל-ALTHOLD עם מצערת נמוכה על הקרקע (prepareForTakeoffOnReset).
//            INAV 9.1 SITL: עם החזקת THS הרחפן הקפיץ ל-115cm; עם ‎-1 נשאר על הקרקע, מנועים בסרק.
// SRF:       E1022A - 1 = לבקש SURFACE (CH8). ההחלטה ב-surface.h; ה-FPGA מוסיף שער משלו
//            (POSHOLD בלבד) ופולס כניסה מחדש. בשורה הזו ולא בשורת השליטה, כדי שאובדן DOWN
//            יגיע ל-FPGA תוך 50ms גם כשאין שורות מהאפליקציה.
// MGH:       בריאות המצפן - 0 לא מאומת · 1 תקין · 2 קפוא. ה-FPGA חוסם לפיו כניסה
//            ל-POSHOLD / RTH / WP, ועל 2 מעביר מצב ניווט פעיל ל-ALTHOLD.
//
// ⚠️ PHS ו-FWA נשלחים כבר עכשיו. cmd_parser מתעלם מהם; רק mission_rx מפענח
//    אותם, וגם הוא קיים ב-FPGA רק כש-MISSION_ENABLE=1.
// FRC / DRC: סיווג הדגימה ל-Mission FSM (lidar.h, MC_*)
// FLM / DLM: מצב המדידה שהוחל בפועל בחיישן - 0 SHORT · 1 LONG
void sendSensorsToFPGA() {
    // E1022C: ההחלטה על SURFACE (surface.h). מסירת המצערת ביציאה - ב-FPGA, לפי THS.
    uint8_t srf = surfTick(
#if MISSION_ENABLE
                           missionFsmActive(),
#else
                           false,
#endif
                           fc.rfPresent && fc.linkOK, fc.magHealth == 2);
    // E1022I: יציאה מ-SURFACE באמצע הוכחת מגע ⇒ navTouch מיד (phase.h). לפני THS של השורה הזו.
    static uint8_t srfPrevOut = 0;
    if (srfPrevOut == 1 && srf == 0) phaseSurfExit(flight, millis());
    srfPrevOut = srf;
    fpgaSend("FWD:" + String(lidar.fwd) + " FOK:" + String(lidar.fwdSt) +
             " DWN:" + String(lidar.dwn) + " DOK:" + String(lidar.dwnSt) +
             " FSEQ:" + String(lidar.fwdSeq) + " DSEQ:" + String(lidar.dwnSeq) +
             " PHS:" + String(flight.phase) + " FWA:" + (flight.fwdActive ? "1" : "0") +
             " MGH:" + String(fc.magHealth) +
             // E1022I: THS **לפני** SRF. cmd_parser קולט כל שדה ברגע שהוא מפוענח, ולכן ביציאה עם
             //   מגע מאומת ה-FPGA חייב לראות THS:-1 לפני SRF:0 (אחרת הוא מחזיק את ה-THS הישן).
             " THS:" + String(flight.navTouch ? -1 : surfThs()) + " SRF:" + String(srf)
#if MISSION_ENABLE
             + " FRC:" + String(lidar.fwdCls) + " DRC:" + String(lidar.dwnCls) +
             " FLM:" + (lidar.fwdLong ? "1" : "0") + " DLM:" + (lidar.dwnLong ? "1" : "0")
#endif
             );
}

// ─── חצאי אחוז לטקסט: 131 → "65.5", 130 → "65.0", -1 → "-1" ───
String halfPctStr(int h) {
    if (h < 0) return "-1";
    return String(h / 2) + ((h & 1) ? ".5" : ".0");
}

// ─── שדה סטטוס רכיבים ─────────────────────────
// 0=תקלה 1=תקין 2=לא ידוע
String buildSysStatus() {
    char s[8];
    s[0] = fpgaAlive   ? '1' : '0';                                    // FPGA
    s[1] = fc.linkOK   ? '1' : '0';                                    // FC link
    s[2] = fc.attFresh ? '1' : '0';                                    // IMU / attitude

    // סוללה - לפי הדיווח של iNav, לא לפי חישוב שלנו
    s[3] = !batt.fresh                       ? '2'
         : (batt.state == BATT_NOT_PRESENT)  ? '2'
         : (batt.state == BATT_OK)           ? '1' : '0';

    // GPS - תנאי למצבי POSHOLD / RTH / WP, ולא לחימוש בטיסה ידנית (ANGLE).
    s[4] = !fc.gpsFresh ? '2' : (fc.gpsFix ? '1' : '0');
    s[5] = (lidar.fwdSt == LID_FAULT) ? '0' : '1';                     // LiDAR FWD
    s[6] = (lidar.dwnSt == LID_FAULT) ? '0' : '1';                     // LiDAR DWN
    s[7] = '\0';
    return String(s);
}

// ─── טלמטריה לאפליקציה ────────────────────────
void sendTelemetry() {
    // מרחק: -1 רק כשהחיישן באמת מת. המשמעות המלאה מגיעה ב-FOK / DOK.
    int appFwd = (lidar.fwdSt == LID_FAULT) ? -1 : lidar.fwd;
    int appDwn = (lidar.dwnSt == LID_FAULT) ? -1 : lidar.dwn;

    String telem =
        "BTV:" + String(batt.vAvg)      +      // מתח, יחידות 0.01V
        " BAT:" + String(batt.pctAvg)   +      // אחוז - החישוב של iNav
        " BTS:" + String(batt.state)    +      // 0=OK 1=WARN 2=CRIT 3=NONE
        " BTC:" + String(batt.cells)    +      // מספר תאים
        " ARM:" + (fc.armed ? "1":"0")  +
        " FWD:" + String(appFwd)        +
        " FOK:" + String(lidar.fwdSt)   +
        " DWN:" + String(appDwn)        +
        " DOK:" + String(lidar.dwnSt)   +
        " FLM:" + (lidar.fwdLong ? "1":"0") +   // מצב המדידה שהוחל: 0 SHORT · 1 LONG
        " DLM:" + (lidar.dwnLong ? "1":"0") +
        " FMX:" + String(lidar.fwdMax)  +      // הטווח המרבי במצב הנוכחי, cm
        " DMX:" + String(lidar.dwnMax)  +
        " PHS:" + String(flight.phase)  +      // שלב הטיסה - נקבע כאן, לא באפליקציה
        " FWA:" + (flight.fwdActive ? "1":"0") +
        " CTH:" + halfPctStr(lastSentThrHalf) + // המצערת שנשלחה בפועל ל-FPGA (65.5), -1 = טרם התקבלה
        " FMD:" + String((unsigned int)fc.modeFlags, HEX) +   // מצבי טיסה בפועל (ביטי FM_*)
        " FMV:" + (fc.modesFresh ? "1":"0") +                  // המיפוי ידוע והסטטוס טרי
        " SRS:" + String(srfState) + "," + String(srfLosses) + "," + String(srfExits) + "," + String(surfRelPct()) + "," + String(flight.navTouch ? 1 : 0) + "," + String(flight.fastTd ? 1 : 0) +  // SURFACE (surface.h): מצב, אובדני DOWN, יציאות אוטומטיות, אמינות %, מגע מאומת (E1022G)
        " ATM:" + String((int)fc.altHoldThr) +                 // nav_mc_althold_throttle, -1 לא ידוע
        " MGH:" + String(fc.magHealth) +       // בריאות המצפן: 0 לא מאומת · 1 תקין · 2 קפוא
        " FCL:" + (fc.linkOK ? "1":"0") +
        " FSW:" + (fc.disarmSuspect ? "1":"0") +
        " FST:" + String(fpgaState)     +      // מצב ה-FSM
        " FDB:" + String(fpgaFlags) + String(fpgaEnd) +   // אבחון FPGA: דגלים + סיבת עצירה אחרונה
        " SYS:" + buildSysStatus() +
        " FWV:" FW_VERSION +
        " FPV:" + fpgaVer +                    // גרסת ה-FPGA (מתשובת STATUS), "-" = לא ידועה
        " DGN:" + String(dgOutLoop) + "," + String(dgOutGap) + "," + String(dgOutReq) + "," + String(dgOutFail);

    // שלב 0א: כל קריאות ה-DOWN מאז החבילה הקודמת + עוגן זמן ה-ESP (dwnlog.h).
    // אבחון פסיבי בלבד - האפליקציה רק שומרת את החבילה במקליט.
    telem += dwnLogTake(millis());
    telem += rfMspTelem();                    // שלב 0ב: RFM:sent,overwritten,noRoom,unsent

    // ── חבילת התצוגה: ATTITUDE, כרטיס ה-GPS כולו ומגנטומטר ──
    //    מצורפים רק לחבילה אחת בכל SLOW_TELEM_MS (5Hz), ולא לכל חבילת 10Hz.
    //    ROLL, PITCH, Heading, MAG X/Y/Z ו-Fix, LAT, LON, SATS, HDOP, EPH, EPV
    //    יוצאים יחד באותה חבילה, כך שהתצוגה מתעדכנת כיחידה אחת כל 200ms.
    //    האפליקציה שומרת את הערך האחרון בין החבילות. -1 / MGV:0 = לא ידוע.
    if (millis() - lastSlowTelemMs >= SLOW_TELEM_MS) {
        lastSlowTelemMs = millis();
        String gps = fc.gpsFix
            ? "GPS:3D LAT:" + String(fc.lat, 6) + " LON:" + String(fc.lon, 6)
            : "GPS:NO_FIX LAT:0 LON:0";
        telem += " ATR:" + String(fc.attRoll, 1) +   // מעלות, עשירית מעלה
                 " ATP:" + String(fc.attPitch, 1) +
                 " HDG:" + String(fc.heading) +
                 " " + gps +
                 " SAT:" + String(fc.gpsSats) +
                 " HDP:" + String(fc.gpsHdop) +          // HDOP × 100
                 " EPH:" + String(fc.gpsEph)  +          // cm
                 " EPV:" + String(fc.gpsEpv)  +          // cm
                 " MGV:" + (fc.magFresh ? "1" : "0") +
                 " MGX:" + String(fc.magX) +
                 " MGY:" + String(fc.magY) +
                 " MGZ:" + String(fc.magZ);
    }

#if MISSION_ENABLE
    // ── שדות ה-Mission FSM (mission.h) ──
    //    משימה נבחרה / פעילה - בכל חבילה. אחרת פעם ב-5 חבילות (2Hz), למצב בלבד.
    //    האפליקציה שומרת את הערך הקודם בחבילה בלי MSV.
    static uint8_t misTelN = 0;
    if (missionEngaged() || (++misTelN % 5) == 0) telem += missionTelemetry(lidar);
#endif

    udp.beginPacket(clientIP, UDP_PORT);
    udp.print(telem);
    udp.endPacket();
}

void respond(const String& msg) {
    if (!clientKnown) return;
    udp.beginPacket(clientIP, UDP_PORT);
    udp.print(msg);
    udp.endPacket();
}
