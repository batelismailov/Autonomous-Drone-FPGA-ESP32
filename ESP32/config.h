#pragma once

// ═══════════════════════════════════════════════
//   ESP32-S3 - מיפוי פינים סופי
// ═══════════════════════════════════════════════
#define UART_FPGA_TX   17    // ESP TX → FPGA PIN_43
#define UART_FPGA_RX   18    // ESP RX ← FPGA PIN_42
#define BAUD_FPGA      115200

#define UART_FC_TX     4     // ESP TX → Matek RX1  (טלמטריה בלבד)
#define UART_FC_RX     5     // ESP RX ← Matek TX1
#define BAUD_FC        115200

#define I2C_SDA        8
#define I2C_SCL        9

#define XSHUT_DWN      10    // חיישן תחתון
#define XSHUT_FWD      11    // חיישן מכשולים

// ═══════════════════════════════════════════════
//   כתובות VL53L1X - שתי מוסכמות, לא לבלבל!
//     8-bit → setI2CAddress()     (הספרייה עושה >>1)
//     7-bit → Wire.beginTransmission()
// ═══════════════════════════════════════════════
#define ADDR_FWD_8B    0x60      // → 7-bit 0x30
#define ADDR_DWN_8B    0x62      // → 7-bit 0x31
#define ADDR_FWD_7B    0x30
#define ADDR_DWN_7B    0x31

// ═══════════════════════════════════════════════
//   התקנה פיזית של הלידארים
//
//   הקריאה הגולמית היא מהחיישן. אנחנו מקזזים אותה כך
//   שהמספר המוצג הוא המרווח מ**הנקודה שתפגע ראשונה**:
//
//     FWD  →  קצה דיסקת הפרופלור הקדמי
//     DWN  →  תחתית הרחפן (אין רגליים)
//
//   כלומר 0 פירושו נגיעה, לא "קרוב מאוד".
//   ⚠️ ספים של מכשול נמדדים מקצה הפרופלור - לא להוסיף שוב 18cm.
// ═══════════════════════════════════════════════
#define LIDAR_FWD_MOUNT_CM  18    // מקצה הפרופלור עד החיישן
#define LIDAR_DWN_MOUNT_CM   1    // החיישן כ-1cm מעל הרצפה כשהרחפן עומד - מתחת לטווח המינימלי

// כיול שגיאת חיישן - למלא אחרי מדידה מול סרגל
#define LIDAR_FWD_BIAS_CM    0
#define LIDAR_DWN_BIAS_CM    0

// ═══════════════════════════════════════════════
//   VL53L1X - מצב מדידה (SHORT / LONG) לכל חיישן, ועיבוד המדידות
//
//   המצב נבחר מהאפליקציה (LMODE), רק כשמנוטרל ועל הקרקע. ה-ESP מחיל אותו
//   בחיישן, קורא בחזרה את המצב ואת ה-timing budget, ושומר ב-NVS. אחרי הדלקה
//   המצב נטען מה-NVS ומוחל - האפליקציה רק מציגה את מה שהוחל בפועל.
//   ברירת מחדל: FRONT LONG · DOWN SHORT.
//     SHORT - עד ~1.3m, עמיד לאור שמש.
//     LONG  - עד ~4m בתאורה פנימית, פחות באור שמש חזק.
//   אותו timing budget בשני המצבים (50ms - מותר גם ב-LONG), ולכן אותו קצב
//   דגימות, אותם מסננים ואותם ספי קפיצה. משתנה רק הטווח המרבי.
// ═══════════════════════════════════════════════
#define LIDAR_TIMING_BUDGET_MS  50    // SHORT: 15/20/33/50/100/200/500 · LONG: 20/33/50/100/200/500
#define LIDAR_IMP_MS            55    // מרווח בין מדידות - חייב להיות ≥ timing budget
#define LIDAR_POLL_MS           20    // כל כמה זמן בודקים אם הגיעה דגימה חדשה
#define LIDAR_STALE_MS          300   // בלי דגימה חדשה ⇒ SENSOR_FAULT
#define LIDAR_MODE_MIN_GAP_MS   1000  // מרווח מינימלי בין שתי החלפות SHORT/LONG באותו חיישן
#define LIDAR_MAX_CM_SHORT      130   // מצב SHORT: מעבר לזה החיישן לא רואה
#define LIDAR_MAX_CM_LONG       400   // מצב LONG
#define LIDAR_FWD_LONG_DEFAULT  1     // FRONT: LONG
#define LIDAR_DWN_LONG_DEFAULT  0     // DOWN: SHORT

#define LIDAR_MIN_VALID_MM      40    // status 0 אבל קרוב מזה ⇒ TOO_CLOSE (מתחת לטווח המינימלי)
#define LIDAR_JUMP_MM           100   // קפיצה גדולה מזה צריכה אישור של קריאה שנייה עקבית
#define LIDAR_JUMP_PCT          15    // ...או 15% מהמרחק האחרון, הגדול מביניהם
#define LIDAR_HISTORY_MS        300   // קריאה ישנה מזה לא משמשת להשוואה
#define LIDAR_NEAR_MM           100   // היסטוריה: קריאה תקינה אחרונה קרובה מזה...
#define LIDAR_PROX_CHAIN_N      3     // E1022K: כמה קפיצות יורדות שנדחו ברצף נחשבות הוכחת קרבה (lastInRangeMm)
#define LIDAR_TREND_MM          10    // ...ומרחק שקטן לפחות בזה ⇒ Invalid שאחריה = TOO_CLOSE

// ═══ DOWN: קבלת TOO_CLOSE (E1021C) ═══
//   נמצא בטיסה (21.10 21:58, ‏2.1m): דגימה בודדת raw=0mm status=0 בזמן שה-DOWN מחוץ לטווח
//   ⇒ TOO_CLOSE ⇒ נעילה שלא השתחררה ~56 שנ' (כל הדגימות אחריה status 4).
//   ב-DOWN בלבד, ורק כשעוד אין נעילה, דגימת TOO_CLOSE (status 0 + raw < LIDAR_MIN_VALID_MM, או status 3)
//   מתקבלת כש:
//     · INAV לא מראה גובה מעל טווח החיישן (dwnAltImplausible) - אחרת UNCERTAIN, בלי נעילה.
//       ליד הקרקע הברומטר מראה **נמוך** מהאמת (נמדד: עד ‎-5m), ולכן זה לא חוסם מגע אמיתי.
//     · ויש הקשר של התקרבות: מדידה תקינה אחרונה עד LIDAR_TC_CTX_MM, לפני עד LIDAR_TC_CTX_MS,
//       או LIDAR_TC_CONFIRM_N דגימות כאלה ברצף (על הקרקע החיישן מחזיר 0-3mm כל הזמן).
#define LIDAR_TC_CONFIRM_N      3
#define LIDAR_TC_CTX_MM         400
#define LIDAR_TC_CTX_MS         600

// גובה (מה-Matek) שמעליו קריאת DWN "תקינה" לא אפשרית: הטווח של המצב + מרווח לנדידת הברומטר
#define DWN_ALIAS_ALT_SHORT_CM  180
#define DWN_ALIAS_ALT_LONG_CM   450
// E1022F: מעל מכשול (Mission CROSS / REAR_EDGE) VALID של DOWN מתקבל למרות הגובה - רק עד כאן
//   (25.10: קיפול של SHORT בגובה 2.5m נתן 1361-1386mm; ראש הקרטון נמדד ב-344-716mm)
#define LIDAR_DWN_OBST_MAX_MM   1000

// Debug דרך מחבר ה-Type-C UART:
//   ברירת מחדל אחרי כל אתחול = כבוי לחלוטין.
//   שליחת D ב-Serial Monitor מפעילה הדפסות בזמן ריצה.
//   שליחת Q מכבה אותן שוב.
//
// LIDAR_DEBUG / FC_DEBUG קובעים רק אילו קבוצות Debug נכללות בקומפילציה.
// גם כשהם 1, שום דבר לא מודפס כל עוד debugEnabled=false.
extern bool debugEnabled;
#define DBG_PRINTF(...)   do { if (debugEnabled) Serial.printf(__VA_ARGS__); } while (0)

// ═══ גרסת הקוד - מוצגת באפליקציה (FWV) כדי לדעת בדיוק מה צרוב ═══
#define FW_VERSION  "E1022Q"

// ═══ אבחון קל (DGN בטלמטריה, חלון של שנייה) ═══
//   מונים בלבד - בלי שום שאילתה נוספת. ראו drone_esp32.ino.
#include <stdint.h>
void ioIdleHook();                     // drone_esp32.ino - פקודות מהאפליקציה בזמן המתנה
static uint16_t dgMspReq  = 0;         // שאילתות MSP שנשלחו ל-Matek
static uint16_t dgMspFail = 0;         // שאילתות שלא נענו בזמן / פגומות
#define DBG_PRINT(...)    do { if (debugEnabled) Serial.print(__VA_ARGS__); } while (0)
#define DBG_PRINTLN(...)  do { if (debugEnabled) Serial.println(__VA_ARGS__); } while (0)

#define LIDAR_DEBUG             1     // 1 = אפשר Debug לידאר בזמן ריצה אחרי פקודת D
#define FC_DEBUG                1     // 1 = אפשר Debug INAV/Phase בזמן ריצה אחרי פקודת D

// ═══════════════════════════════════════════════
//   WiFi
// ═══════════════════════════════════════════════
#define WIFI_SSID      "DroneAP"
#define WIFI_PASS      "drone1234"
#define UDP_PORT       4210

// ערוץ ורוחב ערוץ קבועים - התוצאה של בדיקת הפרעות ה-WiFi ל-GPS.
// עוצמת השידור ולוגיקת התקשורת לא שונו.
#define WIFI_CHANNEL   1       // ערוץ 1, 20MHz (HT20)

// ═══════════════════════════════════════════════
//   TIMINGS
// ═══════════════════════════════════════════════
#define TELEMETRY_HZ        10
#define TELEMETRY_INTERVAL  (1000 / TELEMETRY_HZ)
#define SENS_TO_FPGA_MS     50     // שורת חיישנים ל-FPGA, 20Hz

#define FPGA_TIMEOUT_MS     150    // ה-FPGA עונה מהר
#define FPGA_POLL_MS        2000   // דגימת חיות
#define FPGA_STALE_MS       5000

// ═══════════════════════════════════════════════
//   סקר MSP - לפי חותמת זמן לכל סוג שאילתה
//
//   כל שאילתה חוסמת עד MSP_TIMEOUT_MS, ולכן כל סוג נדגם בקצב שהוא באמת
//   צריך ולא בתור אחיד. כשה-FC לא עונה עוברים לסקר איטי (MSP_BACKOFF_MS),
//   כדי שהלולאה תמשיך לשרת את הלידארים ואת האפליקציה.
//
//   ⚠️ MSP_ALTITUDE נקרא לשימוש פנימי בלבד - בדיקת סבירות של חיישן DWN.
//      הגובה לא נשלח ל-FPGA ולא לאפליקציה: אומדן הגובה הוא באחריות INAV.
// ═══════════════════════════════════════════════
#define MSP_TIMEOUT_MS      30
#define MSP_SETTING_TIMEOUT_MS 250   // קריאת הגדרה (MSP2_COMMON_SETTING) - רק כשמנוטרל. נמדד: INAV 9.1 עונה אחרי 60-115ms (חיפוש השם), ולכן 100 נכשל
#define MSP_MIN_GAP_MS      10     // מרווח מזערי בין שתי שאילתות
#define MSP_BACKOFF_MS      250    // אין קישור ל-FC: שאילתה אחת לרבע שנייה

#define MSP_STATUS_MS       125    // 8Hz - MSP2_INAV_STATUS (חימוש + מצבי טיסה)
#define MSP_ATT_MS          200    // 5Hz - MSP_ATTITUDE (גם מדד ה-IMU החי), בקצב חבילת התצוגה
#define MSP_BATT_MS         330    // 3Hz - MSP2_INAV_ANALOG
#define MSP_GPS_MS          200    // 5Hz - MSP_RAW_GPS, בקצב חבילת התצוגה
#define MSP_ALT_MS          250    // 4Hz - MSP_ALTITUDE, מקומי בלבד
// נתוני תצוגה ואבחון בלבד - לא נכנסים לשום החלטה. 5Hz = קצב חבילת התצוגה
#define MSP_GPSSTAT_MS      200    // 5Hz - MSP_GPSSTATISTICS (HDOP / EPH / EPV)
#define MSP_IMU_MS          200    // 5Hz - MSP_RAW_IMU, רק בשביל MAG X/Y/Z

// ═══════════════════════════════════════════════
//   מכונת שלבי הטיסה - רצה ב-ESP, לא באפליקציה
//
//   ה-ESP הוא הרכיב היחיד שמחזיק בו-זמנית את מצב החימוש של INAV, את חיישן
//   DWN ואת אומדן הגובה הפנימי, ולכן הוא זה שקובע את השלב ומדווח אותו
//   לאפליקציה ול-FPGA. האפליקציה רק מציגה ומשתמשת בו, לא מחשבת אותו.
//
//   GROUND_DISARMED → GROUND_ARMED → TAKEOFF_CONFIRMING → AIRBORNE
//
//   ⚠️ TAKEOFF_CONFIRMING קיים כדי לסגור חור בטיחותי: בין העדות הראשונה
//      להמראה לבין האישור, שחרור מקל המצערת אסור שיחזיר מצערת למינימום.
// ═══════════════════════════════════════════════
#define PHASE_TAKEOFF_CM         12   // דגימת DWN תקינה מעל זה = עדות ראשונה להמראה
#define PHASE_TAKEOFF_SAMPLES     4   // דגימות תקינות **רצופות** לאישור AIRBORNE (~220ms)
// ⚠️ אין פסק זמן שמחזיר לקרקע. מרגע שהתקבלה עדות להמראה, חזרה לקרקע מחייבת
//    עדות חיובית שאנחנו על הקרקע - היעדר מידע לעולם אינו ראיה.
#define PHASE_CONFIRM_GROUND_SAMPLES 3 // דגימות קרקע **רצופות** כדי לבטל אישור המראה
#define PHASE_ALT_FALLBACK_CM    28   // גיבוי: גובה יחסי מ-INAV כש-DWN מפסיק להחזיר קריאה
#define PHASE_ALT_STABLE_MS     500
#define PHASE_LAND_CM             5   // DWN תקין מתחת לזה = מגע קרקע (שמרני בכוונה)
#define PHASE_LAND_STABLE_MS    800
#define PHASE_FWD_ACTIVE_CM      30   // מתחת לגובה הזה FWD לא משמש להחלטות מכשול
#define PHASE_FWD_ACTIVE_MS     400
#define PHASE_FWD_ALT_CM         40   // גיבוי: גובה יחסי מ-INAV שמאשר את החיישן הקדמי

// ═══════════════════════════════════════════════
//   רצפת המצערת בזמן אישור ההמראה
//
//   ⚠️ הרצפה אינה נקבעת מהחבילה האחרונה בלבד. אם הטייס דחף 60% להמראה
//      ושחרר מעט ברגע שלפני דגימת ה-DWN שהפעילה את השלב, החבילה האחרונה
//      כבר נמוכה - ורצפה שנקבעת ממנה אינה מגינה על כלום. לכן נשמר שיא
//      קצר של המצערת, והרצפה היא המרבי מבין השניים.
//
//   החלון קצר בכוונה: שיא מאז החימוש כולו היה הופך דחיפה ישנה של הסטיק
//   לרצפה גבוהה מדי, וזו תקלה בכיוון ההפוך.
// ═══════════════════════════════════════════════
#define PHASE_THR_PEAK_MS       300   // חלון שיא המצערת שנלקח בחשבון ברצפה
#define PHASE_THR_TRACE_N         8   // דגימות שנשמרות (שליטה ב-20Hz ⇒ 400ms)

// ⚠️ שחרור הרצפה במעבר ל-AIRBORNE מחכה לאישור מהאפליקציה (שדה PHA בשורת
//    השליטה), כי הטלמטריה מגיעה אליה רק ב-10Hz. הגיבוי הזה משחרר את הרצפה
//    גם בלי אישור, כדי שאפליקציה בגרסה ישנה - או אישור שאבד - לא ינעלו את
//    הרחפן במצערת שאי אפשר להוריד ממנה.
#define PHASE_AIRBORNE_ACK_MS   600
#define PHASE_ACK_FRESH_MS      500   // אישור ישן מזה אינו נחשב אישור

// פעימת לב של האפליקציה. במצב אובדן קשר מותר לה לשלוח רק PING, DISARM ו-ABORT,
// ולכן צריך אות חיים שאינו שורת שליטה ואינו מאפס את מונה אובדן הקשר ב-FPGA.
#define APP_PING_STALE_MS      2000

#define FC_STALE_MS         1000   // סטטוס
#define ATT_STALE_MS        1000
#define ALT_STALE_MS        1000
#define GPS_STALE_MS        1500
#define GPSSTAT_STALE_MS    3000   // 3 סבבי סקר של MSP_GPSSTATISTICS
#define MAG_STALE_MS        1500   // 3 סבבי סקר של MSP_RAW_IMU

// ═══ בריאות המצפן - זיהוי מצפן קפוא (msp.h) ═══
//   INAV עלול לדווח MAG=OK בזמן ש-X/Y/Z קפואים לגמרי (נצפה אחרי אתחול רך של
//   הבקר מה-CLI). ערכים שלא משתנים כשהרחפן עומד הם תקינים, ולכן ההחלטה דורשת
//   **הוכחת תנועה בלתי תלויה** - ג'יירו מאותה תשובת MSP_RAW_IMU, או Roll/Pitch.
//   Heading אינו ראיה: הוא עצמו נגזר מהמצפן.
//   כויל מול 9 לוגים אמיתיים: מצפן קפוא - שינוי 0 בכל חלון תנועה (5 טיסות),
//   מצפן חי - שינוי של 71 לפחות בכל חלון תנועה (4 טיסות).
#define MAG_CHECK_WINDOW_MS   2500   // חלון ההחלטה
#define MAG_CHECK_MIN_SAMPLES    8   // דגימות RAW_IMU טריות לפחות בחלון (5Hz ⇒ ~12)
#define MAG_MOVE_ROT_DEG        45   // תנועה: סיבוב מצטבר לפי הג'יירו בתוך החלון...
#define MAG_MOVE_PEAK_DPS       20   // ...ולפחות דגימה אחת בקצב הזה
#define MAG_MOVE_ATT_DEG        20   // או: טווח Roll או Pitch בתוך החלון
#define MAG_FROZEN_SPAN          2   // שינוי מרבי בכל ציר שנחשב "לא זז"
#define MAG_ALIVE_SPAN          30   // שינוי בציר כלשהו שמוכיח תגובה לתנועה
#define SLOW_TELEM_MS        200   // חבילת התצוגה (ATTITUDE + GPS + MAG), 5Hz
#define DISARM_GRACE_MS     2000

// ═══════════════════════════════════════════════
//   BATTERY - 3S LiPo (11.1V)
//
//   ⚠️ המתח והאחוזים מגיעים מ-MSP2_INAV_ANALOG (batt.h).
//      הקבועים כאן נשארים לשימוש batt.h ולתיעוד המערכת.
// ═══════════════════════════════════════════════
#define BATTERY_CELLS       3
#define CELL_FULL           4.20f
#define CELL_EMPTY          3.50f
#define BATTERY_MIN_PCT     20
#define BATT_SMOOTH         0.15f   // ממוצע נע ישן · הסינון בפועל ב-batt.h

// ═══════════════════════════════════════════════
//   זיהוי המראה ונחיתה - שני מקורות, אף אחד מהם אינו יחיד
//
//   ⚠️ ה-DWN **אינו** חיישן הגובה של הרחפן. הטווח שלו מסתיים סביב 1.3m,
//      והטיסות יהיו ברובן מעליו. הוא מדויק וזמין דווקא קרוב לקרקע, ולכן הוא
//      המקור המועדף בהמראה ובנחיתה - אבל INAV חייב להיות מסוגל לאשר המראה
//      בכוחות עצמו. אחרת דגימה חסרה משאירה את המערכת ב"קרקע" בזמן שהרחפן
//      כבר באוויר, ואז שחרור המקל מצווה מצערת קרקע על רחפן שטס.
//
//   ⚠️⚠️ **גובה לבדו אינו המראה.** הברומטר נודד, ונצפתה אצלנו נדידה של יותר
//      מחצי מטר ברחפן שעומד לגמרי במקום. לכן מסלול ההמראה דרך INAV דורש
//      שלושה תנאים יחד: גובה יציב, עדות טיפוס אמיתית לפי המהירות האנכית,
//      ומצערת שמתאימה להמראה. סחיפת ברומטר אינה מייצרת אף אחד משני האחרונים.
// ═══════════════════════════════════════════════
#define PHASE_TAKEOFF_ALT_CM     45   // גובה יחסי מ-INAV שמעיד על המראה בלי DWN
#define PHASE_TAKEOFF_ALT_MS    700   // ...ויציב כך למשך הזמן הזה
#define PHASE_TAKEOFF_VSPD_CMS      40   // מהירות אנכית שמעידה על טיפוס אמיתי
// ⚠️ נספרות **דגימות MSP_ALTITUDE חדשות**, לא זמן. הסקר הוא 4Hz ומכונת השלבים
//    רצה ב-20Hz, ולכן מדידת זמן על ערך שמור הייתה סופרת את אותה דגימה חמש
//    פעמים - ודגימה חולפת אחת הייתה נראית כמו טיפוס רצוף.
#define PHASE_TAKEOFF_CLIMB_SAMPLES  2   // דגימות VSPD חדשות רצופות מעל הסף
#define PHASE_CLIMB_MEMORY_MS     3000   // כמה זמן עדות הטיפוס נשארת תקפה
#define PHASE_TAKEOFF_THR_PCT    35   // מצערת מינימלית שמלווה המראה אמיתית

#define PHASE_NEAR_GROUND_CM     60   // DWN תקין וקרוב מזה = אזור הקרקע
#define PHASE_NEAR_GROUND_MS   1500   // כמה זמן היסטוריית "היינו באזור הקרקע" תקפה
#define PHASE_TOUCHDOWN_MS      600   // אימות מגע קרקע
#define PHASE_TOUCHDOWN_THR_PCT  45   // מצערת נמוכה מזה = כבר לא מרחפים
// ⚠️ המדידה התקינה **האחרונה** לפני שהחיישן השתתק חייבת להיות ממש נמוכה.
//    בלי זה, רצף כמו 55cm ואז שקט היה נספר כנחיתה בזמן שהרחפן עוד באוויר.
#define PHASE_TOUCHDOWN_LAST_CM  10
#define PHASE_TOUCHDOWN_VSPD_CMS 20   // |מהירות אנכית| מתחת לזה = כבר לא יורדים
// ═══ מגע (E1021C): TOO_CLOSE לבדו אינו מגע ═══
//   TOO_CLOSE נחשב עדות קרקע במגע רק אם הוא התחיל מיד אחרי התקרבות טרייה (מדידה תקינה
//   ≤ PHASE_NEAR_GROUND_CM בתוך PHASE_NEAR_GROUND_MS) ונמשך ברצף מאז. ובנוסף, בדיקת סבירות:
//   כש-INAV מראה יותר מ-PHASE_TOUCHDOWN_ALT_MAX_CM אין מגע (הברומטר ליד הקרקע מראה נמוך -
//   ולכן הבדיקה לא חוסמת מגע אמיתי; היא לבדה גם אינה הוכחה למגע).
#define PHASE_TOUCHDOWN_ALT_MAX_CM 100
// ═══ E1021D: TOO_CLOSE כעדות מגע רק כשיש מדידה גולמית טרייה שתומכת בו ═══
//   נמצא בבדיקה: התקרבות ⇒ 0mm ⇒ נעילה, ואחריה רק NO_RETURN 3 שנ' ⇒ הנעילה נשארת, ו-TOO_CLOSE
//   נחשב עדות מגע עם ההתקרבות הישנה. עכשיו: TOO_CLOSE בלי דגימה גולמית שתומכת בו (dwnTcRawMs)
//   בתוך PHASE_TC_FRESH_MS אינו עדות, וגם שובר את הרצף (צריך התקרבות חדשה).
//   600: על הקרקע בטיסות 20.10 + 21.10 הפער המרבי בין שתי דגימות כאלה היה 552ms (13,536 פערים, 99% < 200ms).
#define PHASE_TC_FRESH_MS        600
// E1022G: מגע מאומת במצב ניווט (SURFACE) - navTouch, נשלח ב-SRS שדה 5. האפליקציה נועלת 0% רק לפיו.
//   נקבע באותה הוכחת מגע כמו GROUND_ARMED (PHASE_TOUCHDOWN_MS), ונמחק רק בעדות לאוויר:
//   דגימות DWN תקינות חדשות ורצופות מעל PHASE_TAKEOFF_CM, או גובה INAV מעל PHASE_TOUCHDOWN_ALT_MAX_CM.
//   שתיקת DWN על הקרקע אינה מוחקת אותו (על הקרקע היא המצב הרגיל).
#define PHASE_NAVTD_CLEAR_SAMPLES  2
// E1022I: יציאה מ-SURFACE בזמן שהוכחת המגע כבר מתקיימת (לפני ה-600ms) ⇒ navTouch מיד.
//   INAV 9.1 SITL: יציאה 0.2 שנ' אחרי נגיעה, לפני האימות ⇒ החזקת מרכז ⇒ הרחפן עלה ל-2m.
//   "מתקיימת" = הוכחת המגע המלאה (groundProof + כוונת ירידה + מצערת נמוכה, במצב ניווט) התקיימה
//   בתוך PHASE_NAVTD_EXIT_MS האחרונים, בלי עדות לאוויר מאז.
#define PHASE_NAVTD_EXIT_MS      300
// E1022J: וגם המדידה התקינה האחרונה של ה-DOWN לפני העדות (הגולמית, lid.dwnRawCm) הייתה ממש ליד הקרקע.
//   host test J1: קריאת 0-3mm בודדת ב-20-30cm + אובדן DOWN + יציאה ⇒ ב-E1022I navTouch שגוי באוויר.
//   בנחיתה אמיתית המדידה התקינה האחרונה היא 4-12cm (מינימום הטווח 4cm + 55ms בין דגימות ב-50-65cm/s).
//   מעל 15cm ⇒ לא; ≤ 15cm באוויר ⇒ 0% מגובה ≤ 15cm (INAV: prepareForTakeoffOnReset) - לא מסוכן.
#define PHASE_NAVTD_EXIT_LAST_CM  15
// E1022M: במצב ניווט הוכחת מגע שמבוססת רק על שתיקת DOWN (בלי TOO_CLOSE / DWN ≤ 5cm) תקפה רק עד
//   PHASE_NAVTD_SILENT_MS אחרי המדידה התקינה האחרונה ליד הקרקע. קצר מ-PHASE_TOUCHDOWN_MS בכוונה: שתיקה לבדה
//   לא משלימה אימות מלא, רק מחזיקה את חלון היציאה (PHASE_NAVTD_EXIT_MS) פתוח. ‏ANGLE (GROUND_ARMED) לא משתנה.
#define PHASE_NAVTD_SILENT_MS     400
// ═══ E1022N: נחיתה רכה ב-SURFACE / POSHOLD - מעבר מהיר ל-ANGLE+סרק בנגיעה ═══
//   28.10: בכל הנחיתות במצב ניווט הרחפן נגע ב-1.15-3.4g וקפץ: הברומטר צונח ליד הקרקע (אפקט הפרופלורים),
//   INAV רואה "ירידה" מדומה ומוסיף דחף. אימות המגע (PHASE_TOUCHDOWN_MS=600) איטי מדי ולא מספיק לפני הקפיצה.
//   מסלול מהיר: אותה הוכחת קרקע (TOO_CLOSE אחרי התקרבות / DWN תקין ≤ PHASE_LAND_CM עם מדידה אחרונה ≤
//   PHASE_NAVTD_EXIT_LAST_CM), בגובה סביר (PHASE_TOUCHDOWN_ALT_MAX_CM), כוונת ירידה ומצערת נמוכה, אבל רק
//   PHASE_FASTTD_SAMPLES דגימות DWN **חדשות** רצופות (~110ms). הנעילה (fastTd) גורמת ל-ESP להחליף את
//   שורת השליטה ל-MODE:ANGLE עם מצערת 0 (ה-FPGA מקבל ANGLE מיד; אותו רצף שהאפליקציה שולחת ב-DISARM).
//   שחרור: ניתוק מנועים, ביטול navTouch (עדות לאוויר: ‏DWN > PHASE_TAKEOFF_CM ברצף או INAV > 100cm), או
//   מקל מעל PHASE_FASTTD_RELEASE_PCT לפחות PHASE_FASTTD_RELEASE_MS. אחרי שחרור - PHASE_FASTTD_LOCK_MS בלי
//   הפעלה מחדש. לא פעיל בזמן משימה אוטונומית (missionEngaged).
#ifndef PHASE_FASTTD_SAMPLES
#define PHASE_FASTTD_SAMPLES        2
#endif
#define PHASE_FASTTD_WINDOW_MS    300   // שתי דגימות הקרקע חייבות להיות בתוך החלון הזה
#define PHASE_FASTTD_RELEASE_PCT   55
#define PHASE_FASTTD_RELEASE_MS   250
#define PHASE_FASTTD_LOCK_MS     1000
// E1022N: חסימת האליאס של ה-DOWN (DWN_ALIAS_ALT_SHORT_CM) הסתירה את הקרקע 1.5 שנ' בנחיתה (INAV z=190 כשהלידאר 88cm):
//   נגיעה בלי DOWN תקין ⇒ בלי אימות. בזמן כוונת ירידה, ועד גובה INAV זה, דגימה תקינה ≤ LIDAR_DWN_OBST_MAX_MM
//   (כמו בחציית מכשול) לא נחסמת. גבוה מזה/בלי כוונת ירידה - החסימה כמקודם.
#define DWN_ALIAS_RELAX_ALT_CM    300
// E1022N: DOWN status 2 (signal fail) נחשב OOR למשימה רק אחרי כך וכך דגימות ברצף (~55ms כל אחת). עם REAR_N=4 ב-FSM
//   ראש כהה שמחזיר signal fail חייב להימשך כ-0.5 שנ' לפחות כדי לאשר קצה מכשול.
#define LIDAR_SIGFAIL_OOR_N         6
// E1022K: "הקריאה הגולמית האחרונה" היא רק קריאה **מאומתת**: התקבלה בסינון הקפיצות, או קפיצה שנדחתה
//   שהיא חלק משרשרת של LIDAR_PROX_CHAIN_N קפיצות יורדות ברצף (ירידה מהירה אמיתית). קריאה בודדת שנדחתה
//   (למשל 90mm באמצע ריחוף ב-25cm) אינה הוכחת קרבה. host test J6 / J10.

// ═══════════════════════════════════════════════
//   Mission FSM - המשימה האוטונומית (mission.h)
//
//   כלול תמיד ורדום: המשימה מתחילה רק ב-CMD:MSTART מפורש מהאפליקציה, אחרי
//   הכנה (CMD:MPREP) כשהרחפן מנוטרל, ARM, ו-16 בדיקות SELF_CHECK ב-FPGA.
//   אין שום מסלול שבו משימה מתחילה מעצמה, גם לא אחרי אתחול.
//   ⚠️ ה-ESP לא כותב שום הגדרה ל-INAV ולא טוען משימה. הוא רק קורא ובודק:
//      משימת ה-WP, nav_mc_althold_throttle, nav_user_control_mode,
//      nav_wp_mission_restart, nav_auto_speed, nav_max_auto_speed ו-thr_mid.
//      ההחלטה - ב-SELF_CHECK של ה-FPGA.
// ═══════════════════════════════════════════════
#define MISSION_ENABLE          1      // כלול תמיד; 0 רק לבדיקת רגרסיה

// ═══ שלב 0ב: DOWN ל-INAV כ-MSP Rangefinder (rfmsp.h) ═══
//   0 = לא נשלח כלום (כמו E1019A). INAV משתמש בזה רק כש-rangefinder_hardware = MSP.
#define RF_MSP_ENABLE           1

// ═══ E1022A: SURFACE ידני (surface.h) ═══
//   האפליקציה מבקשת (SRF:1 בשורת השליטה). ה-ESP שולח ל-FPGA את SRF בשורת החיישנים (20Hz,
//   גם בלי שורות מהאפליקציה), וה-FPGA מוציא CH8 + פולס כניסה מחדש (POSHOLD בלבד).
//   SRF:1 רק כש: בקשה טרייה מהאפליקציה · MODE:POSHOLD נבחר · אין משימה · INAV מדווח
//   rangefinder (rangefinder_hardware = MSP) · נשלחה ל-INAV מדידה חיובית (VALID עד
//   SURF_MAX_MM, או קרקע 10mm) ב-SURF_LOSS_MS האחרונים.
//   אובדן DOWN ⇒ SRF:0 והחסימה נשארת עד שהאפליקציה מבקשת מחדש (0 ואז 1).
//   SURF_LOSS_MS: INAV מוותר על ה-AGL אחרי 400ms בלי נתון (INAV_SURFACE_TIMEOUT_MS). ב-SITL
//   של INAV 9.1, יציאה אחרי 150-300ms לא שינתה את הגובה; בלי יציאה - ירידה מ-61 ל-10cm.
#define SURF_ENABLE             1
#define SURF_LOSS_MS            150    // E1022B: 200 ⇒ 150, כדי שגם עם 100ms ההקדמה ב-FPGA היציאה תהיה לפני 400ms של INAV
#define SURF_MAX_MM             1100   // nav_max_terrain_follow_alt = 100cm + מרווח; SHORT רואה עד 1.3m
// E1022D: כניסה ל-SURFACE רק כש-INAV כבר סומך על ה-AGL. INAV 9.1 (navigation_pos_estimator_agl.c):
//   reliability = מסנן RC (קבוע 0.47802s) על כל מדידה שהתקבלה (1 = 0..max_surface_altitude, 0 = ‎-1),
//   מתאפס אחרי 400ms בלי מדידה; AGL "TRUSTED" רק מ-0.75. עד אז SURFACE של INAV **יורד** ב-50..200 ס"מ/שנ'
//   (adjustMulticopterAltitudeFromRCInput) - נמדד ב-23.10 15:16:23: כניסה 0.12s אחרי שה-DOWN חזר ⇒ נגיעה ב-‎-86.
//   כאן אותו חישוב בדיוק על המסגרות שנכתבו בפועל, וכניסה רק מ-SURF_REL_ENTER (מרווח מעל 0.75: ~1s רצוף).
#define SURF_REL_TAU_S          0.47802f   // RANGEFINDER_RELIABILITY_RC_CONSTANT
#define SURF_REL_TIMEOUT_MS     400        // INAV_SURFACE_TIMEOUT_MS
#define SURF_REL_ENTER          0.85f      // INAV: RANGEFINDER_RELIABILITY_HIGH_THRESHOLD = 0.75
// E1022E: כניסה רק עד 90 ס"מ. האפליקציה ממפה את מרכז המקל לגובה רק עד 100 ס"מ (nav_max_terrain_follow_alt),
//   ומעל זה היעד של INAV נקבע מהמקל הישן: 24.10 23:44:37 כניסה ב-102 ס"מ עם יעד 56 ⇒ ירידה ותנודות.
//   היציאה לא השתנתה (מעל SURF_MAX_MM).
#define SURF_ENTER_MAX_MM       900
// E1022B: מעל LINK_LOSS_MS של ה-FPGA (1000). פער WiFi קצר (נמדד עד 581ms) לא נוגע ב-SURFACE;
//   פער ארוך - ה-FPGA עוצר את ה-RC, והבקשה ננעלת עד SRF:0 ⇒ SRF:1 (אין כניסה חוזרת מעצמה).
#define SURF_APP_STALE_MS       1500
// מסירת המצערת ביציאה מ-SURFACE: ב-FPGA (F1022B). כאן רק גבולות ה-THS שנשלח לו
#define SURF_HOLD_MIN_H         40     // 20%: מעל min_check, כדי ש-INAV ייקח אותו כנייטרל (STICK)
#define SURF_HOLD_MAX_H         180

// ── המשימה ──
//   המשימה נטענת ל-INAV מה-Configurator. האפליקציה רק מכינה אותה (MPREP),
//   וה-ESP שומר את מספר ה-WP וה-CRC שלה עד MCANCEL או אתחול.
//   מותר: WAYPOINT / HOLD_TIME בלבד. LAND / RTH במשימה ⇒ נדחית (הסיום: POSHOLD ב-B).
//   ⚠️ המהירות נקבעת ב-INAV: p1 של כל WP (≥50), או nav_auto_speed כש-p1 < 50,
//      מוגבלת ב-nav_max_auto_speed. המהירות האפקטיבית המרבית חייבת להיות עד
//      MISSION_SPEED_MAX_CMS (ה-FPGA בודק את אותו ערך ב-SELF_CHECK).
#define MISSION_SPEED_MAX_CMS   100    // = AUTO_SPEED_MAX ב-mission_fsm.v
#define MISSION_ALT_MIN_CM      100    // גובה WP1 מינימלי (יחסי לבית)
#define MISSION_ALT_MAX_CM      400    // גובה WP1 מרבי (התקרה המרבית)

// ── קצבים ──
#define MISSION_POSE_MS         100    // MSP2_INAV_FULL_LOCAL_POSE, 10Hz
#define MISSION_NAV_MS          100    // MSP_NAV_STATUS, 10Hz
#define MISSION_FAST_LINE_MS    100    // שורת INAV ל-FPGA, 10Hz
#define MISSION_SLOW_LINE_MS    500    // שורת המשימה והתצורה ל-FPGA, 2Hz
#define MISSION_STAT_MS         200    // CMD:MSTAT ל-FPGA, 5Hz - כשמשימה נבחרה
#define MISSION_STAT_IDLE_MS    500    // ...ו-2Hz בלי משימה (MST נחשב ישן אחרי 1000ms)
#define MISSION_MSE_MS          1000   // סט ההגדרות ל-FPGA, 1Hz (ובכל שינוי)
#define MISSION_SET_CONFIRM_MS  2500   // אישור הסט מה-FPGA (SCK ב-MST) - עד כאן
#define MISSION_VERIFY_MS       5000   // בדיקה חוזרת - רק כשה-Mission FSM אינו פעיל
#define MISSION_VERIFY_RETRY_MS 1000   // אחרי כישלון קריאה
#define MISSION_MST_STALE_MS    1000   // בלי תשובת MST זמן כזה ⇒ MSV:0 לאפליקציה

// ── הגדרות העקיפה (CMD:MSET) - ברירות מחדל וטווחים ──
//   נשמרות ב-NVS של ה-ESP, ונשלחות ל-FPGA כסט אחד עם checksum (MSE).
//   אותם טווחים באפליקציה (MissionSettings) וב-FPGA (set_in_range).
//   כל המרחקים ב-cm. FRONT נמדד מקצה הפרופלור הקדמי (קיזוז 18cm כבר כאן, פעם אחת).
//   התקרה יחסית לבית של INAV (Z של FULL_LOCAL_POSE).
#define MSET_DEF_STOP           100
#define MSET_DEF_CLEAR          150
#define MSET_DEF_VCLR           70
#define MSET_DEF_FCLR           70
#define MSET_DEF_CEIL           400
#define MSET_DEF_DEPTH          50
#define MSET_DEF_STAGE          2      // ברירת מחדל: עצירה בלבד
#define MSET_DEF_FLAGS          0      // המראה אוטומטית כבויה עד אישור מפורש
#define MSET_MIN_STOP           40
#define MSET_MAX_STOP           300
#define MSET_MAX_CLEAR          380
#define MSET_MIN_VCLR           30
#define MSET_MAX_VCLR           150
#define MSET_MIN_FCLR           30
#define MSET_MAX_FCLR           200
#define MSET_MIN_CEIL           150
#define MSET_MAX_CEIL           400
#define MSET_MIN_DEPTH          10
#define MSET_MAX_DEPTH          200
#define MSET_FRONT_MARGIN_CM    10     // מרחק זיהוי / פנוי חייבים להיות בתוך הטווח של FRONT

// ── סיווג LiDAR למשימה (FRC / DRC) ──
//   ה-status של ST שנחשבים OUT_OF_RANGE: חיישן תקין בלי מטרה בטווח. 4 = out of bounds.
//   7 = wrapped target אינו OUT_OF_RANGE: הוא נשלח כ-UNCERTAIN (lidar.h).
//   ⚠️ OUT_OF_RANGE לבדו אינו הוכחה: ב-FPGA הוא נספר כ"פנוי" רק ב-FRONT במצב
//      LONG, ורק אחרי זיהוי VALID של המכשול. FAULT / TIMEOUT הם כשל אמיתי.
#define LIDAR_OOR_STATUS_MASK   (1u << 4)
