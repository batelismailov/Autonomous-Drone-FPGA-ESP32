#pragma once
#include <HardwareSerial.h>
#include "config.h"

// ⚠️ חייב להיכלל אחרי msp.h — SerialFC מוגדר שם.

#define MSP2_INAV_ANALOG   0x2002
#define MSP2_TIMEOUT_MS    40
#define BATT_AVG_N         16      // חלון הממוצע הנע
#define BATT_STALE_MS      1500    // כשל MSP רגעי לא מבטל מיד נתון סוללה תקין

enum BattState : uint8_t {
    BATT_OK          = 0,
    BATT_WARNING     = 1,
    BATT_CRITICAL    = 2,
    BATT_NOT_PRESENT = 3
};

struct BattData {
    uint16_t vRaw   = 0;    // מדידה אחרונה, יחידות 0.01V
    uint16_t vAvg   = 0;    // ממוצע נע, יחידות 0.01V
    uint8_t  pctRaw = 0;
    uint8_t  pctAvg = 0;
    uint8_t  state  = BATT_NOT_PRESENT;
    uint8_t  cells  = 0;
    bool     fresh      = false;
    unsigned long lastGoodMs = 0;  // זמן קריאת הסוללה התקינה האחרונה
};

// ═══ ממוצע נע ═══
static uint16_t _vBuf[BATT_AVG_N];
static uint8_t  _pBuf[BATT_AVG_N];
static uint8_t  _bIdx   = 0;
static uint8_t  _bCount = 0;

static void battPush(uint16_t v, uint8_t p) {
    _vBuf[_bIdx] = v;
    _pBuf[_bIdx] = p;
    _bIdx = (_bIdx + 1) % BATT_AVG_N;
    if (_bCount < BATT_AVG_N) _bCount++;
}

static uint16_t battAvgV() {
    if (_bCount == 0) return 0;
    uint32_t s = 0;
    for (uint8_t i = 0; i < _bCount; i++) s += _vBuf[i];
    return (uint16_t)(s / _bCount);
}

static uint8_t battAvgP() {
    if (_bCount == 0) return 0;
    uint32_t s = 0;
    for (uint8_t i = 0; i < _bCount; i++) s += _pBuf[i];
    return (uint8_t)(s / _bCount);
}

// ═══ MSP v2 ═══
static uint8_t crc8DvbS2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

// "$X<" flag fn_lo fn_hi size_lo size_hi crc     — בקשה ללא מטען
static void msp2Request(uint16_t fn) {
    dgMspReq++;
    uint8_t h[8];
    h[0] = '$'; h[1] = 'X'; h[2] = '<';
    h[3] = 0;                                  // flag
    h[4] = (uint8_t)(fn & 0xFF);
    h[5] = (uint8_t)(fn >> 8);
    h[6] = 0; h[7] = 0;                        // payload size = 0
    uint8_t crc = 0;
    for (uint8_t i = 3; i < 8; i++) crc = crc8DvbS2(crc, h[i]);
    SerialFC.write(h, 8);
    SerialFC.write(crc);
}

// מחזיר את אורך המטען, או -1
static int msp2Read(uint16_t wantFn, uint8_t* out, int maxLen, uint16_t timeoutMs) {
    unsigned long t0 = millis();
    uint8_t  st = 0, crc = 0;
    uint16_t fn = 0, size = 0, idx = 0;

    // ⚠️ ממשיכים לקרוא גם אחרי פסק הזמן כל עוד יש בתים בתור: הטיפול בפקודות מהאפליקציה
    //    בזמן ההמתנה (ioIdleHook) לוקח זמן, ותשובה שכבר הגיעה לא תיחשב ככישלון.
    while (millis() - t0 < timeoutMs || SerialFC.available()) {
        if (!SerialFC.available()) { ioIdleHook(); continue; }
        uint8_t b = SerialFC.read();
        switch (st) {
            case 0: if (b == '$') st = 1; break;
            case 1: st = (b == 'X') ? 2 : 0; break;
            case 2: st = (b == '>') ? 3 : 0; break;
            case 3: crc = crc8DvbS2(0, b);   st = 4; break;                 // flag
            case 4: fn  = b;  crc = crc8DvbS2(crc, b); st = 5; break;
            case 5: fn |= (uint16_t)b << 8;  crc = crc8DvbS2(crc, b); st = 6; break;
            case 6: size = b; crc = crc8DvbS2(crc, b); st = 7; break;
            case 7: size |= (uint16_t)b << 8; crc = crc8DvbS2(crc, b);
                    idx = 0; st = (size == 0) ? 9 : 8; break;
            case 8: if (idx < maxLen) out[idx] = b;
                    crc = crc8DvbS2(crc, b);
                    idx++;
                    if (idx >= size) st = 9;
                    break;
            case 9: if (b == crc && fn == wantFn) return (int)size;
                    st = 0; break;
        }
    }
    dgMspFail++;
    return -1;
}

// ═══════════════════════════════════════════════
//   MSP2_INAV_ANALOG — 24 בתים
//     [0]      flags: bit0 full · bit1 capacity · bits2-3 state · bits4-7 cells
//     [1..2]   מתח        uint16, יחידות 0.01V
//     [3..4]   זרם        (אין חיישן — מתעלמים)
//     [5..8]   הספק       (מתעלמים)
//     [9..12]  mAh        (מתעלמים)
//     [13..16] mWh        (מתעלמים)
//     [17..20] קיבולת     (מתעלמים)
//     [21]     אחוז סוללה — החישוב של iNav
//     [22..23] RSSI
// ═══════════════════════════════════════════════
bool pollBattery(BattData& b) {
    while (SerialFC.available()) SerialFC.read();      // ניקוי שאריות

    msp2Request(MSP2_INAV_ANALOG);

    uint8_t p[32];
    int n = msp2Read(MSP2_INAV_ANALOG, p, sizeof(p), MSP2_TIMEOUT_MS);
    if (n < 22) {
        unsigned long now = millis();
        b.fresh = (b.lastGoodMs != 0 && (unsigned long)(now - b.lastGoodMs) <= BATT_STALE_MS);
        return false;
    }

    uint8_t flags = p[0];
    b.state  = (flags >> 2) & 0x03;
    b.cells  = (flags >> 4) & 0x0F;
    b.vRaw   = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
    b.pctRaw = p[21];

    battPush(b.vRaw, b.pctRaw);
    b.vAvg     = battAvgV();
    b.pctAvg   = battAvgP();
    b.lastGoodMs = millis();
    b.fresh    = true;
    return true;
}