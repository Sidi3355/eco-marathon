/*
  Driver information system, Imperial Eco-Marathon prototype
  Target: ESP32-S3-DevKitC-1 (Arduino-ESP32 core 3.x)

  Inputs : Hall wheel sensor (GPIO7), GPS on UART1 (GPIO17 RX, GPIO18 TX),
           buttons (GPIO4/5/6), battery divider on GPIO1 (ADC1)
  Outputs: Sharp memory LCD and microSD on one hardware SPI bus,
           red/green pace LEDs (GPIO40/39), CSV log on microSD

  Libraries (Library Manager): Adafruit_SharpMem, Adafruit_GFX, TinyGPSPlus, SD
  Set WHEEL_CIRC_M, the track and the strategy plan in config.h before flashing.
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <TinyGPSPlus.h>
#include "config.h"

// ---------------------------------------------------------------- pins
// Chosen to avoid strapping pins (0, 3, 45, 46), native USB (19, 20),
// UART0 (43, 44) and the octal PSRAM pins (35 to 37).
constexpr int PIN_VBAT      = 1;   // ADC1_CH0, 100k/100k divider
constexpr int PIN_BTN_LAP   = 4;
constexpr int PIN_BTN_PAGE  = 5;
constexpr int PIN_BTN_RESET = 6;
constexpr int PIN_PULSE     = 7;   // Hall sensor, falling edge
constexpr int PIN_LCD_CS    = 10;  // Sharp CS is active high; the library handles it
constexpr int PIN_MOSI      = 11;
constexpr int PIN_SCK       = 12;
constexpr int PIN_MISO      = 13;
constexpr int PIN_SD_CS     = 14;
constexpr int PIN_LCD_DISP  = 16;  // high = display on
constexpr int PIN_GPS_RX    = 17;  // from GPS TX
constexpr int PIN_GPS_TX    = 18;  // to GPS RX
constexpr int PIN_LED_G     = 39;
constexpr int PIN_LED_R     = 40;

// ---------------------------------------------------------------- speed from Hall pulses
// The ISR records the time between successive pulses. Period measurement, not
// pulse counting, so resolution is good at low speed. Pulses closer than
// MIN_PERIOD_US are treated as bounce or EMI (that is ~96 km/h here).
volatile uint32_t g_lastPulseUs  = 0;
volatile uint32_t g_periodUs     = 0;
volatile uint32_t g_pulseCount   = 0;
constexpr uint32_t MIN_PERIOD_US = 15000;

void IRAM_ATTR onWheelPulse() {
  uint32_t now = micros();
  uint32_t dt  = now - g_lastPulseUs;
  if (dt < MIN_PERIOD_US) return;
  g_lastPulseUs = now;
  g_periodUs    = dt;
  g_pulseCount++;
}

float wheelSpeedKmh() {
  uint32_t period, last;
  noInterrupts();
  period = g_periodUs;
  last   = g_lastPulseUs;
  interrupts();
  // No pulse for 2 s means stopped
  if (period == 0 || (micros() - last) > 2000000UL) return 0.0f;
  float metresPerPulse = WHEEL_CIRC_M / MAGNETS_PER_REV;
  return metresPerPulse / (period * 1e-6f) * 3.6f;
}

float distanceM() {
  uint32_t n;
  noInterrupts();
  n = g_pulseCount;
  interrupts();
  return n * (WHEEL_CIRC_M / MAGNETS_PER_REV);
}

// ---------------------------------------------------------------- peripherals
// Hardware SPI for both devices. A bit-banged LCD on the same pins would steal
// them from the SPI peripheral and break the SD card.
Adafruit_SharpMem lcd(&SPI, PIN_LCD_CS, 400, 240, 2000000);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
File logFile;
bool sdOk = false;

// ---------------------------------------------------------------- race state
struct RaceState {
  bool     running   = false;
  bool     finished  = false;
  uint32_t startMs   = 0;
  uint32_t finishMs  = 0;
  uint32_t lapStartMs = 0;
  uint8_t  lap       = 0;      // completed laps
  float    lapDistStartM = 0;
  float    distStartM = 0;
  uint32_t lastLapMs = 0;
  bool     insideStartZone = false;
} race;

uint8_t page = 0;   // 0 = race, 1 = diagnostics

// Exponentially smoothed speed for the display, raw speed for logging
float speedKmh = 0, speedSmooth = 0;

float batteryVolts() {
  // 11 dB attenuation: about 0 to 3.1 V, calibrated by analogReadMilliVolts.
  // 100k/100k divider, so 4.2 V reads 2.1 V.
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);
  return (mv / 8) / 1000.0f * 2.0f;
}

// ---------------------------------------------------------------- buttons with debounce
struct Button {
  int pin; bool state = true; bool last = true; uint32_t tChange = 0; bool pressed = false;
  void update() {
    bool r = digitalRead(pin);
    if (r != last) { tChange = millis(); last = r; }
    if ((millis() - tChange) > 30 && r != state) { state = r; if (!state) pressed = true; }
  }
  bool take() { bool p = pressed; pressed = false; return p; }
};
Button btnLap{PIN_BTN_LAP}, btnPage{PIN_BTN_PAGE}, btnReset{PIN_BTN_RESET};

// ---------------------------------------------------------------- lap and pacing logic
void startRace() {
  race = RaceState();
  race.running = true;
  race.startMs = millis();
  race.lapStartMs = race.startMs;
  race.lapDistStartM = distanceM();
  race.distStartM = race.lapDistStartM;
  race.insideStartZone = true;
}

void markLap() {
  uint32_t now = millis();
  if (race.finished) return;
  if (!race.running) { startRace(); return; }
  if (now - race.lapStartMs < MIN_LAP_MS) return;    // reject double trigger
  race.lastLapMs = now - race.lapStartMs;
  race.lapStartMs = now;
  race.lap++;
  race.lapDistStartM = distanceM();
  if (race.lap >= TOTAL_LAPS) { race.finished = true; race.running = false; race.finishMs = now; }
}

// Geofence on the start/finish line: fire once when entering the zone
void checkGpsLap() {
  if (!gps.location.isValid() || gps.location.age() > 1500) return;
  double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),
                                             START_LAT, START_LON);
  bool inside = dist < START_RADIUS_M;
  if (inside && !race.insideStartZone) markLap();
  race.insideStartZone = inside;
}

float lapDistM() { return distanceM() - race.lapDistStartM; }

// Strategy cue: current plan action and metres until the next change
struct Cue { bool burn; float toNextM; };
Cue planCue() {
  float d = fmodf(lapDistM(), LAP_M);
  int i = 0;
  while (i + 1 < PLAN_STEPS && d >= PLAN[i + 1].fromM) i++;
  float next = (i + 1 < PLAN_STEPS) ? PLAN[i + 1].fromM : LAP_M;
  return { PLAN[i].burn, next - d };
}

// Pace cue. Compare the projected finish time at the current average speed
// with the time limit. Hysteresis stops the LED flickering between states.
enum Pace { COAST, HOLD, BURN };
Pace pace = HOLD;

Pace computePace(float avgKmh) {
  if (!race.running || avgKmh < 1.0f) return HOLD;
  float remainingKm = TOTAL_LAPS * LAP_M / 1000.0f - (distanceM() - race.distStartM) / 1000.0f;
  float elapsedS = (millis() - race.startMs) / 1000.0f;
  float projectedS = elapsedS + (remainingKm / avgKmh) * 3600.0f;
  float margin = TIME_LIMIT_S - projectedS;        // seconds of slack
  switch (pace) {
    case COAST: return (margin < PACE_LOW_S)  ? HOLD : COAST;
    case BURN:  return (margin > PACE_HIGH_S) ? HOLD : BURN;
    default:
      if (margin > PACE_HIGH_S + PACE_HYST_S) return COAST;
      if (margin < PACE_LOW_S  - PACE_HYST_S) return BURN;
      return HOLD;
  }
}

void setPaceLed(Pace p) {
  digitalWrite(PIN_LED_G, p != BURN);
  digitalWrite(PIN_LED_R, p != COAST);
}

// ---------------------------------------------------------------- display
void fmtTime(char* buf, size_t n, uint32_t ms) {
  uint32_t s = ms / 1000;
  snprintf(buf, n, "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
}

void drawRacePage(float avgKmh, uint32_t elapsedMs) {
  char buf[32];
  lcd.clearDisplay();
  lcd.setTextColor(BLACK);

  // Top line: lap and elapsed
  lcd.setTextSize(3);
  snprintf(buf, sizeof buf, "LAP %02u/%02u", race.lap + (race.running ? 1 : 0), TOTAL_LAPS);
  lcd.setCursor(8, 6); lcd.print(buf);
  fmtTime(buf, sizeof buf, elapsedMs);
  lcd.setCursor(290, 6); lcd.print(buf);

  // Speed, large, one decimal
  lcd.setTextSize(9);
  snprintf(buf, sizeof buf, "%4.1f", speedSmooth);
  lcd.setCursor(4, 44); lcd.print(buf);
  lcd.setTextSize(2);
  lcd.setCursor(222, 44); lcd.print("km/h");

  // Strategy cue box, reverse video while burning
  Cue c = planCue();
  if (c.burn) lcd.fillRect(300, 40, 96, 74, BLACK);
  else        lcd.drawRect(300, 40, 96, 74, BLACK);
  lcd.setTextColor(c.burn ? WHITE : BLACK);
  lcd.setTextSize(3);
  lcd.setCursor(306, 50); lcd.print(c.burn ? "BURN" : "GLIDE");
  lcd.setTextSize(2);
  snprintf(buf, sizeof buf, "%4.0fm", c.toNextM);
  lcd.setCursor(318, 88); lcd.print(buf);
  lcd.setTextColor(BLACK);

  // Average vs required minimum, and last-lap delta to target
  lcd.setTextSize(3);
  snprintf(buf, sizeof buf, "AVG %4.1f", avgKmh);
  lcd.setCursor(8, 132); lcd.print(buf);
  snprintf(buf, sizeof buf, "MIN %4.1f", MIN_AVG_KMH);
  lcd.setCursor(222, 132); lcd.print(buf);
  if (race.lastLapMs) {
    int32_t deltaMs = (int32_t)race.lastLapMs - (int32_t)TARGET_LAP_MS;
    snprintf(buf, sizeof buf, "LAST %+5.1fs", deltaMs / 1000.0f);
    lcd.setCursor(8, 166); lcd.print(buf);
  }
  int32_t remS = (int32_t)TIME_LIMIT_S - (int32_t)(elapsedMs / 1000);
  snprintf(buf, sizeof buf, "REM %02ld:%02ld", (long)(remS / 60), (long)(labs(remS) % 60));
  lcd.setCursor(222, 166); lcd.print(buf);

  // Status line
  lcd.setTextSize(2);
  snprintf(buf, sizeof buf, "%s %.2fV SAT%lu %s",
           pace == BURN ? "BURN " : pace == COAST ? "COAST" : "HOLD ",
           batteryVolts(), (unsigned long)gps.satellites.value(), sdOk ? "LOG" : "NO SD");
  lcd.setCursor(8, 218); lcd.print(buf);
  lcd.refresh();   // also flips VCOM in the command byte (EXTMODE tied low)
}

void drawDiagPage() {
  char buf[40];
  lcd.clearDisplay();
  lcd.setTextColor(BLACK);
  lcd.setTextSize(2);
  int y = 10;
  auto line = [&](const char* s) { lcd.setCursor(10, y); lcd.print(s); y += 24; };
  snprintf(buf, sizeof buf, "pulses %lu", (unsigned long)g_pulseCount);        line(buf);
  snprintf(buf, sizeof buf, "period %lu us", (unsigned long)g_periodUs);       line(buf);
  snprintf(buf, sizeof buf, "wheel %.1f  gps %.1f", speedKmh, gps.speed.kmph()); line(buf);
  snprintf(buf, sizeof buf, "dist %.1f m", distanceM());                       line(buf);
  snprintf(buf, sizeof buf, "lat %.5f", gps.location.lat());                   line(buf);
  snprintf(buf, sizeof buf, "lon %.5f", gps.location.lng());                   line(buf);
  snprintf(buf, sizeof buf, "hdop %.1f fix %d", gps.hdop.hdop(), gps.location.isValid()); line(buf);
  snprintf(buf, sizeof buf, "vbat %.2f V", batteryVolts());                    line(buf);
  snprintf(buf, sizeof buf, "sd %s  up %lus", sdOk ? "ok" : "FAIL", (unsigned long)(millis() / 1000)); line(buf);
  lcd.refresh();
}

// ---------------------------------------------------------------- logging
void openLog() {
  if (!SD.begin(PIN_SD_CS, SPI, 20000000)) { sdOk = false; return; }
  char name[20];
  for (int i = 0; i < 999; i++) {
    snprintf(name, sizeof name, "/run%03d.csv", i);
    if (!SD.exists(name)) break;
  }
  logFile = SD.open(name, FILE_WRITE);
  sdOk = (bool)logFile;
  if (sdOk) logFile.println("t_ms,speed_kmh,gps_kmh,dist_m,lap,lap_ms,lat,lon,sats,vbat,pace,plan_burn");
}

void logRow(uint32_t t) {
  if (!sdOk) return;
  logFile.printf("%lu,%.2f,%.2f,%.1f,%u,%lu,%.6f,%.6f,%lu,%.2f,%d,%d\n",
                 (unsigned long)t, speedKmh, gps.speed.kmph(), distanceM(), race.lap,
                 (unsigned long)(race.running ? millis() - race.lapStartMs : 0),
                 gps.location.lat(), gps.location.lng(), (unsigned long)gps.satellites.value(),
                 batteryVolts(), (int)pace, (int)planCue().burn);
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) { logFile.flush(); lastFlush = millis(); }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  pinMode(PIN_PULSE, INPUT_PULLUP);
  pinMode(PIN_BTN_LAP, INPUT_PULLUP);
  pinMode(PIN_BTN_PAGE, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LCD_DISP, OUTPUT);
  digitalWrite(PIN_LCD_DISP, LOW);           // blank until the first frame is drawn
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  attachInterrupt(digitalPinToInterrupt(PIN_PULSE), onWheelPulse, FALLING);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  lcd.begin();
  lcd.clearDisplay();
  lcd.refresh();
  digitalWrite(PIN_LCD_DISP, HIGH);

  gpsSerial.begin(115200, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  openLog();

  // Power-on self test: both LEDs for 500 ms
  setPaceLed(HOLD); delay(500); setPaceLed(COAST);

  // If the loop ever stalls, reboot rather than freeze the display with DC
  // across the panel. Race state is rebuilt from the next lap button press.
  enableLoopWDT();
}

void loop() {
  static uint32_t lastTick = 0, lastDraw = 0;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  btnLap.update(); btnPage.update(); btnReset.update();
  if (btnLap.take())   markLap();
  if (btnPage.take())  page ^= 1;
  if (btnReset.take()) { race = RaceState(); pace = HOLD; }

  uint32_t now = millis();
  if (now - lastTick >= 100) {               // 10 Hz control and logging
    lastTick = now;
    speedKmh = wheelSpeedKmh();
    speedSmooth += 0.3f * (speedKmh - speedSmooth);

    checkGpsLap();

    uint32_t elapsedMs = race.running ? now - race.startMs
                       : race.finished ? race.finishMs - race.startMs : 0;
    float avgKmh = (elapsedMs > 5000)
                   ? (distanceM() - race.distStartM) / 1000.0f / (elapsedMs / 3600000.0f) : 0;
    if (race.finished) avgKmh = TOTAL_LAPS * LAP_M / 1000.0f / (elapsedMs / 3600000.0f);
    pace = computePace(avgKmh);
    setPaceLed(pace);
    logRow(now);

    if (now - lastDraw >= 250) {             // 4 Hz refresh is plenty for a memory LCD
      lastDraw = now;
      if (page == 0) drawRacePage(avgKmh, elapsedMs); else drawDiagPage();
    }
  }
}
