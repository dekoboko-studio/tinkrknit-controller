// Silver Reed knitting machine controller - ESP32 (classic DevKit) port
// Original design: Seeed XIAO RP2040 (knitscape/silver-reed-write)
//
// Pin map (GPIO numbers). Carriage inputs are grouped on the VIN/GND header side.
//   DIN1 ND1  -> GPIO25  (input, external pull-up to 3V3)  needle reference (display only)
//   DIN2 KSL  -> GPIO14  (input, external pull-up to 3V3)  point cam, HIGH = in knitting range
//   DIN3 DOB  <- GPIO26  (output)                          data out to selector
//   DIN4 CCP  -> GPIO27  (input, external pull-up to 3V3)  carriage clock
//   DIN5 HOK  -> GPIO13  (input, external pull-up to 3V3)  direction, LOW = right, HIGH = left
//   DIN6 16V  -> not connected while testing
//   DIN7 5V   -> VIN (USB 5V)
//   DIN8 GND  -> GND
//   OLED SDA  -> GPIO22, SCL -> GPIO21, VCC -> 3V3, GND -> GND
//   GPIO2 (onboard LED) mirrors the OUT pin so you can see data without solenoid power.
//
// ESP32 GPIOs are NOT 5V tolerant: pull the signal lines up to 3V3, never 5V.
// GPIO12 is deliberately unused (strapping pin, a pull-up on it can stop the ESP32 booting).

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int ND1_PIN = 25;
const int CAMS_PIN = 14;
const int OUT_PIN = 26;
const int CLOCK_PIN = 27;
const int DIRECTION_PIN = 13;
const int LED_PIN = 2;
const int EXT_LED_PIN = 32;  // external blue LED, mirrors the onboard LED (solenoid OUT)
const int RED_LED_PIN = 4;   // external red LED, flashes while the ! warning is shown
const int BOOT_BTN_PIN = 0;  // onboard BOOT button, same job as the menu button
const int MENU_BTN_PIN = 33; // external menu button to GND, internal pull-up

const int OLED_SDA = 22;  // swapped to match the as-built wiring
const int OLED_SCL = 21;
const uint8_t OLED_ADDR = 0x3C;

// Protocol constants
// Commands (host -> device)
const byte CMD_SET_ROW = 0x02;        // Set row data: [CMD, length, packed_data...]
const byte CMD_CLEAR_ROW = 0x03;      // Clear current row pattern
const byte CMD_HELLO = 0x10;          // Ask the firmware version: [CMD] -> MSG_HELLO
const byte CMD_PASS_INFO = 0x11;      // About the next pass: [CMD, length, payload...]

// Messages (device -> host)
const byte MSG_ACK_ROW = 0x04;
const byte MSG_ENTER_CAMS = 0x05;
const byte MSG_EXIT_CAMS = 0x06;
const byte MSG_CHANGE_DIRECTION = 0x07;  // [MSG, direction]
const byte MSG_STRING = 0x08;            // [MSG, type, length, message...]
const byte MSG_NEEDLE = 0x09;            // [MSG, needle_index]
const byte MSG_HELLO = 0x10;             // [MSG, version]

// Knit Designer additions (knit-designer public/js/knit/protocol.js). The web app sends
// CMD_PASS_INFO only after MSG_HELLO has reported a version, because older firmware reads
// unknown bytes as commands.
// 1: change signal on the LEDs. 2: feeders are yarn letters (0 = A), shown on the OLED.
const byte FIRMWARE_VERSION = 2;

// CMD_PASS_INFO payload. Length-prefixed so the host can append fields later (a color
// changer) without breaking this sketch: it keeps the bytes it knows and skips the rest.
//   0  flags     PASS_YARN_CHANGE | PASS_CAM_CHANGE | PASS_DONE
//   1  feeder 1  its yarn as lettered on the knit screen, 0 = A, 1 = B..., 0xFF = empty
//   2  feeder 2  likewise
//   3  next      passes from this one to the next change, 0xFF = none ahead
//   4  cam       0 plain, 1 Fair Isle, 2 tuck, 3 slip, 4 jacquard
const byte PASS_YARN_CHANGE = 0x01;   // rethread before this pass
const byte PASS_CAM_CHANGE = 0x02;    // change the carriage setting before this pass
const byte PASS_DONE = 0x04;          // the piece is finished
const int PASS_INFO_MAX = 32;
byte passInfo[PASS_INFO_MAX];
int passInfoLength = 0;

const byte DIR_RIGHT = 0x00;
const byte DIR_LEFT = 0x01;

const byte STRING_INFO = 0x00;
const byte STRING_ERROR = 0x01;

// Pattern storage (packed: 8 needles per byte)
const int MAX_NEEDLE_COUNT = 200;
const int MAX_PACKED_BYTES = 25;
byte pattern[MAX_PACKED_BYTES];
int patternLength = 0;
bool patternReady = false;

inline bool getNeedle(int index) {
  return (pattern[index / 8] >> (index % 8)) & 1;
}

// State variables
int needleCount = 0;
int lastWindowWidth = 0;     // needles counted in the previous full pass (window width)
bool lastClockState = LOW;
bool lastCamsState = LOW;
bool lastDirectionState = LOW;

// Serial state machine
enum SerialState { SERIAL_IDLE, SERIAL_WAITING_LENGTH, SERIAL_WAITING_DATA, SERIAL_WAITING_INFO_LENGTH, SERIAL_WAITING_INFO };
SerialState serialState = SERIAL_IDLE;
byte expectedLength = 0;
byte infoLength = 0;
int infoRead = 0;
unsigned long serialTimeout = 0;
const unsigned long SERIAL_TIMEOUT_MS = 1000;

// Safety: track how long OUT has been on
unsigned long outOnSince = 0;
const unsigned long OUT_MAX_ON_MS = 3000;

// ---- Status shared with the display task (core 0) ----
volatile uint32_t edgesND1 = 0, edgesCAMS = 0, edgesCLK = 0, edgesDIR = 0;
volatile uint32_t lastHostRxMs = 0;      // millis() of last received byte
volatile uint32_t rowsKnit = 0;          // completed rows
volatile uint32_t rowsReceived = 0;      // rows received from the host
volatile int lastNeedleCount = 0;        // needles seen in last/current row
volatile bool outState = false;
volatile bool dirLeft = false;
volatile bool inCams = false;
volatile bool errorFlag = false;
volatile uint32_t passClkIn = 0;   // clock edges/2 seen by interrupt while cams HIGH, last pass
volatile uint32_t passClkOut = 0;  // clock edges/2 seen while cams LOW, between passes
uint32_t clkAtEnter = 0, clkAtExit = 0;
bool patternFromPc = false;     // current pattern was sent by the web app
volatile bool testMode = false;  // built-in 3x3 checkerboard test pattern
char lastEvent[24] = "boot";

volatile uint32_t badRows = 0;     // rows whose needle count differed from the previous row

// What the knitter has to do before the next pass, from CMD_PASS_INFO. Set when the web app
// sends the pass, cleared when the carriage enters the cams (the knitter has made the change
// and started knitting) or the pattern is cleared.
enum ChangeSignal { CHANGE_NONE, CHANGE_YARN, CHANGE_CAM, CHANGE_DONE };
volatile ChangeSignal changeSignal = CHANGE_NONE;
// The pass's yarns and cam from the same message, copied out for the display task (core 0)
volatile byte changeFeeder1 = 0xFF;
volatile byte changeFeeder2 = 0xFF;
volatile byte changeCam = 0;
volatile bool changeCamToo = false;  // a yarn change that also needs the cam changed

// Glitch filter: a level only counts once it has held steady for holdUs.
// Spikes shorter than that (e.g. from solenoid switching) are counted as glitches and ignored.
struct Debounced {
  int pin;
  uint32_t holdUs;
  int stable;
  int raw;
  uint32_t since;
  volatile uint32_t glitches;

  void init() {
    raw = stable = digitalRead(pin);
    since = micros();
  }

  int read() {
    int r = digitalRead(pin);
    uint32_t now = micros();
    if (r != raw) {
      if (raw != stable) glitches++;  // a pending change reverted before it settled
      raw = r;
      since = now;
    }
    if (raw != stable && now - since >= holdUs) stable = raw;
    return stable;
  }
};
Debounced clockIn = {CLOCK_PIN, 200, LOW, LOW, 0, 0};
Debounced camsIn = {CAMS_PIN, 1000, LOW, LOW, 0, 0};
Debounced dirIn = {DIRECTION_PIN, 1000, LOW, LOW, 0, 0};

void IRAM_ATTR isrND1() { edgesND1++; }
void IRAM_ATTR isrCAMS() { edgesCAMS++; }
void IRAM_ATTR isrCLK() { edgesCLK++; }
void IRAM_ATTR isrDIR() { edgesDIR++; }

void noteEvent(const char* msg) {
  strncpy(lastEvent, msg, sizeof(lastEvent) - 1);
  lastEvent[sizeof(lastEvent) - 1] = 0;
}

void sendInfo(const char* message);

// Checkerboard of 3x3 squares: 3 needles ON / 3 OFF, flipping every 3 rows.
void loadTestPattern(bool left) {
  memset(pattern, 0, sizeof(pattern));
  for (int b = 0; b < MAX_NEEDLE_COUNT; b++) {
    int i = b + 1;  // bit b is used by clock pulse i (1-based needle from the entering end)
    // Needles are counted from the end the carriage enters, so mirror left passes
    // to keep the squares on the same physical needles in both directions.
    int w = lastWindowWidth > 0 ? lastWindowWidth : MAX_NEEDLE_COUNT;
    int pos = (left && i <= w) ? (w + 1 - i) : i;
    if ((((pos + 2) / 3) + (rowsKnit / 3)) % 2 == 0) pattern[b / 8] |= (1 << (b % 8));
  }
  patternLength = MAX_NEEDLE_COUNT;
  patternReady = true;
  patternFromPc = false;
}

// Screens: knitting view (default) and diagnostics
enum Screen { SCREEN_KNIT, SCREEN_DIAG };
volatile Screen screen = SCREEN_KNIT;
const uint32_t LONG_PRESS_MS = 1000;
const uint32_t TAP_MAX_MS = 400;   // a press shorter than this counts as a tap
const uint32_t TAP_GAP_MS = 500;   // taps closer together than this form one burst
const int DIAG_TAPS = 4;
Debounced bootBtn = {BOOT_BTN_PIN, 30000, HIGH, HIGH, 0, 0};
Debounced menuBtn = {MENU_BTN_PIN, 30000, HIGH, HIGH, 0, 0};

void toggleTestMode() {
  testMode = !testMode;
  if (testMode) {
    noteEvent("TEST MODE ON");
  } else {
    patternReady = false;
    setOut(LOW);
    noteEvent("Test mode off");
  }
}

// Menu button on GPIO33 or BOOT (both active low).
// Knitting screen: hold 1s resets the row counter straight away (display only, PC knitting
// unaffected) and the LEDs double flash to confirm; 4 quick taps switch to diagnostics.
// Diagnostics screen: a fresh tap goes back; hold 1s toggles the test pattern.
bool btnDown = false;
bool btnLongFired = false;        // the current press already triggered its hold action
uint32_t resetFlashAt = 0;        // millis() of the last row reset, drives the double flash
void checkBootButton() {
  static int lastBtn = HIGH;
  static uint32_t pressedAt = 0;
  static uint32_t lastTapAt = 0;
  static int tapCount = 0;
  int btn = (bootBtn.read() == LOW || menuBtn.read() == LOW) ? LOW : HIGH;
  uint32_t now = millis();
  if (btn == LOW && lastBtn == HIGH) {
    pressedAt = now;
    btnLongFired = false;
  }
  if (btn == LOW && !btnLongFired && now - pressedAt >= LONG_PRESS_MS) {
    if (screen == SCREEN_DIAG) {
      toggleTestMode();
    } else {
      rowsKnit = 0;
      rowsReceived = 0;
      noteEvent("Rows reset");
      resetFlashAt = now;
    }
    btnLongFired = true;
    tapCount = 0;
  }
  if (btn == HIGH && lastBtn == LOW && !btnLongFired) {
    if (now - pressedAt < TAP_MAX_MS) {
      // Taps in the same burst keep counting, so a 5th tap after entering diagnostics
      // doesn't immediately bounce back out
      if (now - lastTapAt > TAP_GAP_MS) tapCount = 0;
      tapCount++;
      lastTapAt = now;
      if (screen == SCREEN_KNIT && tapCount == DIAG_TAPS) {
        screen = SCREEN_DIAG;
      } else if (screen == SCREEN_DIAG && tapCount == 1) {
        screen = SCREEN_KNIT;
      }
    } else {
      tapCount = 0;
    }
  }
  lastBtn = btn;
  btnDown = (btn == LOW);
}

// External LEDs (blue + red share one button, so both on reads as magenta). Priority:
// row reset -> two magenta flashes (starts the moment the 1s hold fires, even if still held);
// button held (before its hold action fires) -> magenta; ! warning -> red flashing, blue off;
// change pending between passes -> red and blue alternate every 500ms (onboard LED with blue);
// otherwise blue mirrors the solenoid output. Blue is dimmed whenever red is on with it so the
// magenta doesn't wash out to blue; the change alternation dims blue the same way so the
// two colors look equally bright. The change signal outranks the ! warning; the OLED still
// shows the ! while the LEDs are busy with the change.
const uint32_t CHANGE_BLINK_MS = 500;
const int BLUE_FULL = 255;
const int BLUE_WITH_RED = 64;  // PWM duty out of 255; ~25% duty looks about half as bright
void updateLeds() {
  uint32_t now = millis();
  bool blue, red;
  uint32_t sinceReset = now - resetFlashAt;
  if (resetFlashAt && sinceReset < 600) {
    bool on = (sinceReset / 150) % 2 == 1;  // off, on, off, on
    blue = red = on;
  } else if (btnDown && !btnLongFired) {
    blue = red = true;
  } else if (changeSignal != CHANGE_NONE && !inCams) {
    red = (now / CHANGE_BLINK_MS) % 2 == 0;
    blue = !red;
  } else if (errorFlag) {
    blue = false;
    red = (now / 250) % 2 == 0;
  } else {
    blue = outState;
    red = false;
  }
  // The onboard LED mirrors OUT (setOut); between passes it blinks with blue for a change
  static bool onboardBlinking = false;
  bool changeShowing = changeSignal != CHANGE_NONE && !inCams;
  if (changeShowing) {
    digitalWrite(LED_PIN, blue);
    onboardBlinking = true;
  } else if (onboardBlinking) {
    digitalWrite(LED_PIN, outState);
    onboardBlinking = false;
  }
  // Only touch the PWM when the level changes; this runs every pass of the needle loop
  static int lastBlueDuty = -1;
  int blueDuty = !blue ? 0 : ((red || changeShowing) ? BLUE_WITH_RED : BLUE_FULL);
  if (blueDuty != lastBlueDuty) {
    analogWrite(EXT_LED_PIN, blueDuty);
    lastBlueDuty = blueDuty;
  }
  digitalWrite(RED_LED_PIN, red);
}

void setOut(int value) {
  digitalWrite(OUT_PIN, value);
  digitalWrite(LED_PIN, value);
  outState = (value == HIGH);
  if (value == HIGH) {
    if (outOnSince == 0) outOnSince = millis();
  } else {
    outOnSince = 0;
  }
}

void checkOutSafety() {
  if (outOnSince > 0 && millis() - outOnSince >= OUT_MAX_ON_MS) {
    setOut(LOW);
    sendInfo("SAFETY: Turned OUT off.");
    noteEvent("SAFETY: OUT off");
    errorFlag = true;
  }
}

void sendString(byte type, const char* message) {
  byte len = strlen(message);
  Serial.write(MSG_STRING);
  Serial.write(type);
  Serial.write(len);
  Serial.write((const uint8_t*)message, len);
}

void sendError(const char* message) {
  sendString(STRING_ERROR, message);
  noteEvent(message);
  errorFlag = true;
}

void sendInfo(const char* message) {
  sendString(STRING_INFO, message);
}

void sendDirection(byte direction) {
  Serial.write(MSG_CHANGE_DIRECTION);
  Serial.write(direction);
}

void sendNeedle(byte needleIndex) {
  Serial.write(MSG_NEEDLE);
  Serial.write(needleIndex);
}

void sendAckRow(byte length) {
  Serial.write(MSG_ACK_ROW);
  Serial.write(length);
}

void sendEnterCams() { Serial.write(MSG_ENTER_CAMS); }
void sendExitCams() { Serial.write(MSG_EXIT_CAMS); }

void applyPassInfo() {
  byte flags = passInfoLength > 0 ? passInfo[0] : 0;
  if (flags & PASS_DONE) changeSignal = CHANGE_DONE;
  else if (flags & PASS_YARN_CHANGE) changeSignal = CHANGE_YARN;
  else if (flags & PASS_CAM_CHANGE) changeSignal = CHANGE_CAM;
  else changeSignal = CHANGE_NONE;
  changeFeeder1 = passInfoLength > 1 ? passInfo[1] : 0xFF;
  changeFeeder2 = passInfoLength > 2 ? passInfo[2] : 0xFF;
  changeCam = passInfoLength > 4 ? passInfo[4] : 0;
  changeCamToo = (flags & PASS_YARN_CHANGE) && (flags & PASS_CAM_CHANGE);
  if (changeSignal == CHANGE_YARN) noteEvent("Change yarn");
  else if (changeSignal == CHANGE_CAM) noteEvent("Change carriage");
  else if (changeSignal == CHANGE_DONE) noteEvent("Piece done");
}

void processSerial() {
  if (Serial.available() > 0) {
    lastHostRxMs = millis();
    if (lastHostRxMs == 0) lastHostRxMs = 1;
  }

  if (serialState != SERIAL_IDLE) {
    if (millis() - serialTimeout >= SERIAL_TIMEOUT_MS) {
      sendError("Serial timeout");
      int flushed = 0;
      while (Serial.available() > 0) {
        Serial.read();
        flushed++;
      }
      if (flushed > 0) sendInfo("Flushed stale bytes");
      serialState = SERIAL_IDLE;
      return;
    }
  }

  switch (serialState) {
    case SERIAL_IDLE:
      if (Serial.available() < 1) return;
      {
        byte cmd = Serial.read();
        if (cmd == CMD_SET_ROW) {
          serialState = SERIAL_WAITING_LENGTH;
          serialTimeout = millis();
        } else if (cmd == CMD_HELLO) {
          Serial.write(MSG_HELLO);
          Serial.write(FIRMWARE_VERSION);
        } else if (cmd == CMD_PASS_INFO) {
          serialState = SERIAL_WAITING_INFO_LENGTH;
          serialTimeout = millis();
        } else if (cmd == CMD_CLEAR_ROW) {
          patternReady = false;
          patternLength = 0;
          changeSignal = CHANGE_NONE;
          sendInfo("Pattern cleared");
          noteEvent("Pattern cleared");
        } else {
          sendError("Unknown command");
        }
      }
      break;

    case SERIAL_WAITING_LENGTH:
      if (Serial.available() < 1) return;
      expectedLength = Serial.read();
      if (expectedLength > MAX_NEEDLE_COUNT) {
        sendError("Pattern too long");
        while (Serial.available() > 0) Serial.read();
        serialState = SERIAL_IDLE;
        return;
      }
      serialState = SERIAL_WAITING_DATA;
      serialTimeout = millis();
      break;

    case SERIAL_WAITING_DATA:
      {
        int packedLength = (expectedLength + 7) / 8;
        if (Serial.available() < packedLength) return;
        for (int i = 0; i < packedLength; i++) pattern[i] = Serial.read();
        // A row from the PC always takes priority over the built-in test pattern
        if (testMode) testMode = false;
        patternLength = expectedLength;
        patternReady = true;
        patternFromPc = true;
        rowsReceived++;
        sendAckRow(expectedLength);
        noteEvent("Row received");
        serialState = SERIAL_IDLE;
      }
      break;

    case SERIAL_WAITING_INFO_LENGTH:
      if (Serial.available() < 1) return;
      infoLength = Serial.read();
      infoRead = 0;
      passInfoLength = 0;
      if (infoLength == 0) {
        applyPassInfo();
        serialState = SERIAL_IDLE;
        return;
      }
      serialState = SERIAL_WAITING_INFO;
      serialTimeout = millis();
      break;

    case SERIAL_WAITING_INFO:
      // Bytes past what this sketch keeps are read and dropped, so a newer host's longer
      // payload never desyncs the stream
      while (Serial.available() > 0 && infoRead < infoLength) {
        byte b = Serial.read();
        if (infoRead < PASS_INFO_MAX) {
          passInfo[infoRead] = b;
          passInfoLength = infoRead + 1;
        }
        infoRead++;
      }
      if (infoRead >= infoLength) {
        applyPassInfo();
        serialState = SERIAL_IDLE;
      }
      break;
  }
}

// ---------------- OLED status display (runs on core 0) ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayOk = false;

// "12s", "3m", "--" style age text
void ageText(char* out, size_t n, uint32_t stamp) {
  if (stamp == 0) { snprintf(out, n, "never"); return; }
  uint32_t s = (millis() - stamp) / 1000;
  if (s < 1) snprintf(out, n, "now");
  else if (s < 100) snprintf(out, n, "%us ago", (unsigned)s);
  else snprintf(out, n, "%um ago", (unsigned)(s / 60));
}

// Large-print knitting view: direction, row count, needles in the current row
void drawKnitScreen(bool hostBlip) {
  // Status icons along the top left. Dot = data just received from the PC,
  // inverted T = test pattern on, ! = last row had a problem
  display.setTextSize(1);
  if (hostBlip) display.fillCircle(3, 4, 3, SSD1306_WHITE);
  if (testMode) {
    display.fillRect(12, 0, 7, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(13, 1);
    display.print("T");
    display.setTextColor(SSD1306_WHITE);
  }
  if (errorFlag) {
    display.setCursor(25, 1);
    display.print("!");
  }

  // Direction arrow under the icons: filled while in the knitting range, outline otherwise
  const int ax = 2, ay = 12, aw = 22, ah = 18;
  int tipX = dirLeft ? ax : ax + aw;
  int baseX = dirLeft ? ax + aw : ax;
  if (inCams) {
    display.fillTriangle(tipX, ay + ah / 2, baseX, ay, baseX, ay + ah, SSD1306_WHITE);
  } else {
    display.drawTriangle(tipX, ay + ah / 2, baseX, ay, baseX, ay + ah, SSD1306_WHITE);
  }

  // Row label top right, count right-aligned beneath it
  char num[12];
  snprintf(num, sizeof(num), "%u", (unsigned)rowsKnit);
  display.setCursor(110, 0);
  display.print("ROW");
  display.setTextSize(3);
  display.setCursor(128 - (int)strlen(num) * 18, 9);
  display.print(num);

  // Needle count on its own line, centred. Between passes a pending change takes the line:
  // the yarn letters to thread ("YARN C", or "1:A  2:C" for two feeders), the cam to set,
  // or DONE. A yarn and cam change together alternate every second.
  int total = lastWindowWidth;
  ChangeSignal change = inCams ? CHANGE_NONE : changeSignal;
  bool showCam = change == CHANGE_CAM || (change == CHANGE_YARN && changeCamToo && (millis() / 1000) % 2 == 1);
  byte f1 = changeFeeder1, f2 = changeFeeder2;
  auto letter = [](byte yarn) -> char { return yarn < 26 ? 'A' + yarn : '?'; };
  if (change == CHANGE_DONE) {
    snprintf(num, sizeof(num), "DONE");
  } else if (showCam) {
    static const char* const cams[] = {"PLAIN", "FAIR ISLE", "TUCK", "SLIP", "JACQUARD"};
    snprintf(num, sizeof(num), "%s", changeCam < 5 ? cams[changeCam] : "SET CAM");
  } else if (change == CHANGE_YARN && f2 == 0xFF) {
    snprintf(num, sizeof(num), "YARN %c", letter(f1));
  } else if (change == CHANGE_YARN) {
    snprintf(num, sizeof(num), "1:%c  2:%c", letter(f1), letter(f2));
  } else if (total > 0) {
    snprintf(num, sizeof(num), "%d/%d", (int)lastNeedleCount, total);
  } else {
    snprintf(num, sizeof(num), "%d", (int)lastNeedleCount);
  }
  display.setTextSize(2);
  display.setCursor((128 - (int)strlen(num) * 12) / 2, 34);
  display.print(num);

  // Full-width progress bar that fills the way the carriage is travelling
  const int bx = 0, by = 52, bw = 128, bh = 12;
  if (total > 0) {
    display.drawRect(bx, by, bw, bh, SSD1306_WHITE);
    int fill = min((int)lastNeedleCount, total) * (bw - 4) / total;
    int fx = dirLeft ? bx + bw - 2 - fill : bx + 2;
    display.fillRect(fx, by + 2, fill, bh - 4, SSD1306_WHITE);
  }
}

// Detailed diagnostics view (4 quick taps to enter, tap to leave, hold here to toggle the test pattern)
void drawDiagScreen(bool hostSeen, bool hostFresh, bool carrActive, uint32_t lastCarriageMs) {
  char age[16], buf[32];
  display.setTextSize(1);

  display.setCursor(0, 0);
  ageText(age, sizeof(age), lastHostRxMs);
  display.printf("PC  : %s %s", hostFresh ? "LINK" : (hostSeen ? "idle" : "none"), age);

  display.setCursor(0, 9);
  ageText(age, sizeof(age), lastCarriageMs);
  display.printf("CARR: %s %s", carrActive ? "ACTIVE" : "quiet", age);

  display.setCursor(0, 18);
  display.printf("Dir:%s Cam:%s", dirLeft ? "<-LEFT" : "RIGHT->", inCams ? "IN" : "out");

  display.setCursor(0, 27);
  display.printf("Ndl : %d/%d  Out:%d", (int)lastNeedleCount, patternLength, outState ? 1 : 0);

  display.setCursor(0, 36);
  // R: rows knit / rows received. Kept short so 3-digit counts still fit in 21 characters
  display.printf("Pat : %s R:%u/%u", testMode ? "TEST" : (patternReady ? "ready" : "none"),
                 (unsigned)rowsKnit, (unsigned)rowsReceived);

  // live pin levels + edge counters (proves each wire is toggling)
  display.setCursor(0, 45);
  // in = raw clock pulses last pass, g = clock glitches filtered, b = rows with a changed count
  display.printf("K%d C%d D%d in%u g%u b%u", camsIn.stable, clockIn.stable, dirIn.stable,
                 (unsigned)passClkIn, (unsigned)clockIn.glitches, (unsigned)badRows);

  display.setCursor(0, 54);
  snprintf(buf, sizeof(buf), "%s", lastEvent);
  display.print(errorFlag ? "!" : ">");
  display.print(buf);
}

void displayTask(void*) {
  uint32_t prevND1 = 0, prevCAMS = 0, prevCLK = 0, prevDIR = 0;
  uint32_t lastCarriageMs = 0;
  for (;;) {
    uint32_t n1 = edgesND1, c = edgesCAMS, k = edgesCLK, d = edgesDIR;
    if (n1 != prevND1 || c != prevCAMS || k != prevCLK || d != prevDIR) {
      lastCarriageMs = millis();
      if (lastCarriageMs == 0) lastCarriageMs = 1;
      prevND1 = n1; prevCAMS = c; prevCLK = k; prevDIR = d;
    }

    bool hostSeen = lastHostRxMs != 0;
    bool hostFresh = hostSeen && (millis() - lastHostRxMs < 15000);
    // Brief activity blip each time the PC sends data (the web app only sends rows while patterning)
    bool hostBlip = hostSeen && (millis() - lastHostRxMs < 1500);
    bool carrActive = lastCarriageMs != 0 && (millis() - lastCarriageMs < 3000);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    if (screen == SCREEN_KNIT) {
      drawKnitScreen(hostBlip);
    } else {
      drawDiagScreen(hostSeen, hostFresh, carrActive, lastCarriageMs);
    }
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(CAMS_PIN, INPUT);
  pinMode(CLOCK_PIN, INPUT);
  pinMode(DIRECTION_PIN, INPUT);
  pinMode(ND1_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(EXT_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  bootBtn.init();
  pinMode(MENU_BTN_PIN, INPUT_PULLUP);
  menuBtn.init();
  setOut(LOW);

  clockIn.init();
  camsIn.init();
  dirIn.init();
  lastClockState = clockIn.stable;
  lastCamsState = camsIn.stable;
  lastDirectionState = dirIn.stable;

  attachInterrupt(digitalPinToInterrupt(ND1_PIN), isrND1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CAMS_PIN), isrCAMS, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), isrCLK, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DIRECTION_PIN), isrDIR, CHANGE);

  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (displayOk) {
    // Clip overlong lines at the right edge instead of wrapping them over the next line
    display.setTextWrap(false);
    display.clearDisplay();
    display.display();
    xTaskCreatePinnedToCore(displayTask, "oled", 4096, nullptr, 1, nullptr, 0);
  }

  sendInfo("Starting ESP32 interface with web communication...");
  sendInfo("Waiting for pattern data...");
}

void loop() {
  processSerial();
  checkBootButton();

  int currentCams = camsIn.read();
  int currentDirection = dirIn.read();
  int currentClock = clockIn.read();

  dirLeft = (currentDirection == HIGH);
  inCams = (currentCams == HIGH);

  if (currentDirection != lastDirectionState) {
    sendDirection(currentDirection == HIGH ? DIR_LEFT : DIR_RIGHT);
  }

  // CAMS rising edge (entering knitting range)
  if (currentCams == HIGH && lastCamsState == LOW) {
    sendEnterCams();
    clkAtEnter = edgesCLK;
    passClkOut = (clkAtEnter - clkAtExit) / 2;
    needleCount = 0;
    lastNeedleCount = 0;
    changeSignal = CHANGE_NONE;  // knitting has begun, so the change is made
    noteEvent(testMode ? "TEST: Enter cams" : "Enter cams");
    if (testMode) loadTestPattern(currentDirection == HIGH);
    if (!patternReady) sendError("No pattern ready");
  }

  // CAMS falling edge (exiting knitting range)
  if (currentCams == LOW && lastCamsState == HIGH) {
    char buf[40];
    snprintf(buf, sizeof(buf), "Finished row, saw %d needles", needleCount);
    sendInfo(buf);
    // A changing count with fixed markers means missed or extra clock pulses (noise)
    bool countChanged = lastWindowWidth > 0 && needleCount != lastWindowWidth;
    // The web app mirrors rows assuming the point cams span exactly its row width
    bool widthMismatch = patternReady && patternFromPc && needleCount != patternLength;
    if (widthMismatch) {
      snprintf(buf, sizeof(buf), "CAMS %d != ROW %d", needleCount, patternLength);
    } else if (countChanged) {
      snprintf(buf, sizeof(buf), "COUNT %d (was %d)", needleCount, lastWindowWidth);
    } else {
      snprintf(buf, sizeof(buf), "Row done: %d needles", needleCount);
    }
    noteEvent(buf);
    errorFlag = countChanged || widthMismatch;
    if (countChanged) badRows++;
    lastWindowWidth = needleCount;
    rowsKnit++;
    clkAtExit = edgesCLK;
    passClkIn = (clkAtExit - clkAtEnter) / 2;
    setOut(LOW);
    sendExitCams();
    patternReady = false;
  }

  if (currentCams == HIGH && patternReady) {
    if (currentClock == HIGH && lastClockState == LOW) {
      needleCount++;
      lastNeedleCount = needleCount;
      sendNeedle(needleCount);

      // Clock pulse N (1-based) selects pattern bit N-1. The web app numbers needles from 0;
      // reading bit N skipped pixel 0 and shifted alternate rows 2 needles apart.
      if (needleCount <= patternLength) {
        setOut(getNeedle(needleCount - 1) ? HIGH : LOW);
      } else {
        setOut(LOW);
      }
    }
  } else if (currentCams == HIGH && currentClock == HIGH && lastClockState == LOW) {
    // No pattern: still count needles so the display proves the clock is working
    needleCount++;
    lastNeedleCount = needleCount;
  }

  lastCamsState = currentCams;
  lastDirectionState = currentDirection;
  lastClockState = currentClock;

  checkOutSafety();
  updateLeds();
}
