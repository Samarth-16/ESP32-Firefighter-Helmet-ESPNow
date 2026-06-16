/*
  Modified from user's sketch to show only one screen at a time (LOCAL or PEER)
  Toggle between LOCAL <-> PEER with a push-button. Shows a "THE CONNECTION HAS
  LOST WITH PEER" message when peer hasn't been heard from in PEER_TIMEOUT_MS.

  Button wiring: connect one side of a momentary push button to the chosen
  BUTTON_PIN (default GPIO 15) and the other side to GND. We use INPUT_PULLUP
  so button press is LOW.
*/

#include <WiFi.h>
#include <esp_now.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DHT.h>
#define DHTPIN   4
#define DHTTYPE  DHT11

#include <MAX30105.h>       // SparkFun MAX3010x (works with MAX30102)
#include "heartRate.h"      // from SparkFun examples (for checkForBeat)

// ----------------- CONFIG PER DEVICE -----------------
uint8_t peerMac[] = {0xFC,0xE8,0xC0,0xE0,0xA8,0xE8}; // <-- set to OTHER ESP32's MAC

#define SEND_MS          200   // <-- on the other ESP32, use 250 (or a different value)
#define OLED_REFRESH_MS  250
// -----------------------------------------------------

// MQ-9 gas sensor analog pin and ADC config
#define MQ9_ADC_PIN      34    // ADC1_CH6 (input-only)
const unsigned long MQ9_WARMUP_MS = 60UL * 1000; // 1 minute warm-up (increase if desired)
unsigned long bootMs = 0;

// OLED (SSD1306 128x64 at I2C addr 0x3C)
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// I2C pins: ESP32 default SDA=21, SCL=22 (Wire.begin() uses those)

// Sensor instances
DHT dht(DHTPIN, DHTTYPE);
MAX30105 particleSensor;

// ----------------- Data packet -----------------
struct SensorPacket {
  float    tempC;        // DHT11
  float    humidity;     // DHT11
  uint16_t mq9_raw;      // 0-4095
  float    mq9_v;        // volts (approx)
  uint16_t hr_bpm;       // 0 if unknown
  uint8_t  spo2;         // 0 if unknown (not computed here)
  uint32_t red;          // debug avg
  uint32_t ir;           // debug avg
  uint32_t seq;          // sequence
};

// Shared state
volatile SensorPacket lastFromPeer{};
volatile bool havePeerData = false;

SensorPacket lastLocal{};     // last local packet we sent (for OLED)
uint32_t lastPeerMillis = 0;
uint32_t seq = 0;
unsigned long lastSend = 0;
unsigned long lastOled = 0;

// ----------------- View / Button config -----------------
// Button toggles between LOCAL (0) and PEER (1)
const int BUTTON_PIN = 15;         // wire button to GND and this pin (uses INPUT_PULLUP)
const unsigned long DEBOUNCE_MS = 50;

volatile uint8_t viewMode = 0;     // 0 = LOCAL, 1 = PEER
unsigned long lastButtonChange = 0;
int lastButtonState = HIGH;

// Peer timeout: if no packet within this many ms, consider connection lost
const unsigned long PEER_TIMEOUT_MS = 5000;

// ----------------- Helpers -----------------
static inline bool mq9Ready() {
  return (millis() - bootMs) >= MQ9_WARMUP_MS;
}

// NEW callback signatures for ESP32 core 3.x
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  (void)info; // info->src_addr available if needed
  if (len == sizeof(SensorPacket)) {
    memcpy((void*)&lastFromPeer, incomingData, sizeof(SensorPacket));
    havePeerData = true;
    lastPeerMillis = millis();
  }
}

void readDHT11(float &tC, float &h) {
  tC = dht.readTemperature();  // Celsius
  h  = dht.readHumidity();
  if (isnan(tC) || isnan(h)) { tC = NAN; h = NAN; }
}

// Average several ADC samples for stability
uint16_t readMQ9Raw() {
  const int N = 16;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(MQ9_ADC_PIN);
    delay(2);
  }
  return acc / N; // 0–4095 (12-bit)
}

// Approximate ADC->voltage (with 11 dB attenuation ~ 0-3.3 V effective scale)
float adcToVolts(uint16_t raw) {
  return (raw / 4095.0f) * 3.3f; // good enough for display/telemetry
}

// Quick MAX30102 sampler (RED/IR average + crude BPM). Finger must be steady.
bool readMAX30102(uint16_t &bpm, uint8_t &spo2, uint32_t &redAvg, uint32_t &irAvg) {
  const int NUM_SAMPLES = 60;
  uint32_t redSum = 0, irSum = 0;
  int valid = 0;

  // Gather a short burst for averages
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (particleSensor.available()) {
      uint32_t ir  = particleSensor.getIR();
      uint32_t red = particleSensor.getRed();
      particleSensor.nextSample();
      if (ir > 5000 && red > 5000) { redSum += red; irSum += ir; valid++; }
    } else {
      delay(2);
    }
  }

  if (valid < 10) { bpm = 0; spo2 = 0; redAvg = 0; irAvg = 0; return false; }

  redAvg = redSum / valid;
  irAvg  = irSum  / valid;

  // Tiny window beat detection (~1.2 s)
  int beats = 0;
  unsigned long start = millis();
  const unsigned long windowMs = 1200;
  while (millis() - start < windowMs) {
    if (particleSensor.available()) {
      uint32_t ir = particleSensor.getIR();
      particleSensor.nextSample();
      if (checkForBeat(ir)) beats++;
    } else {
      delay(2);
    }
  }

  bpm = (beats * 60000UL) / windowMs;
  if (bpm < 30 || bpm > 220) bpm = 0;

  spo2 = 0; // full SpO2 algorithm not included here
  return true;
}

SensorPacket makeLocalPacket() {
  SensorPacket p{};

  readDHT11(p.tempC, p.humidity);

  // MQ-9: if not warmed up, we still read; display will show "warming"
  p.mq9_raw = readMQ9Raw();
  p.mq9_v   = adcToVolts(p.mq9_raw);

  uint16_t bpm=0; uint8_t s=0; uint32_t redA=0, irA=0;
  readMAX30102(bpm, s, redA, irA);
  p.hr_bpm = bpm;
  p.spo2   = s;
  p.red    = redA;
  p.ir     = irA;

  p.seq = seq++;
  return p;
}

// ----------------- OLED rendering (single-screen only) -----------------
static void printTH(const char* label, float tC, float h) {
  display.print(label);
  display.print(' ');
  if (isnan(tC) || isnan(h)) display.println("--/--");
  else {
    display.print(tC, 1); display.print("C ");
    display.print((int)h); display.println('%');
  }
}

static void printMQ9Line(bool ready, uint16_t raw, float volts) {
  display.print("MQ9:");
  if (!ready) {
    display.println(" warming");
  } else {
    display.print(raw);
    display.print(" ");
    display.print(volts, 2);
    display.println("V");
  }
}

static void printVitals(uint16_t hr, uint8_t spo2) {
  display.print("HR:");
  if (hr == 0) display.print("--");
  else display.print(hr);
  display.print("  SpO2:");
  if (spo2 == 0) display.println("--");
  else { display.print(spo2); display.println('%'); }
}

// Draw only the LOCAL data in a clean single-screen layout
void drawLocal(const SensorPacket &local) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("LOCAL");
  display.println();

  printTH("T/H:", local.tempC, local.humidity);
  printMQ9Line(mq9Ready(), local.mq9_raw, local.mq9_v);

  // Line break then vitals centered-ish
  display.println();
  printVitals(local.hr_bpm, local.spo2);

  display.display();
}

// Draw only the PEER data or an explicit connection-lost message
void drawPeer(const SensorPacket &peer, bool peerValid, unsigned long ageMs) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (!peerValid) {
    display.setCursor(0, 0);
    display.println("PEER: waiting for data...");
    display.display();
    return;
  }

  // If peer hasn't been heard from recently, show lost message
  if (ageMs > PEER_TIMEOUT_MS) {
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("THE CONNECTION HAS LOST WITH PEER");
    display.println();
    display.print("Last seen: ");
    display.print(ageMs/1000);
    display.println("s ago");
    display.display();
    return;
  }

  display.setCursor(0, 0);
  display.print("PEER #"); display.print(peer.seq);
  display.print("  "); display.print(ageMs/1000); display.println("s");
  display.println();

  printTH("T/H:", peer.tempC, peer.humidity);
  display.print("MQ9: "); display.print(peer.mq9_raw);
  display.print(" "); display.print(peer.mq9_v, 2); display.println("V");
  display.println();
  printVitals(peer.hr_bpm, peer.spo2);

  display.display();
}

// ----------------- Setup helpers -----------------
void setupMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found; proceeding without it.");
    return;
  }
  // Reasonable HR settings
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x24);
}

void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;     // use current channel
  peer.encrypt = false; // set to true if you later add PMK/LMK keys
  if (!esp_now_is_peer_exist(mac)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  }
}

// ----------------- Button handling -----------------
void handleButton() {
  int v = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  if (v != lastButtonState) {
    lastButtonChange = now;
    lastButtonState = v;
  }
  // only toggle when stable and pressed (LOW)
  if ((now - lastButtonChange) > DEBOUNCE_MS) {
    static int lastStable = HIGH;
    if (v != lastStable) {
      lastStable = v;
      if (v == LOW) {
        // pressed: toggle view
        viewMode = (viewMode == 0) ? 1 : 0;
      }
    }
  }
}

// ----------------- Arduino setup/loop -----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  bootMs = millis();

  // I2C (SDA=21, SCL=22 by default)
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP-NOW OLED Ready");
    display.display();
  }

  // DHT + MAX30102
  dht.begin();
  setupMAX30102();

  // ADC config for MQ-9 on GPIO 34
  analogReadResolution(12);                             // 0-4095
  analogSetPinAttenuation(MQ9_ADC_PIN, ADC_11db);       // ~0-3.3 V effective range
  pinMode(MQ9_ADC_PIN, INPUT);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(BUTTON_PIN);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  addPeer(peerMac);

  Serial.println("Exchanging data + showing LOCAL or PEER on OLED (toggle with button)...");
}

void loop() {
  // handle button presses
  handleButton();

  // Periodic send
  if (millis() - lastSend >= SEND_MS) {
    SensorPacket out = makeLocalPacket();
    lastLocal = out; // save for OLED
    esp_now_send(peerMac, (uint8_t*)&out, sizeof(out));
    lastSend = millis();
  }

  // Snapshot new peer packet if available (safe copy from volatile)
  static SensorPacket peerCopy{};
  static bool peerValid = false;

  if (havePeerData) {
    noInterrupts();
    SensorPacket tmp;
    memcpy(&tmp, (const void*)&lastFromPeer, sizeof(SensorPacket));
    havePeerData = false;
    interrupts();

    peerCopy = tmp;
    peerValid = true;
  }

  // OLED refresh
  if (millis() - lastOled >= OLED_REFRESH_MS) {
    uint32_t age = peerValid ? (millis() - lastPeerMillis) : 0;
    // show only one screen at a time based on viewMode
    if (viewMode == 0) {
      drawLocal(lastLocal);
    } else {
      drawPeer(peerCopy, peerValid, age);
    }
    lastOled = millis();
  }
}

