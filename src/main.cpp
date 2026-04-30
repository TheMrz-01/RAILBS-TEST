#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

struct MotorDriver {
  const char *name;
  uint8_t rpwmPin;
  uint8_t lpwmPin;
  uint8_t rEnablePin;
  uint8_t lEnablePin;
  uint8_t rCurrentPin;
  uint8_t lCurrentPin;
  uint8_t rpwmChannel;
  uint8_t lpwmChannel;
};

struct CurrentReading {
  int rRaw;
  int lRaw;
  int raw;
  float volts;
  float amps;
  bool saturated;
};

const MotorDriver leftMotor = {
    "Left",
    25, // RPWM
    26, // LPWM
    16, // R_EN
    17, // L_EN
    34, // R_IS
    35, // L_IS
    0,  // RPWM LEDC channel
    1   // LPWM LEDC channel
};

const MotorDriver rightMotor = {
    "Right",
    27, // RPWM
    14, // LPWM
    18, // R_EN
    19, // L_EN
    32, // R_IS
    33, // L_IS
    2,  // RPWM LEDC channel
    3   // LPWM LEDC channel
};

const char *FIRMWARE_VERSION = "KY024-RAMP-WEB-v2";
const char *AP_SSID = "KY024-Ramp-v2";
const char *AP_PASSWORD = "12345678"; // AP passwords must be empty or at least 8 chars.

const uint8_t KY024_DO_PIN = 21;
const bool MAGNET_ACTIVE_LOW = true;
const uint8_t TARGET_MAGNET_COUNT = 6;
const uint32_t MAGNET_DEBOUNCE_MS = 40;

const uint32_t SERIAL_BAUD = 115200;
const uint32_t PWM_FREQ_HZ = 20000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;

const int RAMP_START_SPEED = 45;
const int RAMP_MAX_SPEED = 180;
const int RAMP_STEP = 5;
const uint32_t RAMP_INTERVAL_MS = 200;

const float ADC_REF_VOLTS = 3.3f;
const float ADC_MAX_READING = 4095.0f;

// BTS7960 datasheet current sense ratio is typically around 8500.
// If your module has 1k from IS to GND: ampsPerVolt = 8500 / 1000 = 8.5.
const float CURRENT_AMPS_PER_VOLT = 8.5f;

const uint8_t CURRENT_SAMPLE_COUNT = 16;
const int ADC_SATURATION_RAW = 4080;

WebServer server(80);

int leftSpeed = 0;
int rightSpeed = 0;
int rampSpeed = 0;
uint8_t magnetCount = 0;
bool runActive = false;
bool targetReached = false;
bool magnetPresent = false;
bool lastRawMagnetState = false;
bool stableMagnetState = false;
uint32_t lastMagnetChangeMs = 0;
uint32_t lastRampMs = 0;

void setMotorSpeed(const MotorDriver &motor, int speed);

const char indexHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="tr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>KY-024 Ramp Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #080a10;
      --panel: #111827;
      --panel2: #172033;
      --text: #eef4ff;
      --muted: #91a0b8;
      --accent: #00e0a4;
      --warn: #ffcc00;
      --danger: #ff4d5e;
      --line: #273348;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: radial-gradient(circle at top left, #17335c 0, var(--bg) 42%);
      color: var(--text);
      padding: 18px;
    }
    main { max-width: 1020px; margin: 0 auto; }
    header {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      gap: 14px;
      margin-bottom: 18px;
    }
    h1 { margin: 0; font-size: clamp(28px, 6vw, 48px); letter-spacing: -0.05em; }
    .pill {
      border: 1px solid var(--line);
      background: rgba(17, 24, 39, 0.84);
      padding: 10px 12px;
      border-radius: 999px;
      color: var(--muted);
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }
    .card {
      background: linear-gradient(180deg, var(--panel), var(--panel2));
      border: 1px solid var(--line);
      border-radius: 22px;
      padding: 18px;
      box-shadow: 0 18px 50px rgba(0, 0, 0, 0.35);
    }
    .card h2 { margin: 0 0 14px; font-size: 22px; }
    .big {
      font-size: clamp(42px, 10vw, 72px);
      font-weight: 850;
      line-height: 1;
      letter-spacing: -0.06em;
      color: var(--accent);
    }
    .unit { color: var(--muted); font-size: 18px; }
    .rows { margin-top: 16px; display: grid; gap: 8px; }
    .row {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      border-top: 1px solid var(--line);
      padding-top: 8px;
      color: var(--muted);
    }
    .row b { color: var(--text); font-weight: 650; }
    .warn { color: var(--warn); font-weight: 800; }
    .danger { color: var(--danger); font-weight: 900; }
    .controls { margin-top: 14px; display: grid; gap: 14px; }
    .buttons { display: flex; gap: 10px; flex-wrap: wrap; }
    button {
      border: 0;
      border-radius: 14px;
      padding: 12px 16px;
      background: var(--accent);
      color: #03110d;
      font-weight: 850;
      cursor: pointer;
    }
    button.stop { background: var(--danger); color: white; }
    button.secondary { background: #2b3850; color: var(--text); }
    .note { color: var(--muted); line-height: 1.5; margin-top: 16px; }
    @media (max-width: 720px) {
      header { display: block; }
      .pill { display: inline-block; margin-top: 12px; }
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>KY-024 Ramp Control</h1>
        <div class="note">Firmware: <b>KY024-RAMP-WEB-v2</b>. Magnet count + BTS7960 current monitor.</div>
      </div>
      <div id="status" class="pill">connecting...</div>
    </header>

    <section class="grid">
      <article class="card">
        <h2>Run State</h2>
        <div><span id="magnetCount" class="big">0</span> <span class="unit">/ 6 magnets</span></div>
        <div id="runStatus" class="warn">IDLE</div>
        <div class="rows">
          <div class="row"><span>KY-024 DO</span><b id="sensorRaw">0</b></div>
          <div class="row"><span>Magnet present</span><b id="magnetPresent">false</b></div>
          <div class="row"><span>Ramp speed</span><b id="rampSpeed">0</b></div>
          <div class="row"><span>Target reached</span><b id="targetReached">false</b></div>
        </div>
      </article>

      <article class="card">
        <h2>Controls</h2>
        <div class="buttons">
          <button onclick="startRun()">START RAMP</button>
          <button class="stop" onclick="stopRun()">STOP</button>
          <button class="secondary" onclick="resetCount()">RESET COUNT</button>
        </div>
        <div class="note">START resets count, ramps both motors up gradually, and stops automatically when magnet count reaches 6.</div>
      </article>
    </section>

    <section class="grid" style="margin-top:14px">
      <article class="card">
        <h2>Sol Motor</h2>
        <div><span id="leftAmps" class="big">0.00</span> <span class="unit">A</span></div>
        <div id="leftWarn" class="warn"></div>
        <div class="rows">
          <div class="row"><span>R_IS raw</span><b id="leftRRaw">0</b></div>
          <div class="row"><span>L_IS raw</span><b id="leftLRaw">0</b></div>
          <div class="row"><span>Selected raw</span><b id="leftRaw">0</b></div>
          <div class="row"><span>IS voltage</span><b><span id="leftVolts">0.00</span> V</b></div>
          <div class="row"><span>Speed</span><b id="leftSpeedText">0</b></div>
        </div>
      </article>

      <article class="card">
        <h2>Sag Motor</h2>
        <div><span id="rightAmps" class="big">0.00</span> <span class="unit">A</span></div>
        <div id="rightWarn" class="warn"></div>
        <div class="rows">
          <div class="row"><span>R_IS raw</span><b id="rightRRaw">0</b></div>
          <div class="row"><span>L_IS raw</span><b id="rightLRaw">0</b></div>
          <div class="row"><span>Selected raw</span><b id="rightRaw">0</b></div>
          <div class="row"><span>IS voltage</span><b><span id="rightVolts">0.00</span> V</b></div>
          <div class="row"><span>Speed</span><b id="rightSpeedText">0</b></div>
        </div>
      </article>
    </section>

    <div class="note">Connect to WiFi KY024-Ramp-v2, password 12345678, open 192.168.4.1. If current shows ADC SATURATED, that IS pin is at the ESP32 ADC limit.</div>
  </main>

  <script>
    const $ = (id) => document.getElementById(id);

    function setMotorFields(prefix, data) {
      $(prefix + 'Amps').textContent = data.amps.toFixed(2);
      $(prefix + 'RRaw').textContent = data.rRaw;
      $(prefix + 'LRaw').textContent = data.lRaw;
      $(prefix + 'Raw').textContent = data.raw;
      $(prefix + 'Volts').textContent = data.volts.toFixed(3);
      $(prefix + 'SpeedText').textContent = data.speed;
      $(prefix + 'Warn').textContent = data.saturated ? 'ADC SATURATED' : '';
    }

    function setRunFields(data) {
      $('magnetCount').textContent = data.magnet.count;
      $('sensorRaw').textContent = data.magnet.rawLevel;
      $('magnetPresent').textContent = data.magnet.present;
      $('rampSpeed').textContent = data.run.rampSpeed;
      $('targetReached').textContent = data.run.targetReached;
      $('runStatus').textContent = data.run.active ? 'RUNNING' : (data.run.targetReached ? 'DONE' : 'IDLE');
      $('runStatus').className = data.run.targetReached ? 'danger' : 'warn';
    }

    async function refreshData() {
      try {
        const response = await fetch('/data', { cache: 'no-store' });
        const data = await response.json();
        setRunFields(data);
        setMotorFields('left', data.left);
        setMotorFields('right', data.right);
        $('status').textContent = 'live';
      } catch (err) {
        $('status').textContent = 'no connection';
      }
    }

    async function startRun() {
      await fetch('/start', { cache: 'no-store' });
      refreshData();
    }

    async function stopRun() {
      await fetch('/stop', { cache: 'no-store' });
      refreshData();
    }

    async function resetCount() {
      await fetch('/reset', { cache: 'no-store' });
      refreshData();
    }

    setInterval(refreshData, 250);
    refreshData();
  </script>
</body>
</html>
)rawliteral";

bool readMagnetActiveRaw() {
  const bool rawHigh = digitalRead(KY024_DO_PIN) == HIGH;
  return MAGNET_ACTIVE_LOW ? !rawHigh : rawHigh;
}

void stopMotors() {
  leftSpeed = 0;
  rightSpeed = 0;
  rampSpeed = 0;
  setMotorSpeed(leftMotor, 0);
  setMotorSpeed(rightMotor, 0);
}

void setupMotor(const MotorDriver &motor) {
  pinMode(motor.rEnablePin, OUTPUT);
  pinMode(motor.lEnablePin, OUTPUT);
  pinMode(motor.rCurrentPin, INPUT);
  pinMode(motor.lCurrentPin, INPUT);

  digitalWrite(motor.rEnablePin, HIGH);
  digitalWrite(motor.lEnablePin, HIGH);

  ledcSetup(motor.rpwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(motor.lpwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(motor.rpwmPin, motor.rpwmChannel);
  ledcAttachPin(motor.lpwmPin, motor.lpwmChannel);

  ledcWrite(motor.rpwmChannel, 0);
  ledcWrite(motor.lpwmChannel, 0);
}

void setMotorSpeed(const MotorDriver &motor, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);

  if (speed > 0) {
    ledcWrite(motor.rpwmChannel, speed);
    ledcWrite(motor.lpwmChannel, 0);
    return;
  }

  if (speed < 0) {
    ledcWrite(motor.rpwmChannel, 0);
    ledcWrite(motor.lpwmChannel, -speed);
    return;
  }

  ledcWrite(motor.rpwmChannel, 0);
  ledcWrite(motor.lpwmChannel, 0);
}

void applyRampSpeed() {
  leftSpeed = rampSpeed;
  rightSpeed = rampSpeed;
  setMotorSpeed(leftMotor, leftSpeed);
  setMotorSpeed(rightMotor, rightSpeed);
}

void startRun() {
  magnetCount = 0;
  targetReached = false;
  runActive = true;
  rampSpeed = RAMP_START_SPEED;
  lastRampMs = millis();
  applyRampSpeed();
}

void updateRampControl() {
  if (!runActive) {
    return;
  }

  if (magnetCount >= TARGET_MAGNET_COUNT) {
    runActive = false;
    targetReached = true;
    stopMotors();
    return;
  }

  const uint32_t now = millis();
  if (now - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    rampSpeed = min(rampSpeed + RAMP_STEP, RAMP_MAX_SPEED);
    applyRampSpeed();
  }
}

void updateMagnetCounter() {
  const bool rawState = readMagnetActiveRaw();
  const uint32_t now = millis();

  if (rawState != lastRawMagnetState) {
    lastRawMagnetState = rawState;
    lastMagnetChangeMs = now;
  }

  if (now - lastMagnetChangeMs < MAGNET_DEBOUNCE_MS) {
    magnetPresent = stableMagnetState;
    return;
  }

  if (rawState != stableMagnetState) {
    stableMagnetState = rawState;
    magnetPresent = stableMagnetState;

    if (stableMagnetState && magnetCount < TARGET_MAGNET_COUNT) {
      magnetCount++;
    }
  }

  magnetPresent = stableMagnetState;
}

int readAveragedRaw(uint8_t pin) {
  uint32_t total = 0;
  for (uint8_t i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    total += analogRead(pin);
    delayMicroseconds(250);
  }

  return total / CURRENT_SAMPLE_COUNT;
}

CurrentReading readCurrent(const MotorDriver &motor) {
  CurrentReading reading;

  reading.rRaw = readAveragedRaw(motor.rCurrentPin);
  reading.lRaw = readAveragedRaw(motor.lCurrentPin);
  reading.raw = max(reading.rRaw, reading.lRaw);
  reading.volts = (reading.raw * ADC_REF_VOLTS) / ADC_MAX_READING;
  reading.amps = reading.volts * CURRENT_AMPS_PER_VOLT;
  reading.saturated = reading.rRaw >= ADC_SATURATION_RAW || reading.lRaw >= ADC_SATURATION_RAW;

  return reading;
}

String readingJson(const CurrentReading &reading, int speed) {
  String json = "{";
  json += "\"rRaw\":" + String(reading.rRaw) + ",";
  json += "\"lRaw\":" + String(reading.lRaw) + ",";
  json += "\"raw\":" + String(reading.raw) + ",";
  json += "\"volts\":" + String(reading.volts, 4) + ",";
  json += "\"amps\":" + String(reading.amps, 4) + ",";
  json += "\"saturated\":" + String(reading.saturated ? "true" : "false") + ",";
  json += "\"speed\":" + String(speed);
  json += "}";
  return json;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send_P(200, "text/html", indexHtml);
}

void handleData() {
  const CurrentReading left = readCurrent(leftMotor);
  const CurrentReading right = readCurrent(rightMotor);

  String json = "{";
  json += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"left\":" + readingJson(left, leftSpeed) + ",";
  json += "\"right\":" + readingJson(right, rightSpeed) + ",";
  json += "\"magnet\":{";
  json += "\"count\":" + String(magnetCount) + ",";
  json += "\"target\":" + String(TARGET_MAGNET_COUNT) + ",";
  json += "\"present\":" + String(magnetPresent ? "true" : "false") + ",";
  json += "\"rawLevel\":" + String(digitalRead(KY024_DO_PIN));
  json += "},";
  json += "\"run\":{";
  json += "\"active\":" + String(runActive ? "true" : "false") + ",";
  json += "\"targetReached\":" + String(targetReached ? "true" : "false") + ",";
  json += "\"rampSpeed\":" + String(rampSpeed) + ",";
  json += "\"maxSpeed\":" + String(RAMP_MAX_SPEED);
  json += "}";
  json += "}";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", json);
}

void handleVersion() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "text/plain", FIRMWARE_VERSION);
}

void handleStart() {
  startRun();
  server.send(200, "text/plain", "STARTED");
}

void handleStop() {
  runActive = false;
  targetReached = false;
  stopMotors();
  server.send(200, "text/plain", "STOPPED");
}

void handleReset() {
  magnetCount = 0;
  targetReached = false;
  server.send(200, "text/plain", "RESET");
}

void setupWifiAP() {
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.softAPdisconnect(true);
  delay(300);
  WiFi.mode(WIFI_AP);
  delay(300);

  const bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println("ESP32 AP setup");
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("AP status: ");
  Serial.println(apOk ? "OK" : "FAILED");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  analogReadResolution(12);
  pinMode(KY024_DO_PIN, INPUT_PULLUP);
  lastRawMagnetState = readMagnetActiveRaw();
  stableMagnetState = lastRawMagnetState;
  magnetPresent = stableMagnetState;

  setupMotor(leftMotor);
  setupMotor(rightMotor);
  stopMotors();
  setupWifiAP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/version", HTTP_GET, handleVersion);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/reset", HTTP_GET, handleReset);
  server.begin();

  Serial.println("ESP32 motor web interface ready");
}

void loop() {
  server.handleClient();
  updateMagnetCounter();
  updateRampControl();
}