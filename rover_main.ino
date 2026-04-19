#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include "imu.h"
#include "motor.h"
#include "rover.h"

#define SERVO_PIN_FL 4
#define SERVO_PIN_FR 5
#define SERVO_PIN_BL 6
#define SERVO_PIN_BR 7

/*
20 and 21 reserved for IMU
*/

#define MOTOR_FL_IN1 9
#define MOTOR_FL_IN2 10
#define MOTOR_FL_PWM 11
MotorPinSet pinset_fl = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM};

#define MOTOR_FR_IN1 17
#define MOTOR_FR_IN2 18
#define MOTOR_FR_PWM 12
MotorPinSet pinset_fr = {MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM};

#define MOTOR_BL_IN1 9
#define MOTOR_BL_IN2 10
#define MOTOR_BL_PWM 13
MotorPinSet pinset_bl = {MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM};

#define MOTOR_BR_IN1 17
#define MOTOR_BR_IN2 18
#define MOTOR_BR_PWM 14
MotorPinSet pinset_br = {MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM};

ServoPins servopins = {SERVO_PIN_FL, SERVO_PIN_FR, SERVO_PIN_BL, SERVO_PIN_BR};
MotorPins motorpins = {pinset_fl, pinset_fr, pinset_bl, pinset_br};
MotorSpeeds motor_speeds = {0, 0, 0, 0, 0, 0, 0, 0};
ServoPositions servopos = {102.5, 95, 110, 95};
Rover rover(servopins, servopos, motorpins);

WebServer server(80);

const char* AP_SSID = "Rover-ESP32";
const char* AP_PASS = "rover1234";
IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);


int driveSpeed = 120;
int leftCmd = 0;
int rightCmd = 0;
String driveMode = "stop";
unsigned long lastDriveCommandMs = 0;
const unsigned long DRIVE_COMMAND_TIMEOUT_MS = 400;

unsigned long controlPeriodMs = 50;
unsigned long lastControlMs = 0;
unsigned long bootMs = 0;
unsigned long loopCounter = 0;

struct Telemetry {
  float pitch = 0.0f;
  float roll = 0.0f;
  float pitchMin = 9999.0f;
  float pitchMax = -9999.0f;
  float rollMin = 9999.0f;
  float rollMax = -9999.0f;
  float pitchAbsMax = 0.0f;
  float rollAbsMax = 0.0f;
  double pitchSum = 0.0;
  double rollSum = 0.0;
  unsigned long samples = 0;
  float avgPitch = 0.0f;
  float avgRoll = 0.0f;
} telemetry;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Rover Dashboard</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #111827;
      --card2: #1f2937;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --accent: #38bdf8;
      --ok: #22c55e;
      --warn: #f59e0b;
      --danger: #ef4444;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: linear-gradient(180deg, #020617, #0f172a 35%, #111827);
      color: var(--text);
    }
    .wrap {
      max-width: 1100px;
      margin: 0 auto;
      padding: 16px;
    }
    h1, h2, h3 { margin: 0 0 12px; }
    .sub { color: var(--muted); margin-bottom: 18px; }
    .grid {
      display: grid;
      gap: 16px;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    }
    .card {
      background: rgba(17, 24, 39, 0.94);
      border: 1px solid rgba(148, 163, 184, 0.18);
      border-radius: 16px;
      padding: 16px;
      box-shadow: 0 12px 28px rgba(0,0,0,0.25);
    }
    .row { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; }
    .row > * { flex: 1; min-width: 120px; }
    label { display: block; font-size: 13px; color: var(--muted); margin-bottom: 6px; }
    input[type="number"], input[type="range"] {
      width: 100%;
    }
    input[type="number"] {
      background: #0b1220;
      color: var(--text);
      border: 1px solid #334155;
      border-radius: 10px;
      padding: 10px;
    }
    button {
      background: #0ea5e9;
      color: white;
      border: none;
      border-radius: 12px;
      padding: 12px 14px;
      font-weight: 700;
      cursor: pointer;
    }
    button.secondary { background: #334155; }
    button.danger { background: #dc2626; }
    button:active { transform: scale(0.98); }
    .drive {
      display: grid;
      grid-template-columns: 90px 90px 90px;
      gap: 10px;
      justify-content: center;
      margin-top: 12px;
    }
    .drive .spacer { visibility: hidden; }
    .pill {
      display: inline-block;
      padding: 6px 10px;
      border-radius: 999px;
      background: #082f49;
      color: #bae6fd;
      font-size: 12px;
      font-weight: 700;
    }
    .stats {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 10px;
    }
    .stat {
      background: rgba(31, 41, 55, 0.9);
      border-radius: 12px;
      padding: 10px;
    }
    .stat .k { color: var(--muted); font-size: 12px; }
    .stat .v { font-size: 22px; font-weight: 700; margin-top: 6px; }
    canvas {
      width: 100%;
      height: 260px;
      background: #020617;
      border-radius: 14px;
      border: 1px solid #334155;
    }
    .footer {
      color: var(--muted);
      font-size: 12px;
      margin-top: 14px;
    }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>ESP32 Rover Dashboard</h1>
    <div class="sub">Connect to the rover AP, open <b>192.168.4.1</b>, tune PID, drive, and watch live telemetry.</div>

    <div class="grid">
      <div class="card">
        <h2>Drive</h2>
        <div class="row">
          <div>
            <label for="speed">Drive speed</label>
            <input id="speed" type="range" min="0" max="255" value="120">
            <div><span id="speedVal">120</span></div>
          </div>
          <div>
            <label>Mode</label>
            <div><span id="mode" class="pill">stop</span></div>
          </div>
        </div>
        <div class="drive">
          <div class="spacer">.</div>
          <button data-cmd="forward">Forward</button>
          <div class="spacer">.</div>
          <button data-cmd="left">Left</button>
          <button class="danger" id="stopBtn">Stop</button>
          <button data-cmd="right">Right</button>
          <div class="spacer">.</div>
          <button data-cmd="backward">Backward</button>
          <div class="spacer">.</div>
        </div>
        <div class="footer">Press and hold a drive button. Release to stop.</div>
      </div>

      <div class="card">
        <h2>Sensitivity</h2>
        <div class="row">
          <div>
            <label>Sensitivity</label>
            <input id="sensitivity" type="number" step="0.01" value="1.0">
          </div>
        </div>
        <div class="row" style="margin-top:12px;">
          <button id="applySensitivity">Apply Sensitivity</button>
          <button id="resetStats" class="secondary">Reset Stats</button>
        </div>
        <div class="footer"><span id="sensitivityStatus" class="ok">Ready.</span></div>
      </div>
    </div>
    </div>

    <div class="card" style="margin-top:16px;">
      <h2>Live Telemetry</h2>
      <div class="stats">
        <div class="stat"><div class="k">Pitch</div><div class="v" id="pitchNow">0.00</div></div>
        <div class="stat"><div class="k">Roll</div><div class="v" id="rollNow">0.00</div></div>
        <div class="stat"><div class="k">Pitch avg</div><div class="v" id="pitchAvg">0.00</div></div>
        <div class="stat"><div class="k">Roll avg</div><div class="v" id="rollAvg">0.00</div></div>
        <div class="stat"><div class="k">Pitch abs max</div><div class="v" id="pitchAbsMax">0.00</div></div>
        <div class="stat"><div class="k">Roll abs max</div><div class="v" id="rollAbsMax">0.00</div></div>
        <div class="stat"><div class="k">Loop count</div><div class="v" id="loops">0</div></div>
        <div class="stat"><div class="k">Uptime (s)</div><div class="v" id="uptime">0</div></div>
      </div>
      <div style="margin-top:16px;">
        <canvas id="chart" width="1000" height="260"></canvas>
      </div>
      <div class="footer">Blue = pitch, green = roll, zero line centered.</div>
    </div>
  </div>

  <script>
    const speed = document.getElementById('speed');
    const speedVal = document.getElementById('speedVal');
    const mode = document.getElementById('mode');
    const sensitivityStatus = document.getElementById('sensitivityStatus');
    const chart = document.getElementById('chart');
    const ctx = chart.getContext('2d');
    const pitchHistory = [];
    const rollHistory = [];
    const MAX_POINTS = 180;
    let activeCmd = 'stop';
    let driveRepeatTimer = null;


    speed.addEventListener('input', () => {
      speedVal.textContent = speed.value;
    });

    function api(url, opts = {}) {
      return fetch(url, opts).then(r => r.json());
    }

    function sendDrive(cmd) {
  activeCmd = cmd;
  api(`/api/drive?cmd=${encodeURIComponent(cmd)}&speed=${encodeURIComponent(speed.value)}`)
    .catch(() => {});
}

function startDriveHold(cmd) {
  if (activeCmd === cmd && driveRepeatTimer !== null) {
    return;
  }

  activeCmd = cmd;
  sendDrive(cmd);

  if (driveRepeatTimer !== null) {
    clearInterval(driveRepeatTimer);
  }

  driveRepeatTimer = setInterval(() => {
    if (activeCmd !== 'stop') {
      sendDrive(activeCmd);
    }
  }, 100);
}

function stopDrive() {
  if (driveRepeatTimer !== null) {
    clearInterval(driveRepeatTimer);
    driveRepeatTimer = null;
  }

  activeCmd = 'stop';
  api('/api/stop').catch(() => {});
}

    function bindHold(btn) {
  const cmd = btn.dataset.cmd;

  const start = (e) => {
    e.preventDefault();
    startDriveHold(cmd);
  };

  const end = (e) => {
    e.preventDefault();
    stopDrive();
  };

  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointerleave', end);
  btn.addEventListener('pointercancel', end);
}

    document.querySelectorAll('[data-cmd]').forEach(bindHold);
    document.getElementById('stopBtn').addEventListener('click', stopDrive);
    window.addEventListener('pointerup', stopDrive);
    window.addEventListener('blur', stopDrive);

    document.getElementById('applySensitivity').addEventListener('click', async () => {
      const sensitivity = document.getElementById('sensitivity').value;
      try {
        const data = await api(`/api/sensitivity?value=${encodeURIComponent(sensitivity)}`);
        sensitivityStatus.textContent = `Applied sensitivity: ${data.sensitivity.toFixed(2)}`;
      } catch {
        sensitivityStatus.textContent = 'Failed to apply sensitivity';
      }
    });

    document.getElementById('resetStats').addEventListener('click', async () => {
      await api('/api/resetStats');
    });

    function drawChart() {
      const w = chart.width;
      const h = chart.height;
      ctx.clearRect(0, 0, w, h);

      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = (h / 4) * i;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }

      ctx.strokeStyle = '#64748b';
      ctx.beginPath();
      ctx.moveTo(0, h / 2);
      ctx.lineTo(w, h / 2);
      ctx.stroke();

      const maxAbs = Math.max(5, ...pitchHistory.map(v => Math.abs(v)), ...rollHistory.map(v => Math.abs(v)));
      const sx = w / Math.max(1, MAX_POINTS - 1);
      const scaleY = (h * 0.42) / maxAbs;

      function drawSeries(arr, color) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        arr.forEach((v, i) => {
          const x = i * sx;
          const y = h / 2 - v * scaleY;
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
      }

      drawSeries(pitchHistory, '#38bdf8');
      drawSeries(rollHistory, '#22c55e');
    }

    async function refresh() {
      try {
        const t = await api('/api/telemetry');
        document.getElementById('pitchNow').textContent = t.pitch.toFixed(2);
        document.getElementById('rollNow').textContent = t.roll.toFixed(2);
        document.getElementById('pitchAvg').textContent = t.pitchAvg.toFixed(2);
        document.getElementById('rollAvg').textContent = t.rollAvg.toFixed(2);
        document.getElementById('pitchAbsMax').textContent = t.pitchAbsMax.toFixed(2);
        document.getElementById('rollAbsMax').textContent = t.rollAbsMax.toFixed(2);
        document.getElementById('loops').textContent = t.samples;
        document.getElementById('uptime').textContent = t.uptimeSec;
        document.getElementById('sensitivity').value = document.activeElement === document.getElementById('sensitivity') ? document.getElementById('sensitivity').value : t.sensitivity;
        mode.textContent = t.driveMode;

        pitchHistory.push(t.pitch);
        rollHistory.push(t.roll);
        while (pitchHistory.length > MAX_POINTS) pitchHistory.shift();
        while (rollHistory.length > MAX_POINTS) rollHistory.shift();
        drawChart();
      } catch (e) {
        sensitivityStatus.textContent = 'Telemetry link lost';
      }
    }

    setInterval(refresh, 200);
    refresh();
  </script>
</body>
</html>
)rawliteral";

void resetStats() {
  telemetry.pitchMin = 9999.0f;
  telemetry.pitchMax = -9999.0f;
  telemetry.rollMin = 9999.0f;
  telemetry.rollMax = -9999.0f;
  telemetry.pitchAbsMax = 0.0f;
  telemetry.rollAbsMax = 0.0f;
  telemetry.pitchSum = 0.0;
  telemetry.rollSum = 0.0;
  telemetry.samples = 0;
  telemetry.avgPitch = 0.0f;
  telemetry.avgRoll = 0.0f;
}

void setDriveCommand(int left, int right, const String& modeName) {
  leftCmd = constrain(left, -255, 255);
  rightCmd = constrain(right, -255, 255);
  driveMode = modeName;
  lastDriveCommandMs = millis();

  motor_speeds.fl_speed = abs(leftCmd);
  motor_speeds.bl_speed = abs(leftCmd);
  motor_speeds.fr_speed = abs(rightCmd);
  motor_speeds.br_speed = abs(rightCmd);

  motor_speeds.fl_dir = (leftCmd >= 0) ? 1 : 0;
  motor_speeds.bl_dir = (leftCmd >= 0) ? 1 : 0;
  motor_speeds.fr_dir = (rightCmd >= 0) ? 1 : 0;
  motor_speeds.br_dir = (rightCmd >= 0) ? 1 : 0;

  rover.set_motor_speed(motor_speeds);
}

void stopDrive() {
  setDriveCommand(0, 0, "stop");
}

void updateTelemetry(const Orientation& data) {
  telemetry.pitch = data.pitch;
  telemetry.roll = data.roll;

  telemetry.pitchMin = min(telemetry.pitchMin, telemetry.pitch);
  telemetry.pitchMax = max(telemetry.pitchMax, telemetry.pitch);
  telemetry.rollMin = min(telemetry.rollMin, telemetry.roll);
  telemetry.rollMax = max(telemetry.rollMax, telemetry.roll);
  telemetry.pitchAbsMax = max(telemetry.pitchAbsMax, fabsf(telemetry.pitch));
  telemetry.rollAbsMax = max(telemetry.rollAbsMax, fabsf(telemetry.roll));

  telemetry.pitchSum += telemetry.pitch;
  telemetry.rollSum += telemetry.roll;
  telemetry.samples++;
  telemetry.avgPitch = telemetry.pitchSum / telemetry.samples;
  telemetry.avgRoll = telemetry.rollSum / telemetry.samples;
}

String jsonFloat(float value, int places = 3) {
  return String(value, places);
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleTelemetry() {
  String json = "{";
  json += "\"pitch\":" + jsonFloat(telemetry.pitch, 3) + ",";
  json += "\"roll\":" + jsonFloat(telemetry.roll, 3) + ",";
  json += "\"pitchMin\":" + jsonFloat(telemetry.pitchMin, 3) + ",";
  json += "\"pitchMax\":" + jsonFloat(telemetry.pitchMax, 3) + ",";
  json += "\"rollMin\":" + jsonFloat(telemetry.rollMin, 3) + ",";
  json += "\"rollMax\":" + jsonFloat(telemetry.rollMax, 3) + ",";
  json += "\"pitchAbsMax\":" + jsonFloat(telemetry.pitchAbsMax, 3) + ",";
  json += "\"rollAbsMax\":" + jsonFloat(telemetry.rollAbsMax, 3) + ",";
  json += "\"pitchAvg\":" + jsonFloat(telemetry.avgPitch, 3) + ",";
  json += "\"rollAvg\":" + jsonFloat(telemetry.avgRoll, 3) + ",";
  json += "\"samples\":" + String(telemetry.samples) + ",";
  json += "\"uptimeSec\":" + String((millis() - bootMs) / 1000UL) + ",";
  json += "\"sensitivity\":" + jsonFloat(rover.get_sensitivity(), 3) + ",";
  json += "\"driveSpeed\":" + String(driveSpeed) + ",";
  json += "\"leftCmd\":" + String(leftCmd) + ",";
  json += "\"rightCmd\":" + String(rightCmd) + ",";
  json += "\"driveMode\":\"" + driveMode + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSensitivity() {
    // 1. Handle Input
    Serial.println("Handling sens");
    if (server.hasArg("value")) {
      Serial.println("Handling sen2s");
      rover.set_sensitivity(server.arg("value").toFloat());
    }

    // 2. Build JSON String
    // Fixed escaping: \" is required for quotes inside the string
    String json = "{";
    json += "\"ok\":true,";
    json += "\"sensitivity\":" + String(rover.get_sensitivity(), 3); 
    json += "}";

    // 3. Send Response
    server.send(200, "application/json", json);
}

void handleDrive() {
  String cmd = server.arg("cmd");
  if (server.hasArg("speed")) {
    driveSpeed = constrain(server.arg("speed").toInt(), 0, 255);
  }

  if (cmd == "forward") {
    setDriveCommand(driveSpeed, driveSpeed, "forward");
  } else if (cmd == "backward") {
    setDriveCommand(-driveSpeed, -driveSpeed, "backward");
  } else if (cmd == "left") {
    setDriveCommand(-driveSpeed, driveSpeed, "left");
  } else if (cmd == "right") {
    setDriveCommand(driveSpeed, -driveSpeed, "right");
  } else {
    stopDrive();
  }

  String json = "{";
  json += "\"ok\":true,";
  json += "\"driveMode\":\"" + driveMode + "\",";
  json += "\"leftCmd\":" + String(leftCmd) + ",";
  json += "\"rightCmd\":" + String(rightCmd) + ",";
  json += "\"speed\":" + String(driveSpeed);
  json += "}";
  server.send(200, "application/json", json);
}

void handleStop() {
  stopDrive();
  server.send(200, "application/json", "{\"ok\":true,\"driveMode\":\"stop\"}");
}

void handleResetStats() {
  resetStats();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/sensitivity", HTTP_GET, handleSensitivity);
  server.on("/api/drive", HTTP_GET, handleDrive);
  server.on("/api/stop", HTTP_GET, handleStop);
  server.on("/api/resetStats", HTTP_GET, handleResetStats);
  server.onNotFound(handleNotFound);
  server.begin();
}

void startSoftAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.println("=== Rover Web Dashboard ===");
  Serial.print("SoftAP status: ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(200);

  bootMs = millis();
  initIMU();
  resetStats();


  startSoftAP();
  setupWebServer();

  Serial.println("Open http://192.168.4.1");
}

void loop() {
  server.handleClient();

  const unsigned long now = millis();

  if (driveMode != "stop" && (now - lastDriveCommandMs) > DRIVE_COMMAND_TIMEOUT_MS) {
    stopDrive();
  }

  if ((now - lastControlMs) >= controlPeriodMs) {
    lastControlMs = now;
    Orientation data = getFilteredOrientation();
    updateTelemetry(data);
    rover.update();
    loopCounter++;

    if ((loopCounter % 10UL) == 0UL) {
      Serial.print("Pitch: ");
      Serial.print(telemetry.pitch, 2);
      Serial.print(" Roll: ");
      Serial.print(telemetry.roll, 2);
      Serial.print(" Drive: ");
      Serial.println(driveMode);
    }
  }
}