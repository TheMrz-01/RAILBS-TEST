#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

struct MotorDriver {
  const char *name;
  uint8_t rpwmPin;
  uint8_t lpwmPin;
  uint8_t rpwmChannel;
  uint8_t lpwmChannel;
};

// BTS7960 driver for left motor.
static constexpr uint8_t LEFT_RPWM_PIN = 25;
static constexpr uint8_t LEFT_LPWM_PIN = 26;

// BTS7960 driver for right motor.
static constexpr uint8_t RIGHT_RPWM_PIN = 27;
static constexpr uint8_t RIGHT_LPWM_PIN = 14;

// KY-024 digital outputs.
// Front sensor is the primary marker counter.
// Rear sensor is logged and can later be used for speed estimation.
static constexpr uint8_t FRONT_MAGNET_PIN = 34;
static constexpr uint8_t REAR_MAGNET_PIN = 35;

// HC-SR04 ultrasonic sensor.
static constexpr uint8_t ULTRASONIC_TRIG_PIN = 5;
static constexpr uint8_t ULTRASONIC_ECHO_PIN = 18;

const MotorDriver leftMotor = {
    "Left",
    LEFT_RPWM_PIN,
    LEFT_LPWM_PIN,
    0,  // RPWM LEDC channel
    1   // LPWM LEDC channel
};

const MotorDriver rightMotor = {
    "Right",
    RIGHT_RPWM_PIN,
    RIGHT_LPWM_PIN,
    2,  // RPWM LEDC channel
    3   // LPWM LEDC channel
};

const char *FIRMWARE_VERSION = "RAIL-RULE-SIM-v6";
const char *AP_SSID = "KY024-Ramp-v2";
const char *AP_PASSWORD = "12345678"; // AP passwords must be empty or at least 8 chars.

// --- Yarışma ölçüleri ---
// 20 m hat, 50 cm mıknatıs aralığı: başlangıç + bitiş dahil 41 mıknatıs / 40 aralık.
const uint16_t RACE_LENGTH_CM = 2000;
const uint16_t MAGNET_INTERVAL_CM = 50;
const uint8_t TARGET_MAGNET_COUNT = 40; // Başlangıç mıknatısı hariç geçilecek aralık sayısı.
const uint16_t TUNNEL_START_CM = 912;
const uint16_t TUNNEL_LENGTH_CM = 88;
const uint16_t TUNNEL_SENSOR_CM = 962; // Tünelin başından 50 cm içerisi.
const uint8_t TUNNEL_STOP_COUNT = 19;  // 19 * 50 = 950 cm; sensör noktasına en yakın mıknatıs tabanlı yaklaşım.
const uint32_t TUNNEL_CREEP_MS = 650;  // 950 cm'den 962 cm'ye yaklaşmak için düşük PWM sürünme süresi. Pistte kalibre et.
const uint32_t TUNNEL_WAIT_MS = 5000;
const uint16_t LAUNCH_END_CM = 200;
const uint8_t LAUNCH_END_COUNT = 4;
const uint16_t FINAL_BRAKE_START_CM = 1750;
const uint8_t FINAL_BRAKE_START_COUNT = 35;

const bool MAGNET_ACTIVE_LOW = true;
const uint32_t MAGNET_DEBOUNCE_MS = 40;

const float ULTRASONIC_PASS_THRESHOLD_CM = 5.0f;
const float ULTRASONIC_RELEASE_THRESHOLD_CM = 7.0f;
const uint32_t ULTRASONIC_READ_INTERVAL_MS = 100;
const uint32_t ULTRASONIC_TIMEOUT_US = 25000;

const uint32_t SERIAL_BAUD = 115200;
const uint32_t PWM_FREQ_HZ = 20000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;

// --- Sürüş profili ---
const int LAUNCH_MIN_SPEED = 34;     // İlk kalkışta yavaş başlasın.
const int CREEP_SPEED = 42;          // Tünel/sensör yaklaşımı.
const int CRUISE_SPEED = 175;        // Ana seyir.
const int FINAL_MIN_SPEED = 34;      // Bitişe yaklaşırken son minimum PWM.
const int RAMP_UP_STEP = 4;          // PWM yumuşatma: düşük değer = daha sakin kalkış.
const int RAMP_DOWN_STEP = 8;        // Fren/azaltma biraz daha hızlı olabilir.
const uint32_t RAMP_INTERVAL_MS = 100;

const int MANUAL_DEFAULT_SPEED = 90;
const int MANUAL_MAX_SPEED = 180;
const uint32_t MANUAL_COMMAND_TIMEOUT_MS = 1500;

WebServer server(80);

int leftSpeed = 0;
int rightSpeed = 0;
int rampSpeed = 0;
int targetSpeed = 0;
uint8_t magnetCount = 0;
uint16_t ultrasonicPassCount = 0;
bool runActive = false;
bool runPaused = false;
bool manualActive = false;
bool targetReached = false;
bool frontMagnetPresent = false;
bool rearMagnetPresent = false;
bool lastRawFrontMagnetState = false;
bool stableFrontMagnetState = false;
bool lastRawRearMagnetState = false;
bool stableRearMagnetState = false;
bool tunnelCreepActive = false;
bool tunnelWaiting = false;
bool tunnelHandled = false;
bool ultrasonicObjectNear = false;
bool ultrasonicReadingValid = false;
float ultrasonicDistanceCm = -1.0f;
uint32_t lastMagnetChangeMs = 0;
uint32_t lastRearMagnetChangeMs = 0;
uint32_t lastRampMs = 0;
uint32_t lastManualCommandMs = 0;
uint32_t lastUltrasonicReadMs = 0;
uint32_t missionStartMs = 0;
uint32_t tunnelCreepStartedMs = 0;
uint32_t tunnelWaitStartedMs = 0;
uint32_t finishMs = 0;

void setMotorSpeed(const MotorDriver &motor, int speed);

const char indexHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="tr">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Raylı Sistem Aracı - Kural Simülatörü</title>
  <style>
    :root{
      --bg:#07101c; --panel:rgba(15,26,46,.92); --panel2:rgba(255,255,255,.055); --line:rgba(255,255,255,.11);
      --text:#eef6ff; --muted:#91a8c7; --green:#63e08e; --yellow:#f4d36e; --red:#ff6b7d; --blue:#67a6ff;
      --shadow:0 20px 70px rgba(0,0,0,.36); --trackPad:34px;
    }
    *{box-sizing:border-box} body{margin:0;min-height:100vh;font-family:Inter,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;color:var(--text);background:radial-gradient(circle at 18% 8%,rgba(103,166,255,.20),transparent 30%),radial-gradient(circle at 85% 18%,rgba(99,224,142,.12),transparent 28%),linear-gradient(135deg,#08111f,#0d1728 55%,#07101c)}
    .wrap{width:min(1440px,calc(100% - 22px));margin:16px auto}.topbar{display:flex;justify-content:space-between;gap:14px;align-items:center;background:var(--panel);border:1px solid var(--line);border-radius:22px;padding:15px 17px;box-shadow:var(--shadow);backdrop-filter:blur(10px);margin-bottom:14px}.brand{display:flex;align-items:center;gap:12px}.logo{width:50px;height:50px;border-radius:16px;display:grid;place-items:center;background:linear-gradient(135deg,var(--blue),var(--green));font-size:24px}.brand small{display:block;color:var(--muted);letter-spacing:.14em;text-transform:uppercase;font-size:11px;font-weight:900}.brand h1{margin:2px 0 0;font-size:28px;letter-spacing:-.04em}.badges{display:flex;gap:9px;flex-wrap:wrap;justify-content:flex-end}.badge{padding:9px 11px;border-radius:999px;border:1px solid var(--line);background:rgba(255,255,255,.05);color:var(--muted);font-weight:900;font-size:12px;white-space:nowrap}.ok{color:#cffff0;border-color:rgba(99,224,142,.25);background:rgba(99,224,142,.10)}.warn{color:#fff0c3;border-color:rgba(244,211,110,.25);background:rgba(244,211,110,.10)}.danger{color:#ffd5db;border-color:rgba(255,107,125,.25);background:rgba(255,107,125,.10)}
    .grid{display:grid;grid-template-columns:1.45fr .95fr;gap:14px}.side{display:grid;gap:14px;align-content:start}.card{background:var(--panel);border:1px solid var(--line);border-radius:22px;box-shadow:var(--shadow);overflow:hidden}.head{padding:15px 17px 0;display:flex;justify-content:space-between;gap:10px;align-items:flex-start}.head h2{margin:0;font-size:18px;letter-spacing:-.03em}.head small{display:block;margin-top:4px;color:var(--muted);font-weight:800;line-height:1.35}.body{padding:17px}.meta{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-bottom:12px}.mini,.stat,.toggle{border-radius:16px;background:var(--panel2);border:1px solid rgba(255,255,255,.075)}.mini{padding:11px}.mini span,.stat span{display:block;color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.12em;font-weight:900;margin-bottom:6px}.mini strong{font-size:22px;letter-spacing:-.04em}.track-shell{border-radius:20px;border:1px solid rgba(255,255,255,.08);background:linear-gradient(180deg,rgba(255,255,255,.04),rgba(255,255,255,.025));padding:12px;overflow:hidden}.scale{position:relative;height:38px;border-radius:14px;overflow:hidden;background:rgba(255,255,255,.03);border:1px solid rgba(255,255,255,.06);margin-bottom:8px}.track{position:relative;height:235px;border-radius:24px;overflow:hidden;background:radial-gradient(circle at 50% 50%,rgba(103,166,255,.10),transparent 36%),linear-gradient(180deg,rgba(255,255,255,.04),rgba(255,255,255,.02));border:1px solid rgba(255,255,255,.08);margin-bottom:8px}.rail{position:absolute;left:var(--trackPad);right:var(--trackPad);height:12px;border-radius:999px;background:rgba(255,255,255,.55);z-index:2}.rail.top{top:78px}.rail.bottom{bottom:78px}.sleeper{position:absolute;top:61px;width:3px;height:112px;border-radius:999px;background:rgba(255,255,255,.16);z-index:1;transform:translateX(-50%)}.magnet{position:absolute;top:108px;width:10px;height:10px;border-radius:50%;transform:translateX(-50%);background:var(--yellow);box-shadow:0 0 0 3px rgba(244,211,110,.10),0 0 14px rgba(244,211,110,.22);z-index:3;transition:.18s}.magnet.read{background:var(--green);box-shadow:0 0 0 3px rgba(99,224,142,.12),0 0 15px rgba(99,224,142,.42);transform:translateX(-50%) scale(1.2)}.tunnel{position:absolute;top:48px;bottom:48px;border:3px solid rgba(244,211,110,.70);border-radius:26px;background:repeating-linear-gradient(135deg,rgba(244,211,110,.14),rgba(244,211,110,.14) 12px,rgba(244,211,110,.05) 12px,rgba(244,211,110,.05) 24px);z-index:2;display:grid;place-items:center;color:#ffeab0;font-weight:900;font-size:11px;letter-spacing:.13em;text-align:center;pointer-events:none}.beam{position:absolute;top:12px;bottom:12px;width:15px;border-radius:999px;transform:translateX(-50%);background:rgba(103,166,255,.18);border:2px solid rgba(103,166,255,.65);box-shadow:0 0 24px rgba(103,166,255,.25);z-index:2;pointer-events:none}.finish{position:absolute;top:35px;bottom:35px;width:12px;border-radius:999px;background:var(--green);box-shadow:0 0 22px rgba(99,224,142,.45);z-index:2}.car{position:absolute;top:75px;width:150px;height:78px;transform:translateX(-50%);border-radius:32px;background:linear-gradient(135deg,#f7fbff,#b9c8e5);border:2px solid rgba(255,255,255,.65);box-shadow:0 16px 36px rgba(0,0,0,.30);z-index:5;transition:left .12s ease}.car:before,.car:after{content:"";position:absolute;bottom:-16px;width:34px;height:34px;border-radius:50%;background:#091224;border:5px solid #dce6ff}.car:before{left:32px}.car:after{right:20px}.window{position:absolute;top:18px;left:26px;right:30px;height:24px;border-radius:999px;background:linear-gradient(90deg,#79aef5,#90e38f);opacity:.95}.dot{position:absolute;bottom:-6px;width:14px;height:14px;border-radius:50%;border:2px solid #fff;background:#21344f}.dot.front{left:18px}.dot.rear{left:44px}.dot.on{background:var(--green);box-shadow:0 0 14px rgba(99,224,142,.55)}.dot.off{background:var(--red);box-shadow:0 0 14px rgba(255,107,125,.40)}.label{position:absolute;top:-22px;font-size:10px;font-weight:900;color:#d7e6ff;background:rgba(8,17,31,.75);padding:3px 6px;border-radius:999px;border:1px solid rgba(255,255,255,.08)}.label.front{left:4px}.label.rear{left:30px}.controls{display:flex;flex-wrap:wrap;gap:9px;margin-top:13px}button{border:0;border-radius:15px;padding:12px 13px;font-weight:900;font-size:14px;cursor:pointer;transition:.16s;user-select:none}button:hover{transform:translateY(-1px);filter:brightness(1.04)}button:active{transform:scale(.98)}.btn-green{background:linear-gradient(135deg,var(--green),#d5ffe3);color:#08111f}.btn-yellow{background:linear-gradient(135deg,var(--yellow),#fff0c0);color:#08111f}.btn-red{background:linear-gradient(135deg,var(--red),#ffd2d9);color:#08111f}.btn-blue{background:linear-gradient(135deg,var(--blue),#d8e7ff);color:#08111f}.btn-dark{background:rgba(255,255,255,.06);color:var(--text);border:1px solid rgba(255,255,255,.08)}.manual{margin-top:13px;display:grid;grid-template-columns:1fr 250px;gap:13px}.pad{border-radius:18px;padding:12px;background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.07)}.pad-title{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;gap:8px}.pill{display:inline-block;padding:7px 9px;border-radius:999px;font-size:11px;font-weight:900;letter-spacing:.1em;text-transform:uppercase;border:1px solid rgba(255,255,255,.08);background:rgba(255,255,255,.05);color:var(--muted);white-space:nowrap}.pad-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;align-items:center;justify-items:center}.empty{visibility:hidden}.arrow{width:58px;height:58px;border-radius:16px;font-size:22px;display:grid;place-items:center}.hint{color:var(--muted);font-size:12px;line-height:1.5;margin-top:10px}.log{height:174px;overflow:auto;display:grid;gap:7px;padding-right:4px}.log-item{padding:9px 10px;background:rgba(255,255,255,.05);border:1px solid rgba(255,255,255,.07);border-radius:13px;color:#d8e6fb;font-size:12px;line-height:1.35;font-weight:700}.stats{display:grid;grid-template-columns:repeat(2,1fr);gap:11px}.stat{padding:14px}.stat strong{display:block;font-size:25px;letter-spacing:-.04em}.bar{height:10px;background:rgba(255,255,255,.08);border-radius:999px;overflow:hidden;margin-top:10px}.bar>div{height:100%;width:0%;border-radius:inherit;background:linear-gradient(90deg,var(--red),var(--yellow),var(--green))}.switches{display:grid;gap:9px}.toggle{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:12px 13px;font-weight:800;color:#d9e7ff}.toggle span{color:var(--muted);text-align:right}.toggle.block{display:block}.chart-card canvas{width:100%;height:210px;border-radius:16px;background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.07)}input[type="range"]{width:100%;accent-color:var(--blue)}
    @media(max-width:1100px){.grid{grid-template-columns:1fr}.manual{grid-template-columns:1fr}.meta{grid-template-columns:repeat(2,1fr)}}@media(max-width:720px){.topbar{align-items:flex-start;flex-direction:column}.stats{grid-template-columns:1fr}.track{height:190px}.rail.top{top:58px}.rail.bottom{bottom:58px}.sleeper{top:45px;height:98px}.magnet{top:87px;width:8px;height:8px}.car{width:122px;height:66px;top:62px}.car:before,.car:after{width:28px;height:28px;bottom:-12px}.tunnel{top:34px;bottom:34px}.finish{top:28px;bottom:28px}}
    .meta{grid-template-columns:repeat(6,1fr)}
    @media(max-width:1100px){.meta{grid-template-columns:repeat(2,1fr)}}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <div class="brand"><div class="logo">🚇</div><div><small>Raylı Sistem Aracı</small><h1>Kural Simülatörü + Canlı Kontrol</h1></div></div>
      <div class="badges"><div class="badge" id="modeBadge">MOD: AUTO</div><div class="badge warn" id="stateBadge">Durum: Bağlanıyor</div><div class="badge" id="scoreBadge">Skor: --</div><div class="badge" id="timeBadge">Süre: 0.0 sn</div><div class="badge" id="wifiBadge">WiFi: --</div></div>
    </div>
    <div class="grid">
      <div class="card">
        <div class="head"><div><h2>20 Metre Yarışma Hattı</h2><small>41 mıknatıs • 50 cm aralık • tünel 9.12-10.00 m • sensör 9.62 m • 5 sn bekleme</small></div><span class="pill" id="pauseInfo">Veri bekleniyor</span></div>
        <div class="body">
          <div class="track-shell">
            <div class="meta">
              <div class="mini"><span>PWM</span><strong id="speedText">0</strong></div>
              <div class="mini"><span>Konum</span><strong id="posText">0.00</strong> m</div>
              <div class="mini"><span>Tünel Bekleme</span><strong id="waitText">0.0</strong> sn</div>
              <div class="mini"><span>Mıknatıs</span><strong><span id="frontCount">0</span> / <span id="rearCount">40</span></strong></div>
              <div class="mini"><span>Ortalama PWM</span><strong id="trustText">0</strong></div>
              <div class="mini"><span>Ultrasonik</span><strong id="ultraCountText">0</strong> geçiş</div>
            </div>
            <div class="scale" id="topScale"></div>
            <div class="track" id="track">
              <div class="rail top"></div><div class="rail bottom"></div><div class="tunnel" id="tunnel">TÜNEL<br>88 cm</div><div class="beam" id="tunnelBeam"></div><div class="finish" id="finishLine"></div>
              <div class="car" id="car"><div class="window"></div><div class="label front">Hall</div><div class="label rear">PWM</div><div class="dot front" id="frontLed"></div><div class="dot rear" id="rearLed"></div></div>
            </div>
            <div class="scale" id="bottomScale"></div>
          </div>
          <div class="controls"><button class="btn-green" id="startBtn">▶ Başlat</button><button class="btn-yellow" id="pauseBtn">⏸ Duraklat</button><button class="btn-blue" id="resumeBtn">⏵ Devam Et</button><button class="btn-red" id="stopBtn">⛔ Acil Dur</button><button class="btn-dark" id="resetBtn">↺ Sıfırla</button></div>
          <div class="manual">
            <div><div class="pad"><div class="pad-title"><strong>Manuel Test</strong><span class="pill">Yarışmada sadece test için</span></div><div class="pad-grid"><div class="empty">.</div><button class="btn-dark arrow" id="manualStop">■</button><div class="empty">.</div><button class="btn-dark arrow" id="manualBack">←</button><button class="btn-dark arrow" id="manualCenter">●</button><button class="btn-dark arrow" id="manualForward">→</button><div class="empty">.</div><button class="btn-dark arrow" id="manualBrake">⏸</button><div class="empty">.</div></div><div class="toggle block" style="margin-top:10px"><div style="display:flex;justify-content:space-between;gap:10px"><span>Manuel PWM</span><b><span id="manualSpeedText">90</span></b></div><input type="range" min="25" max="180" value="90" id="manualSpeedRange"></div></div><div class="hint"><b>Not:</b> Otonomda kalkış yumuşak başlar, tünel noktasında 5 sn bekler, son 2.5 metrede parabolik fren profiline geçer.</div></div>
            <div class="pad"><div class="pad-title"><strong>Olay Kaydı</strong></div><div class="log" id="log"></div></div>
          </div>
        </div>
      </div>
      <div class="side">
        <div class="card"><div class="head"><h2>Canlı Durum</h2><small>ESP32 / yarışma verisi</small></div><div class="body"><div class="stats"><div class="stat"><span>Faz</span><strong id="stateText">--</strong></div><div class="stat"><span>Son Mıknatıs</span><strong id="lastMagText">-</strong></div><div class="stat"><span>Bitişe Kalan</span><strong id="remainText">20.00 m</strong><div class="bar"><div id="remainBar"></div></div></div><div class="stat"><span>Tünel İçi</span><strong id="tunnelText">Hayır</strong><div class="bar"><div id="waitBar"></div></div></div></div></div></div>
        <div class="card chart-card"><div class="head"><h2>PWM Grafiği</h2><small>Sol + sağ motor PWM değerleri, son ölçümler</small></div><div class="body"><canvas id="speedChart" width="720" height="260"></canvas><div class="hint">Grafikte kalın çizgi ortalama PWM, ince çizgiler sol/sağ motor PWM değerini gösterir.</div></div></div>
        <div class="card"><div class="head"><h2>HC-SR04</h2><small>5 cm ve altı geçiş sayacı</small></div><div class="body"><div class="toggle" id="ultraInfo">Mesafe <span>--</span></div></div></div>
        <div class="card"><div class="head"><h2>Veriler</h2><small>Motor / sensör / yazılım</small></div><div class="body"><div class="switches"><div class="toggle" id="leftInfo">Sol motor <span>--</span></div><div class="toggle" id="rightInfo">Sağ motor <span>--</span></div><div class="toggle" id="sensorInfo">KY-024 <span>--</span></div><div class="toggle" id="firmwareInfo">Firmware <span>--</span></div><div class="toggle">Kural profili <span>20 m / 50 cm / parabolik fren</span></div></div></div></div>
      </div>
    </div>
  </div>
<script>
const $=id=>document.getElementById(id);
const el={track:$('track'),topScale:$('topScale'),bottomScale:$('bottomScale'),car:$('car'),tunnel:$('tunnel'),tunnelBeam:$('tunnelBeam'),finishLine:$('finishLine'),startBtn:$('startBtn'),pauseBtn:$('pauseBtn'),resumeBtn:$('resumeBtn'),stopBtn:$('stopBtn'),resetBtn:$('resetBtn'),manualForward:$('manualForward'),manualBack:$('manualBack'),manualStop:$('manualStop'),manualCenter:$('manualCenter'),manualBrake:$('manualBrake'),manualSpeedRange:$('manualSpeedRange'),manualSpeedText:$('manualSpeedText'),modeBadge:$('modeBadge'),stateBadge:$('stateBadge'),scoreBadge:$('scoreBadge'),timeBadge:$('timeBadge'),wifiBadge:$('wifiBadge'),pauseInfo:$('pauseInfo'),speedText:$('speedText'),posText:$('posText'),waitText:$('waitText'),frontCount:$('frontCount'),rearCount:$('rearCount'),trustText:$('trustText'),frontLed:$('frontLed'),rearLed:$('rearLed'),stateText:$('stateText'),lastMagText:$('lastMagText'),remainText:$('remainText'),remainBar:$('remainBar'),tunnelText:$('tunnelText'),waitBar:$('waitBar'),log:$('log'),speedChart:$('speedChart'),leftInfo:$('leftInfo'),rightInfo:$('rightInfo'),sensorInfo:$('sensorInfo'),firmwareInfo:$('firmwareInfo')};
let race={trackCm:2000,magnetIntervalCm:50,targetMagnets:40,tunnelStartCm:912,tunnelLengthCm:88,tunnelSensorCm:962,finalBrakeStartCm:1750,launchEndCm:200};
let manualTimer=null,wasConnected=false,lastPhase='',lastCount=-1; const history=[];
function clamp(n,min,max){return Math.max(min,Math.min(max,n));} function ratio(cm){return clamp(cm/Math.max(race.trackCm,1),0,1)} function mapPos(cm){const lp=34,rp=34,u=Math.max(1,el.track.clientWidth-lp-rp);return lp+u*ratio(cm)}
function addLog(text){const d=document.createElement('div');d.className='log-item';const t=new Date().toLocaleTimeString('tr-TR',{hour:'2-digit',minute:'2-digit',second:'2-digit'});d.textContent=`[${t}] ${text}`;el.log.prepend(d);while(el.log.children.length>32)el.log.lastChild.remove()}
function buildScale(scale){scale.innerHTML='';const lp=34,rp=34,u=Math.max(1,scale.clientWidth-lp-rp);for(let cm=0;cm<=race.trackCm;cm+=50){const x=lp+u*ratio(cm),major=cm%100===0;const tick=document.createElement('div');tick.style.cssText=`position:absolute;left:${x}px;transform:translateX(-50%);bottom:4px;width:2px;height:${major?'22':'12'}px;border-radius:999px;background:${major?'rgba(255,255,255,.45)':'rgba(255,255,255,.20)'}`;scale.appendChild(tick);if(major){const lab=document.createElement('div');lab.style.cssText=`position:absolute;left:${x}px;transform:translateX(-50%);top:3px;font-size:10px;font-weight:900;color:var(--muted);white-space:nowrap`;lab.textContent=(cm/100)+' m';scale.appendChild(lab)}}}
function buildTrack(){[...el.track.querySelectorAll('.sleeper,.magnet')].forEach(x=>x.remove());for(let cm=0;cm<=race.trackCm;cm+=race.magnetIntervalCm){const x=mapPos(cm);const s=document.createElement('div');s.className='sleeper';s.style.left=x+'px';el.track.appendChild(s);const m=document.createElement('div');m.className='magnet';m.style.left=x+'px';m.dataset.cm=cm;m.title=(cm/100).toFixed(2)+' m mıknatıs';el.track.appendChild(m)}buildScale(el.topScale);buildScale(el.bottomScale);el.tunnel.style.left=mapPos(race.tunnelStartCm)+'px';el.tunnel.style.width=Math.max(8,mapPos(race.tunnelStartCm+race.tunnelLengthCm)-mapPos(race.tunnelStartCm))+'px';el.tunnelBeam.style.left=mapPos(race.tunnelSensorCm)+'px';el.finishLine.style.left=(mapPos(race.trackCm)-6)+'px'}
function markMagnets(count){const passed=count*race.magnetIntervalCm;el.track.querySelectorAll('.magnet').forEach(m=>m.classList.toggle('read',Number(m.dataset.cm)<=passed && Number(m.dataset.cm)>0))}
function setBadges(phase){const nice={IDLE:['Hazır','warn'],LAUNCH:['Yumuşak kalkış','ok'],RUN:['Hızlı seyir','ok'],APPROACH:['Tünele yaklaşma','warn'],TUNNEL_CREEP:['Hassas sürünme','warn'],TUNNEL_WAIT:['Tünelde bekliyor','danger'],EXIT:['Tünelden çıkış','ok'],FINAL_BRAKE:['Parabolik fren','warn'],DONE:['Görev tamamlandı','ok'],PAUSED:['Duraklatıldı','warn'],MANUAL:['Manuel','warn'],STOPPED:['Acil durdu','danger']};const [txt,cls]=nice[phase]||[phase,'warn'];el.stateBadge.textContent='Durum: '+txt;el.stateBadge.className='badge '+cls;el.stateText.textContent=txt;el.pauseInfo.textContent=txt;el.modeBadge.textContent=(phase==='MANUAL')?'MOD: MANUEL':(phase==='PAUSED'?'MOD: PAUSED':'MOD: AUTO')}
function drawChart(){const c=el.speedChart,ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);ctx.strokeStyle='rgba(255,255,255,.12)';ctx.lineWidth=1;for(let i=0;i<=4;i++){const y=24+(h-46)*i/4;ctx.beginPath();ctx.moveTo(34,y);ctx.lineTo(w-10,y);ctx.stroke()}ctx.fillStyle='rgba(238,246,255,.72)';ctx.font='12px system-ui';ctx.fillText('255',8,28);ctx.fillText('0',18,h-18);function line(key,width,alpha){if(history.length<2)return;ctx.globalAlpha=alpha;ctx.strokeStyle='rgba(238,246,255,.95)';ctx.lineWidth=width;ctx.beginPath();history.forEach((p,i)=>{const x=34+(w-48)*(i/(120-1));const y=(h-18)-((h-46)*(Math.abs(p[key])/255));if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke();ctx.globalAlpha=1}line('left',1,.45);line('right',1,.45);line('avg',3,.95)}
async function api(path){const r=await fetch(path,{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);return r.text()}
function updateData(d){if(!wasConnected){addLog('ESP32 bağlantısı kuruldu.');wasConnected=true}if(d.race){race=d.race;buildTrack()}const phase=d.run.phase||'IDLE';setBadges(phase);const count=Number(d.magnet.count||0),pos=Number(d.run.positionCm ?? (count*race.magnetIntervalCm)),remain=Math.max(0,race.trackCm-pos);const leftPwm=Number(d.left.speed||0),rightPwm=Number(d.right.speed||0),avgPwm=Math.round((Math.abs(leftPwm)+Math.abs(rightPwm))/2);el.car.style.left=mapPos(pos)+'px';markMagnets(count);el.speedText.textContent=avgPwm;el.posText.textContent=(pos/100).toFixed(2);el.waitText.textContent=Number(d.run.tunnelWaitSeconds||0).toFixed(1);el.frontCount.textContent=count;el.rearCount.textContent=race.targetMagnets;el.trustText.textContent=avgPwm;el.lastMagText.textContent=count>0?count+'. ('+(count*0.5).toFixed(1)+' m)':'-';el.remainText.textContent=(remain/100).toFixed(2)+' m';el.remainBar.style.width=(ratio(pos)*100)+'%';const inTunnel=pos>=race.tunnelStartCm && pos<=race.tunnelStartCm+race.tunnelLengthCm;el.tunnelText.textContent=inTunnel?'Evet':'Hayır';el.waitBar.style.width=clamp(Number(d.run.tunnelWaitSeconds||0)/5,0,1)*100+'%';el.timeBadge.textContent='Süre: '+Number(d.run.elapsedSeconds||0).toFixed(1)+' sn';const x=Math.abs(race.trackCm-pos);const score=(phase==='IDLE')?null:100-(4*x+2*(Number(d.run.elapsedSeconds||0)-10))-Math.min(Math.abs(Number(d.run.tunnelWaitSeconds||0)-5),5);el.scoreBadge.textContent=score===null?'Skor: --':'Skor: '+score.toFixed(1);el.frontLed.classList.toggle('on',!!d.magnet.frontPresent);el.frontLed.classList.toggle('off',!d.magnet.frontPresent);el.rearLed.classList.toggle('on',!!d.magnet.rearPresent);el.rearLed.classList.toggle('off',!d.magnet.rearPresent);el.leftInfo.innerHTML='Sol motor <span>PWM '+leftPwm+'</span>';el.rightInfo.innerHTML='Sağ motor <span>PWM '+rightPwm+'</span>';el.sensorInfo.innerHTML='KY-024 <span>front DO '+d.magnet.frontRawLevel+' / rear DO '+d.magnet.rearRawLevel+'</span>';el.firmwareInfo.innerHTML='Firmware <span>'+d.version+'</span>';history.push({left:leftPwm,right:rightPwm,avg:avgPwm});while(history.length>120)history.shift();drawChart();if(phase!==lastPhase){addLog('Faz: '+phase);lastPhase=phase}if(count!==lastCount){if(count>0)addLog(count+'. ön mıknatıs okundu.');lastCount=count}el.wifiBadge.textContent='WiFi: Canlı';el.wifiBadge.className='badge ok'}
const baseUpdateData=updateData;updateData=function(d){baseUpdateData(d);const u=d.ultrasonic||{};const count=Number(u.passCount||0);const dist=Number(u.distanceCm||0);const valid=!!u.valid;const near=!!u.near;const info=valid?(dist.toFixed(1)+' cm / '+(near?'<=5 cm':'uzak')):'okuma yok';$('ultraCountText').textContent=count;$('ultraInfo').innerHTML='Mesafe <span>'+info+' / geçiş '+count+'</span>';}
async function refresh(){try{const r=await fetch('/data',{cache:'no-store'});updateData(await r.json())}catch(e){wasConnected=false;el.wifiBadge.textContent='WiFi: Kopuk';el.wifiBadge.className='badge danger';el.stateBadge.textContent='Durum: Bağlantı yok';el.stateBadge.className='badge danger'}}
async function startAuto(){await api('/start');addLog('Otonom kural profili başlatıldı.');refresh()}async function pauseSystem(){await api('/pause');addLog('Duraklatıldı.');refresh()}async function resumeSystem(){await api('/resume');addLog('Devam ettirildi.');refresh()}async function stopAll(){stopManual(false);await api('/stop');addLog('Acil durdurma gönderildi.');refresh()}async function resetAll(){await api('/reset');history.length=0;addLog('Sayaç ve simülasyon sıfırlandı.');refresh()}
function manualSpeed(){return Number(el.manualSpeedRange.value||90)}async function sendManual(dir){await api('/manual?dir='+encodeURIComponent(dir)+'&speed='+manualSpeed())}function startManual(dir){stopManual(false);sendManual(dir).catch(()=>{});manualTimer=setInterval(()=>sendManual(dir).catch(()=>{}),350);addLog(dir==='forward'?'Manuel ileri.':'Manuel geri.')}function stopManual(log=true){if(manualTimer){clearInterval(manualTimer);manualTimer=null}sendManual('stop').catch(()=>{});if(log)addLog('Manuel komut durdu.')}
el.startBtn.onclick=startAuto;el.pauseBtn.onclick=pauseSystem;el.resumeBtn.onclick=resumeSystem;el.stopBtn.onclick=stopAll;el.resetBtn.onclick=resetAll;el.manualSpeedRange.oninput=()=>el.manualSpeedText.textContent=el.manualSpeedRange.value;el.manualForward.onmousedown=()=>startManual('forward');el.manualForward.onmouseup=()=>stopManual();el.manualForward.onmouseleave=()=>stopManual(false);el.manualBack.onmousedown=()=>startManual('back');el.manualBack.onmouseup=()=>stopManual();el.manualBack.onmouseleave=()=>stopManual(false);el.manualStop.onclick=()=>stopManual();el.manualCenter.onclick=()=>stopManual();el.manualBrake.onclick=pauseSystem;['touchstart','touchend','touchcancel'].forEach(evt=>{el.manualForward.addEventListener(evt,e=>{e.preventDefault();evt==='touchstart'?startManual('forward'):stopManual(false)},{passive:false});el.manualBack.addEventListener(evt,e=>{e.preventDefault();evt==='touchstart'?startManual('back'):stopManual(false)},{passive:false})});document.addEventListener('keydown',e=>{if(e.repeat)return;if(e.key==='ArrowRight'){e.preventDefault();startManual('forward')}if(e.key==='ArrowLeft'){e.preventDefault();startManual('back')}if(e.key===' '){e.preventDefault();stopManual()}});document.addEventListener('keyup',e=>{if(e.key==='ArrowRight'||e.key==='ArrowLeft')stopManual(false)});window.addEventListener('resize',()=>{buildTrack();refresh()});buildTrack();addLog('Panel açıldı. ESP32 verisi bekleniyor.');setInterval(refresh,250);refresh();
</script>
</body>
</html>
)rawliteral";

bool readMagnetActiveRaw(uint8_t pin) {
  const bool rawHigh = digitalRead(pin) == HIGH;
  return MAGNET_ACTIVE_LOW ? !rawHigh : rawHigh;
}

void setupMotor(const MotorDriver &motor) {
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

void applySpeed(int speed) {
  leftSpeed = speed;
  rightSpeed = speed;
  setMotorSpeed(leftMotor, speed);
  setMotorSpeed(rightMotor, speed);
}

void stopMotors() {
  targetSpeed = 0;
  rampSpeed = 0;
  applySpeed(0);
}

const char *phaseText() {
  if (manualActive) return "MANUAL";
  if (runPaused) return "PAUSED";
  if (targetReached) return "DONE";
  if (!runActive) return "IDLE";
  if (tunnelWaiting) return "TUNNEL_WAIT";
  if (tunnelCreepActive) return "TUNNEL_CREEP";
  if (!tunnelHandled && magnetCount >= 17) return "APPROACH";
  if (tunnelHandled && magnetCount >= FINAL_BRAKE_START_COUNT) return "FINAL_BRAKE";
  if (tunnelHandled && magnetCount < FINAL_BRAKE_START_COUNT) return "EXIT";
  if (magnetCount < LAUNCH_END_COUNT) return "LAUNCH";
  return "RUN";
}

uint16_t estimatedPositionCm() {
  uint16_t pos = (uint16_t)magnetCount * MAGNET_INTERVAL_CM;
  if (tunnelCreepActive || tunnelWaiting) pos = TUNNEL_SENSOR_CM;
  if (targetReached) pos = RACE_LENGTH_CM;
  return (pos > RACE_LENGTH_CM) ? RACE_LENGTH_CM : pos;
}

int plannedAutoSpeed() {
  if (!runActive || runPaused || targetReached) return 0;

  if (!tunnelHandled) {
    if (tunnelWaiting) return 0;
    if (tunnelCreepActive) return CREEP_SPEED;
    if (magnetCount >= TUNNEL_STOP_COUNT) return CREEP_SPEED;
    if (magnetCount >= 17) return 70; // Tünele yaklaşırken güvenli hız.
  }

  if (magnetCount < LAUNCH_END_COUNT) {
    const float r = float(magnetCount + 1) / float(LAUNCH_END_COUNT);
    return LAUNCH_MIN_SPEED + int((CRUISE_SPEED - LAUNCH_MIN_SPEED) * r * r);
  }

  if (tunnelHandled && magnetCount >= FINAL_BRAKE_START_COUNT) {
    const int brakeCounts = TARGET_MAGNET_COUNT - FINAL_BRAKE_START_COUNT;
    const int remaining = max(0, int(TARGET_MAGNET_COUNT) - int(magnetCount));
    const float r = constrain(float(remaining) / float(brakeCounts), 0.0f, 1.0f);
    return FINAL_MIN_SPEED + int((CRUISE_SPEED - FINAL_MIN_SPEED) * r * r); // Parabolik yavaşlama.
  }

  return CRUISE_SPEED;
}

void resetRunState() {
  magnetCount = 0;
  ultrasonicPassCount = 0;
  ultrasonicObjectNear = false;
  ultrasonicReadingValid = false;
  ultrasonicDistanceCm = -1.0f;
  runActive = false;
  runPaused = false;
  manualActive = false;
  targetReached = false;
  tunnelCreepActive = false;
  tunnelWaiting = false;
  tunnelHandled = false;
  missionStartMs = 0;
  tunnelCreepStartedMs = 0;
  tunnelWaitStartedMs = 0;
  finishMs = 0;
  stopMotors();
}

void startRun() {
  resetRunState();
  runActive = true;
  missionStartMs = millis();
  lastRampMs = millis();
  rampSpeed = 0;
  targetSpeed = LAUNCH_MIN_SPEED;
}

void updateRampControl() {
  const uint32_t now = millis();

  if (manualActive) {
    if (now - lastManualCommandMs > MANUAL_COMMAND_TIMEOUT_MS) {
      manualActive = false;
      stopMotors();
    }
    return;
  }

  if (!runActive || runPaused) {
    return;
  }

  if (magnetCount >= TARGET_MAGNET_COUNT) {
    runActive = false;
    targetReached = true;
    finishMs = now;
    stopMotors();
    return;
  }

  if (!tunnelHandled) {
    if (tunnelWaiting) {
      if (now - tunnelWaitStartedMs >= TUNNEL_WAIT_MS) {
        tunnelWaiting = false;
        tunnelHandled = true;
        rampSpeed = 0;
        lastRampMs = now;
      } else {
        stopMotors();
        return;
      }
    } else if (tunnelCreepActive) {
      if (now - tunnelCreepStartedMs >= TUNNEL_CREEP_MS) {
        tunnelCreepActive = false;
        tunnelWaiting = true;
        tunnelWaitStartedMs = now;
        stopMotors();
        return;
      }
    } else if (magnetCount >= TUNNEL_STOP_COUNT) {
      tunnelCreepActive = true;
      tunnelCreepStartedMs = now;
    }
  }

  targetSpeed = plannedAutoSpeed();
  if (now - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    if (rampSpeed < targetSpeed) {
      rampSpeed = min(rampSpeed + RAMP_UP_STEP, targetSpeed);
    } else if (rampSpeed > targetSpeed) {
      rampSpeed = max(rampSpeed - RAMP_DOWN_STEP, targetSpeed);
    }
    applySpeed(rampSpeed);
  }
}

void updateMagnetCounter() {
  const bool rawState = readMagnetActiveRaw(FRONT_MAGNET_PIN);
  const bool rearRawState = readMagnetActiveRaw(REAR_MAGNET_PIN);
  const uint32_t now = millis();

  if (rawState != lastRawFrontMagnetState) {
    lastRawFrontMagnetState = rawState;
    lastMagnetChangeMs = now;
  }

  if (rearRawState != lastRawRearMagnetState) {
    lastRawRearMagnetState = rearRawState;
    lastRearMagnetChangeMs = now;
  }

  if (now - lastRearMagnetChangeMs >= MAGNET_DEBOUNCE_MS && rearRawState != stableRearMagnetState) {
    stableRearMagnetState = rearRawState;
    rearMagnetPresent = stableRearMagnetState;
  }

  if (now - lastMagnetChangeMs < MAGNET_DEBOUNCE_MS) {
    frontMagnetPresent = stableFrontMagnetState;
    rearMagnetPresent = stableRearMagnetState;
    return;
  }

  if (rawState != stableFrontMagnetState) {
    stableFrontMagnetState = rawState;
    frontMagnetPresent = stableFrontMagnetState;

    if (stableFrontMagnetState && runActive && !runPaused && !manualActive && magnetCount < TARGET_MAGNET_COUNT) {
      magnetCount++;
    }
  }

  frontMagnetPresent = stableFrontMagnetState;
  rearMagnetPresent = stableRearMagnetState;
}

float readUltrasonicDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  const uint32_t durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0f;
  }

  return (durationUs * 0.0343f) / 2.0f;
}

void updateUltrasonicCounter() {
  const uint32_t now = millis();
  if (now - lastUltrasonicReadMs < ULTRASONIC_READ_INTERVAL_MS) {
    return;
  }

  lastUltrasonicReadMs = now;
  ultrasonicDistanceCm = readUltrasonicDistanceCm();
  ultrasonicReadingValid = ultrasonicDistanceCm > 0.0f;

  if (!ultrasonicReadingValid) {
    ultrasonicObjectNear = false;
    return;
  }

  if (!ultrasonicObjectNear && ultrasonicDistanceCm <= ULTRASONIC_PASS_THRESHOLD_CM) {
    ultrasonicObjectNear = true;
    ultrasonicPassCount++;
    return;
  }

  if (ultrasonicObjectNear && ultrasonicDistanceCm >= ULTRASONIC_RELEASE_THRESHOLD_CM) {
    ultrasonicObjectNear = false;
  }
}

String motorJson(int speed) {
  String json = "{";
  json += "\"speed\":" + String(speed);
  json += "}";
  return json;
}

void sendNoCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

void handleRoot() {
  sendNoCache();
  server.send_P(200, "text/html", indexHtml);
}

void handleData() {
  uint32_t elapsedMs = 0;
  if (missionStartMs > 0) {
    elapsedMs = (finishMs > 0 ? finishMs : millis()) - missionStartMs;
  }

  float tunnelWaitSeconds = 0.0f;
  if (tunnelWaiting && tunnelWaitStartedMs > 0) {
    tunnelWaitSeconds = min(5.0f, (millis() - tunnelWaitStartedMs) / 1000.0f);
  } else if (tunnelHandled) {
    tunnelWaitSeconds = 5.0f;
  }

  String json = "{";
  json += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"left\":" + motorJson(leftSpeed) + ",";
  json += "\"right\":" + motorJson(rightSpeed) + ",";
  json += "\"race\":{\"trackCm\":" + String(RACE_LENGTH_CM) + ",\"magnetIntervalCm\":" + String(MAGNET_INTERVAL_CM) + ",\"targetMagnets\":" + String(TARGET_MAGNET_COUNT) + ",\"tunnelStartCm\":" + String(TUNNEL_START_CM) + ",\"tunnelLengthCm\":" + String(TUNNEL_LENGTH_CM) + ",\"tunnelSensorCm\":" + String(TUNNEL_SENSOR_CM) + ",\"finalBrakeStartCm\":" + String(FINAL_BRAKE_START_CM) + ",\"launchEndCm\":" + String(LAUNCH_END_CM) + "},";
  json += "\"magnet\":{\"count\":" + String(magnetCount) + ",\"target\":" + String(TARGET_MAGNET_COUNT) + ",";
  json += "\"present\":" + String(frontMagnetPresent ? "true" : "false") + ",\"rawLevel\":" + String(digitalRead(FRONT_MAGNET_PIN)) + ",";
  json += "\"frontPresent\":" + String(frontMagnetPresent ? "true" : "false") + ",\"frontRawLevel\":" + String(digitalRead(FRONT_MAGNET_PIN)) + ",";
  json += "\"rearPresent\":" + String(rearMagnetPresent ? "true" : "false") + ",\"rearRawLevel\":" + String(digitalRead(REAR_MAGNET_PIN)) + "},";
  json += "\"ultrasonic\":{\"passCount\":" + String(ultrasonicPassCount) + ",";
  json += "\"distanceCm\":" + String(ultrasonicReadingValid ? ultrasonicDistanceCm : -1.0f, 2) + ",";
  json += "\"valid\":" + String(ultrasonicReadingValid ? "true" : "false") + ",";
  json += "\"near\":" + String(ultrasonicObjectNear ? "true" : "false") + ",";
  json += "\"thresholdCm\":" + String(ULTRASONIC_PASS_THRESHOLD_CM, 1) + "},";
  json += "\"run\":{\"active\":" + String(runActive ? "true" : "false") + ",";
  json += "\"paused\":" + String(runPaused ? "true" : "false") + ",";
  json += "\"manual\":" + String(manualActive ? "true" : "false") + ",";
  json += "\"targetReached\":" + String(targetReached ? "true" : "false") + ",";
  json += "\"phase\":\"" + String(phaseText()) + "\",";
  json += "\"positionCm\":" + String(estimatedPositionCm()) + ",";
  json += "\"targetSpeed\":" + String(targetSpeed) + ",";
  json += "\"rampSpeed\":" + String(rampSpeed) + ",";
  json += "\"elapsedSeconds\":" + String(elapsedMs / 1000.0f, 2) + ",";
  json += "\"tunnelWaitSeconds\":" + String(tunnelWaitSeconds, 2) + "}";
  json += "}";

  sendNoCache();
  server.send(200, "application/json", json);
}

void handleVersion() {
  sendNoCache();
  server.send(200, "text/plain", FIRMWARE_VERSION);
}

void handleStart() {
  startRun();
  server.send(200, "text/plain", "STARTED_RULE_PROFILE");
}

void handlePause() {
  if (runActive && !targetReached) {
    runPaused = true;
    stopMotors();
  }
  server.send(200, "text/plain", "PAUSED");
}

void handleResume() {
  if (!targetReached && missionStartMs > 0) {
    runPaused = false;
    runActive = true;
    manualActive = false;
    lastRampMs = millis();
  }
  server.send(200, "text/plain", "RESUMED");
}

void handleStop() {
  runActive = false;
  runPaused = false;
  manualActive = false;
  targetReached = false;
  stopMotors();
  server.send(200, "text/plain", "STOPPED");
}

void handleReset() {
  resetRunState();
  server.send(200, "text/plain", "RESET");
}

void handleManual() {
  const String dir = server.arg("dir");
  int speed = server.arg("speed").toInt();
  if (speed <= 0) speed = MANUAL_DEFAULT_SPEED;
  speed = constrain(speed, 0, MANUAL_MAX_SPEED);

  runActive = false;
  runPaused = false;
  targetReached = false;
  manualActive = true;
  lastManualCommandMs = millis();

  if (dir == "forward") {
    rampSpeed = speed;
    targetSpeed = speed;
    applySpeed(speed);
    server.send(200, "text/plain", "MANUAL_FORWARD");
    return;
  }

  if (dir == "back") {
    rampSpeed = -speed;
    targetSpeed = -speed;
    applySpeed(-speed);
    server.send(200, "text/plain", "MANUAL_BACK");
    return;
  }

  manualActive = false;
  stopMotors();
  server.send(200, "text/plain", "MANUAL_STOP");
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

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  pinMode(FRONT_MAGNET_PIN, INPUT_PULLUP);
  pinMode(REAR_MAGNET_PIN, INPUT_PULLUP);
  lastRawFrontMagnetState = readMagnetActiveRaw(FRONT_MAGNET_PIN);
  stableFrontMagnetState = lastRawFrontMagnetState;
  frontMagnetPresent = stableFrontMagnetState;
  lastRawRearMagnetState = readMagnetActiveRaw(REAR_MAGNET_PIN);
  stableRearMagnetState = lastRawRearMagnetState;
  rearMagnetPresent = stableRearMagnetState;

  setupMotor(leftMotor);
  setupMotor(rightMotor);
  resetRunState();
  setupWifiAP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/version", HTTP_GET, handleVersion);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/pause", HTTP_GET, handlePause);
  server.on("/resume", HTTP_GET, handleResume);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/manual", HTTP_GET, handleManual);
  server.begin();

  Serial.println("ESP32 rail rule simulator and control interface ready");
}

void loop() {
  server.handleClient();
  updateMagnetCounter();
  updateUltrasonicCounter();
  updateRampControl();
}
