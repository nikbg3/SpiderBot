#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "webserver.h"

const char* password = "12345678";

int Servo_PROGRAM = 0;
int GPIO_ID = 100;
String ival;

ESP8266WebServer server(80);

const RobotDeviceInfo kRobotDeviceInfo = {
    "QuadBot-E",
    "1.0.0",
    "v1"
};

static String g_deviceName = "Robot";
static int g_currentActionId = PROGRAM_NONE;
static bool g_robotBusy = false;
static bool g_stopRequested = false;
static int g_stopTargetProgram = PROGRAM_STANDBY;
static volatile NavigationControl g_navigationControl = NAV_CONTROL_NONE;
static NavigationTargetKind g_navigationTargetKind = NAV_TARGET_NONE;
static NavigationWaypoint g_navigationWaypoints[MAX_NAVIGATION_WAYPOINTS];
static uint8_t g_navigationWaypointCount = 0;
static bool g_debugUiEnabled = false;
static bool g_wifiApReady = false;
static NavigationTelemetry g_navigationTelemetry = {
    "INITIALIZING", "Starting up", -1.0f, 0.0f, 0.0f, 0, -15, -15, 0,
    false, {0}, {0}
};

static const RobotActionInfo kRobotActions[] = {
    {PROGRAM_STANDBY, "standby", "待机"},
    {PROGRAM_FORWARD, "forward", "前进"},
    {PROGRAM_BACKWARD, "backward", "后退"},
    {PROGRAM_LEFTSHIFT, "leftshift", "左移"},
    {PROGRAM_RIGHTSHIFT, "rightshift", "右移"},
    {PROGRAM_TURNLEFT, "turnleft", "左转"},
    {PROGRAM_TURNRIGHT, "turnright", "右转"},
    {PROGRAM_LIE, "lie", "躺下"},
    {PROGRAM_HELLO, "hello", "打招呼"},
    {PROGRAM_FIGHTING, "fighting", "战斗"},
    {PROGRAM_PUSHUP, "pushup", "俯卧撑"},
    {PROGRAM_SLEEP, "sleep", "睡眠"},
    {PROGRAM_DANCE1, "dance1", "舞步1"},
    {PROGRAM_DANCE2, "dance2", "舞步2"},
    {PROGRAM_DANCE3, "dance3", "舞步3"},
    {PROGRAM_AVOID, "avoid", "避障"},
    {PROGRAM_CENTER, "center", "中位"},
    {PROGRAM_ZERO, "zero", "归零"}
};

// ── Single-page UI stored in Flash (PROGMEM) ─────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>QuadBot Debug</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{font-family:Arial,sans-serif;background:#0f172a;color:#e5e7eb;min-height:100vh;overflow:auto}
.page{min-height:100vh;display:flex;flex-direction:column}
.bar{position:sticky;top:0;z-index:10;display:flex;align-items:center;justify-content:space-between;gap:8px;padding:12px 14px;background:#111827;border-bottom:1px solid #243041}
.title{font-size:1rem;font-weight:700;color:#93c5fd}
.bar-actions{display:flex;gap:8px}
.tool-btn{border:none;border-radius:10px;padding:9px 12px;font-size:.88rem;font-weight:700;cursor:pointer;color:#f8fafc;background:#1f3b64;touch-action:manipulation}
.tool-btn.alt{background:#374151}
.main{flex:1;display:flex;flex-direction:column;gap:14px;padding:14px;padding-bottom:max(18px,env(safe-area-inset-bottom))}
.panel{background:#111827;border:1px solid #243041;border-radius:16px;padding:12px}
.panel-title{font-size:.84rem;font-weight:700;color:#93c5fd;margin-bottom:10px;text-transform:uppercase;letter-spacing:.04em}
.grid{display:grid;gap:10px}
.grid3{grid-template-columns:repeat(3,minmax(0,1fr))}
.btn{min-height:56px;width:100%;border:none;border-radius:12px;font-size:.92rem;font-weight:700;cursor:pointer;color:#1f2937;touch-action:manipulation}
.btn:active{opacity:.72}
.dir{background:#bfdbfe}
.turn{background:#bbf7d0}
.state{background:#fde68a}
.act{background:#fed7aa}
.sleep{background:#fecaca}
.dance{background:#ddd6fe}
.overlay{position:fixed;inset:0;z-index:30;display:flex;align-items:flex-end;justify-content:center;background:rgba(0,0,0,.72);opacity:0;pointer-events:none;transition:opacity .16s ease}
.overlay.show{opacity:1;pointer-events:auto}
.sheet{width:min(560px,100%);max-height:88vh;background:#111827;border-radius:18px 18px 0 0;border:1px solid #243041;border-bottom:none;display:flex;flex-direction:column}
.sheet-head{display:flex;align-items:center;justify-content:space-between;padding:14px 16px;border-bottom:1px solid #243041}
.sheet-title{font-size:.95rem;font-weight:700;color:#93c5fd}
.sheet-body{padding:12px;overflow:auto}
.close-btn{border:none;background:none;color:#9ca3af;font-size:1.4rem;line-height:1;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:.84rem}
th,td{padding:8px 6px;text-align:center;border-bottom:1px solid #1f2937}
th{color:#9ca3af;font-weight:600}
.group td{padding-top:12px;padding-bottom:6px;text-align:left;color:#93c5fd;font-size:.78rem;font-weight:700;border-bottom:none}
input[type=number]{width:68px;padding:6px 4px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:#f8fafc;text-align:center}
.set-btn{border:none;border-radius:8px;background:#14532d;color:#dcfce7;padding:7px 10px;font-size:.8rem;font-weight:700;cursor:pointer}
.steering-calibration{display:flex;align-items:center;gap:10px;margin-top:10px}
.steering-calibration label{flex:1;font-size:.82rem;color:#cbd5e1}
.wide-btn{width:100%;margin-top:12px;padding:11px;border:none;border-radius:10px;background:#1d4ed8;color:#eff6ff;font-size:.9rem;font-weight:700;cursor:pointer}
.hint{font-size:.78rem;color:#94a3b8;margin-top:8px}
.nav-grid{display:grid;grid-template-columns:minmax(240px,360px) minmax(180px,1fr);gap:16px;align-items:center}
.nav-map{display:block;width:100%;height:auto;aspect-ratio:1;background:#24313f;border:1px solid #475569;border-radius:8px;image-rendering:pixelated}
.nav-stats{font-size:.82rem;line-height:1.8;color:#cbd5e1}
.nav-state{font-size:1rem;font-weight:700;color:#86efac}
.nav-notice{display:none;margin:8px 0;padding:9px 11px;border:1px solid #2563eb;border-radius:9px;background:#172554;color:#dbeafe;font-size:.84rem;line-height:1.4}
.nav-notice.show{display:block}
.nav-notice.error{border-color:#b91c1c;background:#450a0a;color:#fee2e2}
.nav-note{font-size:.72rem;color:#94a3b8;margin-top:6px}
.nav-legend{display:flex;flex-wrap:wrap;gap:8px 14px;margin-top:10px;font-size:.75rem;color:#cbd5e1}
.nav-legend-item{display:flex;align-items:center;gap:5px}
.nav-swatch{width:12px;height:12px;border:1px solid #64748b;border-radius:2px}
.nav-actions{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px}
.nav-action-btn{border:0;border-radius:9px;padding:9px 12px;background:#1d4ed8;color:#eff6ff;font-size:.82rem;font-weight:700;cursor:pointer;touch-action:manipulation}
.nav-action-btn.alt{background:#374151}
.nav-grid-host{width:100%}
.map-wrap{position:relative;min-width:0}
.map-fullscreen{position:fixed;inset:0;z-index:40;display:none;background:#0b1120;padding:max(12px,env(safe-area-inset-top)) 14px max(12px,env(safe-area-inset-bottom));overflow:auto}
.map-fullscreen.show{display:flex;flex-direction:column}
.map-fullscreen-head{display:flex;align-items:center;justify-content:space-between;gap:12px;padding-bottom:12px}
.map-fullscreen-title{font-size:1rem;font-weight:700;color:#93c5fd}
.map-fullscreen .nav-grid{flex:1;grid-template-columns:minmax(0,1fr) minmax(200px,260px);align-content:center}
.map-fullscreen .nav-map{width:min(100%,calc(100vh - 50px));max-height:calc(100vh - 92px);margin:auto}
.map-fullscreen .nav-stats{font-size:1rem;line-height:2}
.map-context{position:fixed;z-index:50;display:none;min-width:170px;padding:6px;background:#111827;border:1px solid #475569;border-radius:10px;box-shadow:0 10px 28px rgba(0,0,0,.5)}
.map-context.show{display:grid;gap:4px}
.map-context button{border:0;border-radius:7px;padding:10px;text-align:left;background:#1f2937;color:#f8fafc;font-size:.84rem;font-weight:700;cursor:pointer}
.map-context button:hover{background:#1d4ed8}
.map-context button:disabled{opacity:.45}
.map-context{max-height:80vh;overflow-y:auto}
#mapAnimationOptions{display:grid;gap:4px}
@media(max-width:640px){.nav-grid{grid-template-columns:1fr}.nav-map{width:min(100%,360px);margin:auto}.map-fullscreen .nav-grid{grid-template-columns:1fr}.map-fullscreen .nav-map{width:min(100%,calc(100vw - 28px),calc(100vh - 230px));max-height:none}.map-fullscreen .nav-stats{display:grid;grid-template-columns:1fr 1fr;gap:0 10px;font-size:.86rem;line-height:1.7}}
</style>
</head>
<body>
<div class="page">
  <div class="bar">
    <div class="title">QuadBot Debug</div>
    <div class="bar-actions">
      <button class="tool-btn" onclick="openCal()">Calibration</button>
      <button class="tool-btn alt" onclick="hideDebug()">Hide</button>
    </div>
  </div>

  <div class="main">
    <div class="panel">
      <div class="panel-title">Navigation</div>
      <div class="nav-actions">
        <button class="nav-action-btn" onclick="controlNavigation('resume')">Resume</button>
        <button class="nav-action-btn alt" onclick="controlNavigation('restart')">Restart autonomy</button>
        <button class="nav-action-btn alt" onclick="openMapFullscreen()">Full screen map</button>
        <button class="nav-action-btn alt" id="voiceToggle" onclick="toggleNavigationVoice()" aria-pressed="false">Mute voice</button>
      </div>
      <div id="navNotice" class="nav-notice" role="status" aria-live="polite"></div>
      <div id="navGridHost" class="nav-grid-host">
        <div class="nav-grid" id="navGrid">
          <div class="map-wrap">
            <canvas id="navMap" class="nav-map" width="360" height="360" aria-label="Occupancy grid showing robot position, orientation, and home position"></canvas>
          </div>
          <div class="nav-stats">
          <div class="nav-state" id="navState">Waiting for telemetry</div>
          <div>Front range: <span id="navRange">--</span></div>
          <div>Distance travelled: <span id="navTravel">0</span> cm</div>
          <div>Area covered (map): <span id="navArea">0.00</span> m&sup2;</div>
          <div>Heading: <span id="navHeading">0</span>&deg;</div>
          <div>Recovery attempts: <span id="navRecovery">0</span></div>
          </div>
        </div>
      </div>
      <div class="nav-legend">
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#24313f"></i>Unknown</span>
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#62b6be"></i>Observed clear</span>
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#111827"></i>Obstacle</span>
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#f8fafc"></i>Robot path</span>
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#4f46e5"></i>Robot</span>
        <span class="nav-legend-item"><i class="nav-swatch" style="background:#22c55e"></i>Home</span>
      </div>
      <div class="nav-note">Right-click the map (or long-press on touch) for Go here, animations, routes, pause/resume, and Return to home. Route points can be added with an animation; the robot plays it and continues, pausing at the final point. A single animated destination resumes coverage after the animation. Coverage uses 8 cm forward steps and 18.70&deg; turns, with a scaled final gait for exact 90&deg;, 180&deg;, and 360&deg; rotations. Map targets stay anchored as the map recenters.</div>
    </div>

    <div class="panel">
      <div class="panel-title">Directional Control</div>
      <div class="grid grid3">
        <button class="btn turn" onclick="pm(6)">Turn Left</button>
        <button class="btn dir" onclick="pm(2)">Forward</button>
        <button class="btn turn" onclick="pm(7)">Turn Right</button>
        <button class="btn dir" onclick="pm(4)">Left Shift</button>
        <button class="btn dir" onclick="pm(3)">Backward</button>
        <button class="btn dir" onclick="pm(5)">Right Shift</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title">Actions</div>
      <div class="grid grid3">
        <button class="btn state" onclick="pm(1)">Standby</button>
        <button class="btn act" onclick="pm(9)">Hello</button>
        <button class="btn act" onclick="pm(11)">Push Up</button>
        <button class="btn act" onclick="pm(8)">Lie</button>
        <button class="btn act" onclick="pm(10)">Fighting</button>
        <button class="btn sleep" onclick="pm(12)">Sleep</button>
        <button class="btn dance" onclick="pm(13)">Dance 1</button>
        <button class="btn dance" onclick="pm(14)">Dance 2</button>
        <button class="btn dance" onclick="pm(15)">Dance 3</button>
      </div>
    </div>
  </div>
</div>

<div id="overlay" class="overlay" onclick="bgClose(event)">
  <div class="sheet">
    <div class="sheet-head">
      <div class="sheet-title">Calibration</div>
      <button class="close-btn" onclick="closeCal()">&times;</button>
    </div>
    <div class="sheet-body">
      <table>
        <thead>
          <tr><th>Pin</th><th>Part</th><th>Axis</th><th>Offset</th><th></th></tr>
        </thead>
        <tbody>
          <tr class="group"><td colspan="5">Right Front</td></tr>
          <tr><td>D14</td><td>Foot</td><td>Swing</td><td><input type="number" id="s0" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(0)">SET</button></td></tr>
          <tr><td>D12</td><td>Leg</td><td>Lift</td><td><input type="number" id="s1" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(1)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Right Rear</td></tr>
          <tr><td>D13</td><td>Leg</td><td>Lift</td><td><input type="number" id="s2" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(2)">SET</button></td></tr>
          <tr><td>D15</td><td>Foot</td><td>Swing</td><td><input type="number" id="s3" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(3)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Left Front</td></tr>
          <tr><td>D16</td><td>Foot</td><td>Swing</td><td><input type="number" id="s4" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(4)">SET</button></td></tr>
          <tr><td>D5</td><td>Leg</td><td>Lift</td><td><input type="number" id="s5" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(5)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Left Rear</td></tr>
          <tr><td>D4</td><td>Leg</td><td>Lift</td><td><input type="number" id="s6" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(6)">SET</button></td></tr>
          <tr><td>D2</td><td>Foot</td><td>Swing</td><td><input type="number" id="s7" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(7)">SET</button></td></tr>
        </tbody>
      </table>
      <div class="panel-title">Forward steering correction</div>
      <p class="hint">Positive values counter left drift; negative values counter right drift. Enter degrees of drift per meter. For a 45&deg; left drift over 1.5 m, start at +30&deg;/m. Saved settings persist and apply on the next forward gait.</p>
      <div class="steering-calibration">
        <label for="steeringCorrection">Drift (degrees per meter)</label>
        <input type="number" id="steeringCorrection" min="-90" max="90" step="1" value="30">
        <button class="set-btn" onclick="saveSteeringCorrection()">SET</button>
      </div>
      <div class="panel-title">Turn calibration</div>
      <p class="hint">Enter the physical angle you measured after the latest 360&deg; command. Each save refines the existing correction using the measured result, so you can repeat the test after saving. The correction applies equally to left and right turns and assumes both directions behave similarly.</p>
      <div class="steering-calibration">
        <label for="rotation360Observed">Measured angle</label>
        <input type="number" id="rotation360Observed" min="300" max="420" step="1" value="360">
        <span>deg</span>
      </div>
      <div id="rotationCalibrationCurrent" class="hint" aria-live="polite">Current effective calibration: 360&deg; per commanded 360&deg;</div>
      <button class="wide-btn" onclick="saveRotationCalibration()">Save universal turn calibration</button>
      <button class="wide-btn" onclick="pm(100)">Go To Zero Pose</button>
      <div class="hint">Offset range: -30 to +30 is recommended for quick calibration.</div>
    </div>
  </div>
</div>

<div id="mapFullscreen" class="map-fullscreen" aria-label="Full screen robot map">
  <div class="map-fullscreen-head">
    <div class="map-fullscreen-title">Robot map and navigation</div>
    <button class="tool-btn alt" onclick="closeMapFullscreen()">Close map</button>
  </div>
  <div id="fullscreenMapSlot"></div>
</div>
<div id="mapContextMenu" class="map-context" role="menu">
  <button id="mapGoButton" onclick="mapMenuAction('goto')">Go here</button>
  <button id="mapRouteButton" onclick="mapMenuAction('route')">Draw route</button>
  <button id="mapAddRoutePointButton" onclick="mapMenuAction('add-route-point')">Add route point</button>
  <div id="mapAnimationOptions"></div>
  <button id="mapPauseButton" onclick="mapMenuAction('pause')">Pause</button>
  <button onclick="mapMenuAction('home')">Return to home</button>
  <button id="mapCancelRouteButton" onclick="mapMenuAction('cancel')">Cancel route</button>
</div>

<script>
var latestNavigation = null;
var selectedMapPoint = null;
var plannedRoute = [];
var drawingRoute = false;
var mapAnimations = [
  {id:9,name:'Hello'}, {id:11,name:'Push Up'}, {id:8,name:'Lie'},
  {id:10,name:'Fighting / Attack'}, {id:12,name:'Sleep'},
  {id:13,name:'Dance 1'}, {id:14,name:'Dance 2'}, {id:15,name:'Dance 3'}
];
var longPressTimer = 0;
var ignoreNextMapClick = false;
var lastSpokenNavigationUpdate = null;
var navigationVoiceMuted = false;
try {
  navigationVoiceMuted = localStorage.getItem('navigationVoiceMuted') === 'true';
} catch (error) {}
function showNavigationNotice(message, isError){
  var notice = document.getElementById('navNotice');
  notice.textContent = message;
  notice.className = 'nav-notice show' + (isError ? ' error' : '');
}
function pm(v){
  fetch('/controller?pm=' + v, { cache: 'no-store' })
    .then(function(response){
      if(!response.ok) throw new Error('Command rejected by robot');
      showNavigationNotice('Command accepted');
    })
    .catch(function(error){ showNavigationNotice(error.message, true); });
}
function controlNavigation(command){
  fetch('/api/v1/navigation/control?command=' + encodeURIComponent(command), { method: 'POST', cache: 'no-store' })
    .then(function(response){ return response.json(); })
    .then(function(response){
      if(!response.ok) throw new Error(response.message || 'Navigation command failed');
      showNavigationNotice(response.message || 'Navigation command accepted');
      updateNavigation();
    })
    .catch(function(error){
      showNavigationNotice(error.message || 'Navigation command failed', true);
    });
}
function openMapFullscreen(){
  var grid = document.getElementById('navGrid');
  document.getElementById('fullscreenMapSlot').appendChild(grid);
  document.getElementById('mapFullscreen').classList.add('show');
  if(document.documentElement.requestFullscreen) document.documentElement.requestFullscreen().catch(function(){});
}
function closeMapFullscreen(){
  var grid = document.getElementById('navGrid');
  document.getElementById('navGridHost').appendChild(grid);
  document.getElementById('mapFullscreen').classList.remove('show');
  if(document.fullscreenElement && document.exitFullscreen) document.exitFullscreen().catch(function(){});
}
document.addEventListener('fullscreenchange', function(){
  if(!document.fullscreenElement && document.getElementById('mapFullscreen').classList.contains('show')){
    closeMapFullscreen();
  }
});
document.addEventListener('keydown', function(event){
  if(event.key === 'Escape' && document.getElementById('mapFullscreen').classList.contains('show')){
    closeMapFullscreen();
  }
});
function closeCal(){ document.getElementById('overlay').classList.remove('show'); }
function bgClose(e){ if(e.target.id === 'overlay') closeCal(); }
function save(id){
  var value = document.getElementById('s' + id).value;
  if(!Number.isInteger(Number(value)) || Number(value) < -30 || Number(value) > 30){
    showNavigationNotice('Servo offset must be a whole number from -30 to 30', true);
    return;
  }
  fetch('/save?key=' + id + '&value=' + encodeURIComponent(value), { cache: 'no-store' })
    .then(function(response){
      if(!response.ok) throw new Error('Servo offset could not be saved');
      showNavigationNotice('Servo ' + id + ' offset saved');
    })
    .catch(function(error){ showNavigationNotice(error.message, true); });
}
function hideDebug(){ window.location = '/debug/close'; }
function updateVoiceButton(){
  var button = document.getElementById('voiceToggle');
  if(!window.speechSynthesis || typeof SpeechSynthesisUtterance === 'undefined'){
    button.textContent = 'Voice unavailable';
    button.disabled = true;
    button.setAttribute('aria-pressed', 'true');
    return;
  }
  button.textContent = navigationVoiceMuted ? 'Unmute voice' : 'Mute voice';
  button.setAttribute('aria-pressed', navigationVoiceMuted ? 'true' : 'false');
}
function toggleNavigationVoice(){
  navigationVoiceMuted = !navigationVoiceMuted;
  try {
    localStorage.setItem('navigationVoiceMuted', navigationVoiceMuted ? 'true' : 'false');
  } catch (error) {}
  updateVoiceButton();
  if(navigationVoiceMuted){
    if(window.speechSynthesis) window.speechSynthesis.cancel();
  } else if(window.speechSynthesis){
    speakNavigationMessage('Voice updates are on. I will keep you informed when my navigation state changes.');
  }
}
function speakNavigationMessage(message){
  if(navigationVoiceMuted || !window.speechSynthesis ||
      typeof SpeechSynthesisUtterance === 'undefined') return;
  window.speechSynthesis.cancel();
  var utterance = new SpeechSynthesisUtterance(message);
  utterance.lang = 'en-US';
  utterance.rate = 1.02;
  utterance.pitch = 1.0;
  window.speechSynthesis.speak(utterance);
}
function describeNavigationChange(nav){
  var reason = String(nav.reason || '').toLowerCase();
  switch(nav.state){
    case 'INITIALIZING':
      return 'I am powering up and checking my systems. ' + (nav.reason || '');
    case 'STARTUP_DELAY':
      return 'Systems are ready. I am waiting briefly before starting autonomous patrol.';
    case 'INITIAL_SCANNING':
      return 'I am turning to scan my surroundings and build a map before moving.';
    case 'EXPLORING':
      if(reason.indexOf('resum') >= 0) return 'Resuming autonomous coverage. I will continue exploring from here.';
      if(reason.indexOf('scan complete') >= 0) return 'The scan is complete. I found my bearings and am starting coverage.';
      if(reason.indexOf('lane change complete') >= 0) return 'I have cleared the blockage and am continuing my sweep.';
      return 'Path looks clear. I am continuing my autonomous coverage.';
    case 'AVOIDING_OBSTACLE':
      return 'Obstacle ahead at ' + (nav.distance_cm > 0 ? Math.round(nav.distance_cm) + ' centimeters. ' : '') +
        'I am looking for a safer direction.';
    case 'REPOSITIONING':
      if(reason.indexOf('coverage lane complete') >= 0)
        return 'Coverage lane complete. I am shifting over to start the next lane.';
      return 'I am repositioning to continue navigation.';
    case 'RECOVERING':
      return 'I am recovering from a blocked or uncertain path. Attempt ' +
        nav.recovery_attempts + '.';
    case 'TARGETING':
      return reason.indexOf('home') >= 0
        ? 'Returning to my home position.'
        : 'I am navigating to the requested map point.';
    case 'PAUSED':
      return reason.indexOf('destination reached') >= 0
        ? 'Destination reached. I am paused and waiting for your next instruction.'
        : 'Navigation is paused. I am holding position and waiting for you.';
    case 'STOPPED':
      return 'I have stopped because recovery did not find a safe path. Please check my surroundings.';
    case 'ERROR':
      return 'I have encountered a navigation problem. ' +
        (nav.reason || 'Please check my sensors and surroundings.');
    default:
      return 'Navigation state changed to ' + String(nav.state).toLowerCase() + '.';
  }
}
function announceNavigationState(nav){
  var updateKey = String(nav.state) + '\n' + String(nav.reason || '');
  if(updateKey === lastSpokenNavigationUpdate) return;
  lastSpokenNavigationUpdate = updateKey;
  speakNavigationMessage(describeNavigationChange(nav));
}
function drawNavMap(grid, heading, mapOffsetX, mapOffsetY){
  var canvas = document.getElementById('navMap');
  var ctx = canvas.getContext('2d');
  var width = 31;
  if(typeof grid !== 'string' || grid.length !== width * width) return;
  var cellSize = canvas.width / width;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  for(var y = 0; y < width; y++){
    for(var x = 0; x < width; x++){
      var cell = grid.charAt(y * width + x);
      var left = x * cellSize;
      var top = y * cellSize;
      ctx.fillStyle = cell === '#' ? '#111827' :
        cell === '+' ? '#f8fafc' :
        cell === 'R' ? '#4f46e5' :
        cell === '.' ? '#62b6be' : '#24313f';
      ctx.fillRect(left, top, cellSize, cellSize);
      if(cell === '?'){
        ctx.fillStyle = '#526174';
        ctx.fillRect(left + cellSize * 0.44, top + cellSize * 0.44, 2, 2);
      }
    }
  }
  ctx.strokeStyle = 'rgba(148,163,184,.28)';
  ctx.lineWidth = 1;
  for(var line = 0; line <= width; line++){
    var p = line * cellSize;
    ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, canvas.height); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(canvas.width, p); ctx.stroke();
  }
  var homeCellX = -mapOffsetX;
  var homeCellY = -mapOffsetY;
  if(homeCellX >= 0 && homeCellX < width && homeCellY >= 0 && homeCellY < width){
    var homeX = (homeCellX + .5) * cellSize;
    var homeY = (homeCellY + .5) * cellSize;
    ctx.save();
    ctx.fillStyle = 'rgba(15,23,42,.9)';
    ctx.beginPath(); ctx.arc(homeX, homeY, cellSize * .85, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = Math.max(2, cellSize * .12);
    ctx.beginPath(); ctx.arc(homeX, homeY, cellSize * .7, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#22c55e';
    ctx.font = 'bold ' + Math.max(9, cellSize * .75) + 'px Arial';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('H', homeX, homeY);
    ctx.restore();
  }
  if(plannedRoute.length){
    ctx.save();
    ctx.strokeStyle = '#f97316';
    ctx.fillStyle = '#fb923c';
    ctx.lineWidth = Math.max(2, cellSize * .18);
    ctx.setLineDash([cellSize * .45, cellSize * .25]);
    ctx.beginPath();
    for(var routeIndex = 0; routeIndex < plannedRoute.length; routeIndex++){
      var routeX = (plannedRoute[routeIndex].x / 10 - mapOffsetX + .5) * cellSize;
      var routeY = (-plannedRoute[routeIndex].y / 10 - mapOffsetY + .5) * cellSize;
      if(routeIndex === 0) ctx.moveTo(routeX, routeY);
      else ctx.lineTo(routeX, routeY);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    plannedRoute.forEach(function(point, index){
      var px = (point.x / 10 - mapOffsetX + .5) * cellSize;
      var py = (-point.y / 10 - mapOffsetY + .5) * cellSize;
      ctx.beginPath(); ctx.arc(px, py, cellSize * .3, 0, Math.PI * 2); ctx.fill();
      if(point.animationId){
        ctx.beginPath(); ctx.arc(px, py, cellSize * .43, 0, Math.PI * 2);
        ctx.strokeStyle = '#fde047'; ctx.lineWidth = 2; ctx.stroke();
      }
      ctx.fillStyle = '#fff';
      ctx.font = 'bold ' + Math.max(9, cellSize * .55) + 'px Arial';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(index + 1), px, py);
      ctx.fillStyle = '#fb923c';
    });
    ctx.restore();
  }
  var robotIndex = grid.indexOf('R');
  if(robotIndex >= 0){
    var robotX = (robotIndex % width + 0.5) * cellSize;
    var robotY = (Math.floor(robotIndex / width) + 0.5) * cellSize;
    var facing = (heading - 90) * Math.PI / 180;
    var markerLength = cellSize * 1.5;
    var markerWidth = cellSize * 1.05;
    ctx.save();
    ctx.translate(robotX, robotY);
    ctx.rotate(facing);
    ctx.fillStyle = 'rgba(15,23,42,.9)';
    ctx.beginPath(); ctx.arc(0, 0, cellSize * 1.1, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#facc15';
    ctx.beginPath();
    ctx.moveTo(markerLength, 0);
    ctx.lineTo(-markerLength * 0.65, -markerWidth);
    ctx.lineTo(-markerLength * 0.4, 0);
    ctx.lineTo(-markerLength * 0.65, markerWidth);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }
}
function updateNavigation(){
  fetch('/api/v1/navigation', { cache: 'no-store' })
    .then(function(response){ if(!response.ok) throw new Error('status'); return response.json(); })
    .then(function(response){
      if(!response.ok || !response.data) return;
      var nav = response.data;
      document.getElementById('navState').textContent = nav.state;
      announceNavigationState(nav);
      document.getElementById('navRange').textContent = nav.sensor_valid ? nav.distance_cm + ' cm' : 'unavailable';
      document.getElementById('navTravel').textContent = nav.distance_travelled_cm;
      document.getElementById('navArea').textContent = Number(nav.area_covered_m2).toFixed(2);
      document.getElementById('navHeading').textContent = nav.heading_deg;
      document.getElementById('navRecovery').textContent = nav.recovery_attempts;
      latestNavigation = nav;
      drawNavMap(nav.grid, nav.heading_deg, nav.map_offset_x, nav.map_offset_y);
    })
    .catch(function(){ showNavigationNotice('Telemetry unavailable; check the Wi-Fi connection', true); });
}

function mapPointFromEvent(event){
  if(!latestNavigation) return null;
  var canvas = document.getElementById('navMap');
  var rect = canvas.getBoundingClientRect();
  var x = Math.floor((event.clientX - rect.left) * 31 / rect.width);
  var y = Math.floor((event.clientY - rect.top) * 31 / rect.height);
  if(x < 0 || x >= 31 || y < 0 || y >= 31) return null;
  return {
    x: (x + latestNavigation.map_offset_x) * latestNavigation.cell_size_cm,
    y: -(y + latestNavigation.map_offset_y) * latestNavigation.cell_size_cm
  };
}
function hideMapContextMenu(){
  document.getElementById('mapContextMenu').classList.remove('show');
}
function showMapContextMenu(event){
  var point = mapPointFromEvent(event);
  if(!point) return;
  selectedMapPoint = point;
  var menu = document.getElementById('mapContextMenu');
  var routeMode = drawingRoute;
  document.getElementById('mapGoButton').style.display = routeMode ? 'none' : '';
  document.getElementById('mapRouteButton').style.display = '';
  document.getElementById('mapRouteButton').textContent = routeMode ? 'Finish route' : 'Draw route from here';
  document.getElementById('mapAddRoutePointButton').style.display = routeMode ? '' : 'none';
  var animationOptions = document.getElementById('mapAnimationOptions');
  animationOptions.replaceChildren();
  mapAnimations.forEach(function(animation){
    var button = document.createElement('button');
    button.type = 'button';
    button.textContent = routeMode
      ? 'Add point + ' + animation.name
      : 'Go here + ' + animation.name;
    button.onclick = function(){
      mapMenuAction(routeMode ? 'add-route-animation' : 'goto-animation', animation.id);
    };
    animationOptions.appendChild(button);
  });
  document.getElementById('mapPauseButton').textContent =
    latestNavigation && ['PAUSED','STOPPED','ERROR'].indexOf(latestNavigation.state) >= 0
      ? 'Resume' : 'Pause';
  document.getElementById('mapCancelRouteButton').style.display = routeMode ? '' : 'none';
  menu.classList.add('show');
  var menuRect = menu.getBoundingClientRect();
  menu.style.left = Math.max(8, Math.min(event.clientX,
    window.innerWidth - menuRect.width - 8)) + 'px';
  menu.style.top = Math.max(8, Math.min(event.clientY,
    window.innerHeight - menuRect.height - 8)) + 'px';
}
function queueMapTarget(command, points){
  var body = new URLSearchParams();
  body.set('command', command);
  if(points && points.length){
    body.set('points', points.map(function(point){
      return point.x + ',' + point.y + ',' + (point.animationId || 0);
    }).join(';'));
  }
  return fetch('/api/v1/navigation/target', {
    method: 'POST', body: body, cache: 'no-store',
    headers: {'Content-Type':'application/x-www-form-urlencoded'}
  }).then(function(response){ return response.json().then(function(json){
    if(!response.ok || !json.ok) throw new Error(json.message || 'Navigation target rejected');
    return json;
  }); });
}
function mapMenuAction(action, animationId){
  hideMapContextMenu();
  if(action === 'route'){
    if(!drawingRoute){
      drawingRoute = true;
      plannedRoute = [];
      if(selectedMapPoint) plannedRoute.push({
        x:selectedMapPoint.x, y:selectedMapPoint.y, animationId:0
      });
      showNavigationNotice('Select route points or add them from the map menu, then finish route');
    } else {
      if(!plannedRoute.length){ drawingRoute = false; return; }
      queueMapTarget('route', plannedRoute).then(function(response){
        showNavigationNotice(response.message || 'Route queued');
        drawingRoute = false; updateNavigation();
      }).catch(function(error){ showNavigationNotice(error.message, true); });
    }
  } else if(action === 'add-route-point' || action === 'add-route-animation'){
    if(!drawingRoute || !selectedMapPoint) return;
    if(plannedRoute.length >= 12){
      showNavigationNotice('Route limit is 12 points', true);
      return;
    }
    plannedRoute.push({
      x:selectedMapPoint.x, y:selectedMapPoint.y,
      animationId: action === 'add-route-animation' ? animationId : 0
    });
    showNavigationNotice('Route point ' + plannedRoute.length + ' added');
    if(latestNavigation) drawNavMap(latestNavigation.grid, latestNavigation.heading_deg,
      latestNavigation.map_offset_x, latestNavigation.map_offset_y);
  } else if(action === 'cancel'){
    drawingRoute = false; plannedRoute = []; updateNavigation();
  } else if(action === 'goto' && selectedMapPoint){
    queueMapTarget('goto', [{x:selectedMapPoint.x,y:selectedMapPoint.y,animationId:0}]).then(function(response){
      showNavigationNotice(response.message || 'Destination queued');
      plannedRoute = [selectedMapPoint]; updateNavigation();
    }).catch(function(error){ showNavigationNotice(error.message, true); });
  } else if(action === 'goto-animation' && selectedMapPoint){
    queueMapTarget('goto', [{
      x:selectedMapPoint.x, y:selectedMapPoint.y, animationId:animationId
    }]).then(function(response){
      showNavigationNotice(response.message || 'Destination queued');
      plannedRoute = [{x:selectedMapPoint.x,y:selectedMapPoint.y,animationId:animationId}];
      updateNavigation();
    }).catch(function(error){ showNavigationNotice(error.message, true); });
  } else if(action === 'home'){
    queueMapTarget('home', []).then(function(response){
      plannedRoute = [{x:0,y:0}]; showNavigationNotice(response.message || 'Returning home');
      updateNavigation();
    }).catch(function(error){ showNavigationNotice(error.message, true); });
  } else if(action === 'pause'){
    var command = latestNavigation &&
      ['PAUSED','STOPPED','ERROR'].indexOf(latestNavigation.state) >= 0
      ? 'resume' : 'pause';
    controlNavigation(command);
  }
}
var navigationMapCanvas = document.getElementById('navMap');
navigationMapCanvas.addEventListener('contextmenu', function(event){
  event.preventDefault(); showMapContextMenu(event);
});
navigationMapCanvas.addEventListener('pointerdown', function(event){
  if(event.pointerType !== 'touch') return;
  window.clearTimeout(longPressTimer);
  longPressTimer = window.setTimeout(function(){
    ignoreNextMapClick = true;
    showMapContextMenu(event);
  }, 550);
});
['pointerup','pointercancel','pointerleave'].forEach(function(name){
  navigationMapCanvas.addEventListener(name, function(){ window.clearTimeout(longPressTimer); });
});
navigationMapCanvas.addEventListener('click', function(event){
  if(ignoreNextMapClick){ ignoreNextMapClick = false; return; }
  if(!drawingRoute) return;
  var point = mapPointFromEvent(event);
  if(!point) return;
  if(plannedRoute.length >= 12){
    showNavigationNotice('Route limit is 12 points', true);
    return;
  }
  point.animationId = 0;
  plannedRoute.push(point);
  drawNavMap(latestNavigation.grid, latestNavigation.heading_deg,
    latestNavigation.map_offset_x, latestNavigation.map_offset_y);
});
document.addEventListener('pointerdown', function(event){
  if(!event.target.closest || !event.target.closest('#mapContextMenu')) hideMapContextMenu();
});
updateNavigation();
window.setInterval(updateNavigation, 500);
updateVoiceButton();
function openCal(){
  Promise.all([
    fetch('/setting1', { cache: 'no-store' }).then(function(r){ return r.json(); }),
    fetch('/api/v1/calibration/steering', { cache: 'no-store' })
      .then(function(r){
        if(!r.ok) throw new Error('Could not load steering calibration');
        return r.json();
      }),
    fetch('/api/v1/calibration/rotation', { cache: 'no-store' })
      .then(function(r){
        if(!r.ok) throw new Error('Could not load turn calibration');
        return r.json();
      })
  ]).then(function(results){
      var values = results[0];
      var steering = results[1];
      var rotation = results[2];
      if(!steering.ok || !steering.data ||
          typeof steering.data.degrees_per_meter !== 'number'){
        throw new Error(steering.message || 'Invalid steering calibration response');
      }
      if(!rotation.ok || !rotation.data ||
          typeof rotation.data.observed_360_degrees !== 'number' ||
          typeof rotation.data.calibration_equivalent_degrees !== 'number'){
        throw new Error(rotation.message || 'Invalid turn calibration response');
      }
      for (var i = 0; i < 8; i++) {
        document.getElementById('s' + i).value = values[i] || 0;
      }
      document.getElementById('steeringCorrection').value =
        steering.data.degrees_per_meter;
      document.getElementById('rotation360Observed').value =
        rotation.data.observed_360_degrees;
      document.getElementById('rotationCalibrationCurrent').textContent =
        'Current effective calibration: ' +
        rotation.data.calibration_equivalent_degrees +
        '° per commanded 360°';
      document.getElementById('overlay').classList.add('show');
    }).catch(function(error){
      showNavigationNotice(error.message, true);
    });
}
function saveSteeringCorrection(){
  var value = Number(document.getElementById('steeringCorrection').value);
  if(!Number.isInteger(value) || value < -90 || value > 90){
    showNavigationNotice(
      'Steering correction must be a whole number from -90 to 90 degrees per meter', true);
    return;
  }
  fetch('/api/v1/calibration/steering', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({degrees_per_meter:value})
  }).then(function(response){ return response.json(); })
    .then(function(response){
      if(!response.ok) throw new Error(response.message || 'Could not save steering calibration');
      showNavigationNotice(
        'Forward steering correction saved: ' +
        response.data.degrees_per_meter + ' deg/m');
    }).catch(function(error){
      showNavigationNotice(error.message, true);
    });
}
function saveRotationCalibration(){
  var observed360 = Number(document.getElementById('rotation360Observed').value);
  if(!Number.isInteger(observed360) || observed360 < 300 || observed360 > 420){
    showNavigationNotice('Measured 360° turn must be a whole number from 300° to 420°', true);
    return;
  }
  fetch('/api/v1/calibration/rotation', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({observed_360_degrees:observed360})
  }).then(function(response){ return response.json(); })
    .then(function(response){
      if(!response.ok) throw new Error(response.message || 'Could not save turn calibration');
      if(!response.data ||
          typeof response.data.observed_360_degrees !== 'number' ||
          typeof response.data.calibration_equivalent_degrees !== 'number'){
        throw new Error('Invalid turn calibration response');
      }
      document.getElementById('rotation360Observed').value =
        response.data.observed_360_degrees;
      document.getElementById('rotationCalibrationCurrent').textContent =
        'Current effective calibration: ' +
        response.data.calibration_equivalent_degrees +
        '° per commanded 360°';
      showNavigationNotice(
        'Turn calibration refined from ' + response.data.observed_360_degrees +
        '° measured; effective calibration is now ' +
        response.data.calibration_equivalent_degrees + '°');
    }).catch(function(error){
      showNavigationNotice(error.message, true);
    });
}
</script>
</body>
</html>
)rawliteral";

static const char DEBUG_LOCKED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QuadBot Debug Locked</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#0f172a;color:#e5e7eb;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.card{width:min(420px,100%);background:#111827;border:1px solid #243041;border-radius:16px;padding:22px}
h1{margin:0 0 12px;font-size:1.15rem;color:#93c5fd}
p{margin:0 0 10px;line-height:1.5;color:#cbd5e1}
code{background:#0b1220;padding:2px 6px;border-radius:6px}
</style>
</head>
<body>
  <div class="card">
    <h1>Debug UI Disabled</h1>
    <p>The legacy debug web page is currently hidden.</p>
    <p>Open <code>/debug/open</code> when you want to use it, or keep using the <code>/api/v1</code> endpoints.</p>
  </div>
</body>
</html>
)rawliteral";

// ── EEPROM helpers ────────────────────────────────────────────────────────────

void writeKeyValue(int8_t key, int8_t value)
{
    EEPROM.write(key, value);
    EEPROM.commit();
}

// BUG FIX: was always returning 0 instead of the stored value
int8_t readKeyValue(int8_t key)
{
    return (int8_t)EEPROM.read(key);
}

static const int kForwardSteeringValueAddress = 8;
static const int kForwardSteeringMarkerAddress = 9;
static const uint8_t kForwardSteeringMarker = 0xA7;
static const int8_t kDefaultForwardSteeringCorrection = 30;
static const int kRotation360ValueAddress = 12;
static const int kRotationCalibrationMarkerAddress = 14;
static const uint8_t kRotationCalibrationMarker = 0xD3;
static const int kRotationObservedValueAddress = 10;
static const int kRotationObservedMarkerAddress = 15;
static const uint8_t kRotationObservedMarker = 0xD4;
static const uint16_t kDefaultObserved360Degrees = 360;

int8_t getForwardSteeringCorrection()
{
    if (EEPROM.read(kForwardSteeringMarkerAddress) != kForwardSteeringMarker) {
        return kDefaultForwardSteeringCorrection;
    }
    const int8_t correction =
        (int8_t)EEPROM.read(kForwardSteeringValueAddress);
    return correction >= -90 && correction <= 90
        ? correction : kDefaultForwardSteeringCorrection;
}

bool setForwardSteeringCorrection(int degreesPerMeter)
{
    if (degreesPerMeter < -90 || degreesPerMeter > 90) {
        return false;
    }
    EEPROM.write(kForwardSteeringValueAddress, (uint8_t)(int8_t)degreesPerMeter);
    EEPROM.write(kForwardSteeringMarkerAddress, kForwardSteeringMarker);
    return EEPROM.commit();
}

static uint16_t readEepromUint16(int address)
{
    return (uint16_t)EEPROM.read(address) |
        ((uint16_t)EEPROM.read(address + 1) << 8);
}

static void writeEepromUint16(int address, uint16_t value)
{
    EEPROM.write(address, (uint8_t)(value & 0xFF));
    EEPROM.write(address + 1, (uint8_t)(value >> 8));
}

uint16_t getObserved360RotationDegrees()
{
    if (EEPROM.read(kRotationObservedMarkerAddress) !=
        kRotationObservedMarker) {
        return kDefaultObserved360Degrees;
    }
    const uint16_t value = readEepromUint16(kRotationObservedValueAddress);
    return value >= 300 && value <= 420
        ? value : kDefaultObserved360Degrees;
}

uint16_t getRotationCalibrationEquivalentDegrees()
{
    if (EEPROM.read(kRotationCalibrationMarkerAddress) !=
        kRotationCalibrationMarker) {
        return kDefaultObserved360Degrees;
    }
    const uint16_t value = readEepromUint16(kRotation360ValueAddress);
    return value >= 250 && value <= 490
        ? value : kDefaultObserved360Degrees;
}

float getRotationCalibrationScale()
{
    return (float)getRotationCalibrationEquivalentDegrees() / 360.0f;
}

bool setObserved360RotationDegrees(uint16_t observed360)
{
    if (observed360 < 300 || observed360 > 420) {
        return false;
    }
    const uint16_t currentEquivalent =
        getRotationCalibrationEquivalentDegrees();
    const uint16_t updatedEquivalent = (uint16_t)lroundf(
        (float)currentEquivalent * observed360 / 360.0f);
    if (updatedEquivalent < 250 || updatedEquivalent > 490) {
        return false;
    }
    writeEepromUint16(kRotation360ValueAddress, updatedEquivalent);
    EEPROM.write(kRotationCalibrationMarkerAddress,
        kRotationCalibrationMarker);
    writeEepromUint16(kRotationObservedValueAddress, observed360);
    EEPROM.write(kRotationObservedMarkerAddress, kRotationObservedMarker);
    return EEPROM.commit();
}

const RobotActionInfo* getRobotActions(size_t& count)
{
    count = sizeof(kRobotActions) / sizeof(kRobotActions[0]);
    return kRobotActions;
}

const RobotActionInfo* findRobotActionById(int actionId)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actions[i].id == actionId) {
            return &actions[i];
        }
    }

    return nullptr;
}

const RobotActionInfo* findRobotActionByName(const String& actionName)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actionName.equalsIgnoreCase(actions[i].name)) {
            return &actions[i];
        }
    }

    return nullptr;
}

const char* getActionName(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->name : "idle";
}

const char* getActionLabel(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->label : "空闲";
}

String getDeviceName()
{
    return g_deviceName;
}

void setCurrentAction(int actionId, bool busy)
{
    g_currentActionId = actionId;
    g_robotBusy = busy;
}

void clearCurrentAction()
{
    g_currentActionId = PROGRAM_NONE;
    g_robotBusy = false;
}

int getCurrentActionId()
{
    return g_currentActionId;
}

const char* getCurrentActionName()
{
    return getActionName(g_currentActionId);
}

bool isRobotBusy()
{
    return g_robotBusy;
}

void requestRobotStop(int targetProgram)
{
    g_stopRequested = true;
    g_stopTargetProgram = targetProgram;
}

void clearRobotStopRequest()
{
    g_stopRequested = false;
    g_stopTargetProgram = PROGRAM_STANDBY;
}

bool isRobotStopRequested()
{
    return g_stopRequested;
}

int getRobotStopTargetProgram()
{
    return g_stopTargetProgram;
}

bool queueNavigationControl(NavigationControl control)
{
    if (control == NAV_CONTROL_NONE || Servo_PROGRAM >= 1 ||
        g_navigationControl != NAV_CONTROL_NONE ||
        g_navigationTargetKind != NAV_TARGET_NONE) {
        return false;
    }
    g_navigationControl = control;
    return true;
}

NavigationControl takeNavigationControl()
{
    const NavigationControl control = g_navigationControl;
    g_navigationControl = NAV_CONTROL_NONE;
    return control;
}

bool hasNavigationControlPending()
{
    return g_navigationControl != NAV_CONTROL_NONE;
}

bool queueNavigationTarget(NavigationTargetKind kind,
    const NavigationWaypoint* waypoints, uint8_t count)
{
    if (kind == NAV_TARGET_NONE || Servo_PROGRAM >= 1 ||
        g_navigationTargetKind != NAV_TARGET_NONE ||
        g_navigationControl != NAV_CONTROL_NONE ||
        (kind == NAV_TARGET_POINT && count != 1) ||
        (kind == NAV_TARGET_ROUTE && (count == 0 || count > MAX_NAVIGATION_WAYPOINTS)) ||
        (kind == NAV_TARGET_HOME && count != 0) ||
        (count > 0 && waypoints == nullptr)) {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (waypoints[i].xCm != waypoints[i].xCm ||
            waypoints[i].yCm != waypoints[i].yCm ||
            fabsf(waypoints[i].xCm) > 5000.0f ||
            fabsf(waypoints[i].yCm) > 5000.0f) {
            return false;
        }
        const int16_t animationId = waypoints[i].animationId;
        if (animationId != PROGRAM_NONE && animationId != PROGRAM_HELLO &&
            animationId != PROGRAM_PUSHUP && animationId != PROGRAM_LIE &&
            animationId != PROGRAM_FIGHTING && animationId != PROGRAM_SLEEP &&
            animationId != PROGRAM_DANCE1 && animationId != PROGRAM_DANCE2 &&
            animationId != PROGRAM_DANCE3) {
            return false;
        }
        if (kind != NAV_TARGET_HOME) {
            const float localX = waypoints[i].xCm / 10.0f -
                g_navigationTelemetry.mapOffsetX;
            const float localY = -waypoints[i].yCm / 10.0f -
                g_navigationTelemetry.mapOffsetY;
            if (localX < 0.0f || localX >= 31.0f ||
                localY < 0.0f || localY >= 31.0f) {
                return false;
            }
            const int cellX = (int)lroundf(localX);
            const int cellY = (int)lroundf(localY);
            if (cellX >= 31 || cellY >= 31 ||
                g_navigationTelemetry.occupancyGrid[cellY * 31 + cellX] == 2) {
                return false;
            }
        }
        g_navigationWaypoints[i] = waypoints[i];
    }
    g_navigationWaypointCount = count;
    g_navigationTargetKind = kind;
    return true;
}

NavigationTargetKind takeNavigationTarget(NavigationWaypoint* waypoints,
    uint8_t& count)
{
    const NavigationTargetKind kind = g_navigationTargetKind;
    count = g_navigationWaypointCount;
    if (kind != NAV_TARGET_NONE && waypoints != nullptr) {
        for (uint8_t i = 0; i < count; ++i) {
            waypoints[i] = g_navigationWaypoints[i];
        }
    }
    g_navigationWaypointCount = 0;
    g_navigationTargetKind = NAV_TARGET_NONE;
    return kind;
}

bool hasNavigationTargetPending()
{
    return g_navigationTargetKind != NAV_TARGET_NONE;
}

void publishNavigationTelemetry(const char* state, const char* reason,
    float distanceCm, float distanceTravelledCm, float areaCoveredM2,
    float headingDegrees,
    int mapOffsetX, int mapOffsetY, int recoveryAttempts, bool sensorValid,
    const uint16_t scanDistancesCm[12],
    const uint8_t occupancyGrid[31 * 31])
{
    strncpy(g_navigationTelemetry.state, state, sizeof(g_navigationTelemetry.state) - 1);
    g_navigationTelemetry.state[sizeof(g_navigationTelemetry.state) - 1] = '\0';
    strncpy(g_navigationTelemetry.reason, reason,
        sizeof(g_navigationTelemetry.reason) - 1);
    g_navigationTelemetry.reason[sizeof(g_navigationTelemetry.reason) - 1] = '\0';
    g_navigationTelemetry.distanceCm = distanceCm;
    g_navigationTelemetry.distanceTravelledCm = distanceTravelledCm;
    g_navigationTelemetry.areaCoveredM2 = areaCoveredM2;
    g_navigationTelemetry.headingDegrees = headingDegrees;
    g_navigationTelemetry.mapOffsetX = mapOffsetX;
    g_navigationTelemetry.mapOffsetY = mapOffsetY;
    g_navigationTelemetry.recoveryAttempts = recoveryAttempts;
    g_navigationTelemetry.sensorValid = sensorValid;
    memcpy(g_navigationTelemetry.scanDistancesCm, scanDistancesCm,
        sizeof(g_navigationTelemetry.scanDistancesCm));
    memcpy(g_navigationTelemetry.occupancyGrid, occupancyGrid,
        sizeof(g_navigationTelemetry.occupancyGrid));
}

static bool ensureDebugUiEnabled()
{
    if (g_debugUiEnabled) {
        return true;
    }

    server.send(403, "text/plain", "DEBUG_UI_DISABLED");
    return false;
}

static void sendJsonResponse(int statusCode, bool ok, int code, const char* message, const String& data)
{
    String payload = "{\"ok\":";
    payload.reserve(data.length() + 96);
    payload += ok ? "true" : "false";
    payload += ",\"code\":";
    payload += String(code);
    payload += ",\"message\":\"";
    payload += message;
    payload += "\"";
    if (data != "") {
        payload += ",\"data\":";
        payload += data;
    }
    payload += "}";

    server.send(statusCode, "application/json", payload);
}

static void sendJsonError(int statusCode, int code, const char* message)
{
    sendJsonResponse(statusCode, false, code, message, "");
}

static String buildCalibrationJsonArray()
{
    String offsets = "[";
    for (int i = 0; i < ALLSERVOS; i++) {
        if (i > 0) {
            offsets += ",";
        }
        offsets += String((int8_t)EEPROM.read(i));
    }
    offsets += "]";
    return offsets;
}

static String buildActionsJsonArray()
{
    String actions = "[";
    size_t count = 0;
    const RobotActionInfo* actionList = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (i > 0) {
            actions += ",";
        }

        actions += "{\"id\":";
        actions += String(actionList[i].id);
        actions += ",\"name\":\"";
        actions += actionList[i].name;
        actions += "\",\"label\":\"";
        actions += actionList[i].label;
        actions += "\"}";
    }

    actions += "]";
    return actions;
}

static bool extractJsonInt(const String& body, const char* key, int& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int start = colonPos + 1;
    while (start < body.length() && isspace(body[start])) {
        start++;
    }

    int end = start;
    if (end < body.length() && body[end] == '-') {
        end++;
    }

    while (end < body.length() && isdigit(body[end])) {
        end++;
    }

    if (start == end) {
        return false;
    }

    value = body.substring(start, end).toInt();
    return true;
}

static bool extractJsonString(const String& body, const char* key, String& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int firstQuote = body.indexOf('"', colonPos + 1);
    if (firstQuote < 0) {
        return false;
    }

    int secondQuote = body.indexOf('"', firstQuote + 1);
    if (secondQuote < 0) {
        return false;
    }

    value = body.substring(firstQuote + 1, secondQuote);
    return true;
}

static bool resolveActionRequest(int& actionId)
{
    actionId = PROGRAM_NONE;
    String body = server.arg("plain");
    int parsedId = PROGRAM_NONE;
    String actionName;

    if (body != "") {
        if (extractJsonInt(body, "id", parsedId)) {
            actionId = parsedId;
        } else if (extractJsonString(body, "action", actionName)) {
            const RobotActionInfo* action = findRobotActionByName(actionName);
            if (action != nullptr) {
                actionId = action->id;
            }
        }
    }

    if (actionId == PROGRAM_NONE && server.hasArg("id")) {
        actionId = server.arg("id").toInt();
    }

    if (actionId == PROGRAM_NONE && server.hasArg("action")) {
        const RobotActionInfo* action = findRobotActionByName(server.arg("action"));
        if (action != nullptr) {
            actionId = action->id;
        }
    }

    return findRobotActionById(actionId) != nullptr;
}

static bool parseCalibrationOffsets(int8_t offsets[ALLSERVOS])
{
    String body = server.arg("plain");
    if (body == "") {
        return false;
    }

    int keyPos = body.indexOf("\"offsets\"");
    if (keyPos < 0) {
        return false;
    }

    int startBracket = body.indexOf('[', keyPos);
    int endBracket = body.indexOf(']', startBracket);
    if (startBracket < 0 || endBracket < 0 || endBracket <= startBracket) {
        return false;
    }

    String values = body.substring(startBracket + 1, endBracket);
    values.trim();

    for (int i = 0; i < ALLSERVOS; i++) {
        int commaPos = values.indexOf(',');
        String item;

        if (commaPos >= 0) {
            item = values.substring(0, commaPos);
            values = values.substring(commaPos + 1);
        } else {
            item = values;
            values = "";
        }

        item.trim();
        if (item == "") {
            return false;
        }

        int value = item.toInt();
        if (value < -124 || value > 124) {
            return false;
        }

        offsets[i] = (int8_t)value;
    }

    return values.indexOf(',') < 0 && values == "";
}

static bool parseServoCommand(int& servoId, int& servoValue)
{
    servoId = -1;
    servoValue = 0;

    String body = server.arg("plain");
    if (body != "") {
        extractJsonInt(body, "servo", servoId);
        extractJsonInt(body, "value", servoValue);
    }

    if (servoId < 0 && server.hasArg("servo")) {
        servoId = server.arg("servo").toInt();
    }

    if (!server.hasArg("value") && body == "") {
        return false;
    }

    if (body == "" && server.hasArg("value")) {
        servoValue = server.arg("value").toInt();
    }

    if (servoId < 0 || servoId >= ALLSERVOS) {
        return false;
    }

    if (servoValue < PWMRES_Min || servoValue > PWMRES_Max) {
        return false;
    }

    return true;
}

static bool resolveStopMode(int& targetProgram)
{
    targetProgram = PROGRAM_STANDBY;
    String mode;
    String body = server.arg("plain");

    if (body != "") {
        extractJsonString(body, "mode", mode);
    }

    if (mode == "" && server.hasArg("mode")) {
        mode = server.arg("mode");
    }

    if (mode == "" || mode.equalsIgnoreCase("standby")) {
        targetProgram = PROGRAM_STANDBY;
        return true;
    }

    if (mode.equalsIgnoreCase("center")) {
        targetProgram = PROGRAM_CENTER;
        return true;
    }

    if (mode.equalsIgnoreCase("zero")) {
        targetProgram = PROGRAM_ZERO;
        return true;
    }

    return false;
}

// ── WiFi / EEPROM init ────────────────────────────────────────────────────────

bool webinit()
{
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    WiFi.softAPmacAddress(mac);
    String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
    macID.toUpperCase();

    String AP_NameString = "Robot - " + macID;
    g_deviceName = AP_NameString;
    char AP_NameChar[AP_NameString.length() + 1];
    memset(AP_NameChar, 0, AP_NameString.length() + 1);
    for (int i = 0; i < (int)AP_NameString.length(); i++)
        AP_NameChar[i] = AP_NameString.charAt(i);

    WiFi.mode(WIFI_AP);
    g_wifiApReady = WiFi.softAP(AP_NameChar, password);

    EEPROM.begin(512);
    delay(10);
    return g_wifiApReady;
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────

void handleClient()
{
    server.handleClient();
}

// Main page: serve PROGMEM HTML directly (no String allocation)
void handleIndex()
{
    if (!g_debugUiEnabled) {
        server.send_P(200, "text/html", DEBUG_LOCKED_HTML);
        return;
    }

    server.send_P(200, "text/html", INDEX_HTML);
}

// Returns calibration offsets as JSON array [s0, s1, ..., s7]
// Index i = servo i = EEPROM key i
void handleSetting1()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String content = "[";
    for (int i = 0; i < 8; i++) {
        if (i > 0) content += ",";
        content += String((int8_t)EEPROM.read(i));
    }
    content += "]";
    server.send(200, "application/json", content);
}

static void handleApiInfo()
{
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"model\":\"";
    data += kRobotDeviceInfo.model;
    data += "\",\"firmware_version\":\"";
    data += kRobotDeviceInfo.firmwareVersion;
    data += "\",\"protocol_version\":\"";
    data += kRobotDeviceInfo.protocolVersion;
    data += "\",\"wifi_mode\":\"AP\",";
    data += "\"servo_count\":";
    data += String(ALLSERVOS);
    data += ",\"actions\":";
    data += buildActionsJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiState()
{
    int requestedActionId = Servo_PROGRAM;
    bool avoidEnabled = (getCurrentActionId() == PROGRAM_AVOID) || (requestedActionId == PROGRAM_AVOID);
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"current_action_id\":";
    data += String(getCurrentActionId());
    data += ",\"current_action\":\"";
    data += getCurrentActionName();
    data += "\",\"current_action_label\":\"";
    data += getActionLabel(getCurrentActionId());
    data += "\",\"requested_action_id\":";
    data += String(requestedActionId);
    data += ",\"requested_action\":\"";
    data += getActionName(requestedActionId);
    data += "\",\"busy\":";
    data += isRobotBusy() ? "true" : "false";
    data += ",\"avoid_enabled\":";
    data += avoidEnabled ? "true" : "false";
    data += ",\"stop_requested\":";
    data += isRobotStopRequested() ? "true" : "false";
    data += ",\"stop_target\":\"";
    data += getActionName(getRobotStopTargetProgram());
    data += "\",\"debug_ui_enabled\":";
    data += g_debugUiEnabled ? "true" : "false";
    data += ",\"wifi_ap_ready\":";
    data += g_wifiApReady ? "true" : "false";
    data += ",\"wifi_mode\":\"AP\"}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiNavigation()
{
    String data;
    data.reserve(1600);
    data = "{";
    data += "\"state\":\"";
    data += g_navigationTelemetry.state;
    data += "\",\"reason\":\"";
    data += g_navigationTelemetry.reason;
    data += "\",\"sensor_valid\":";
    data += g_navigationTelemetry.sensorValid ? "true" : "false";
    data += ",\"distance_cm\":";
    data += String(g_navigationTelemetry.distanceCm, 1);
    data += ",\"distance_travelled_cm\":";
    data += String(g_navigationTelemetry.distanceTravelledCm, 1);
    data += ",\"area_covered_m2\":";
    data += String(g_navigationTelemetry.areaCoveredM2, 2);
    data += ",\"heading_deg\":";
    data += String(g_navigationTelemetry.headingDegrees, 2);
    data += ",\"map_offset_x\":";
    data += String(g_navigationTelemetry.mapOffsetX);
    data += ",\"map_offset_y\":";
    data += String(g_navigationTelemetry.mapOffsetY);
    data += ",\"recovery_attempts\":";
    data += String(g_navigationTelemetry.recoveryAttempts);
    data += ",\"scan_cm\":[";
    for (int i = 0; i < 12; ++i) {
        if (i > 0) {
            data += ",";
        }
        data += String(g_navigationTelemetry.scanDistancesCm[i]);
    }
    data += "],\"map_width\":31,\"map_height\":31,\"cell_size_cm\":10,\"grid\":\"";
    static const char kMapCellSymbols[] = {'?', '.', '#', '+', 'R'};
    for (size_t i = 0; i < sizeof(g_navigationTelemetry.occupancyGrid); ++i) {
        const uint8_t cell = g_navigationTelemetry.occupancyGrid[i];
        data += cell < sizeof(kMapCellSymbols) ? kMapCellSymbols[cell] : '?';
    }
    data += "\"}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiNavigationControl()
{
    if (!server.hasArg("command")) {
        sendJsonError(400, 4010, "missing navigation command");
        return;
    }

    const String command = server.arg("command");
    NavigationControl control = NAV_CONTROL_NONE;
    if (command == "resume") {
        control = NAV_CONTROL_RESUME;
    } else if (command == "pause") {
        control = NAV_CONTROL_PAUSE;
    } else if (command == "restart") {
        control = NAV_CONTROL_RESTART;
    } else {
        sendJsonError(400, 4011, "invalid navigation command");
        return;
    }

    if (!queueNavigationControl(control)) {
        sendJsonError(409, 4012,
            "navigation command unavailable during a manual action or another pending command");
        return;
    }

    sendJsonResponse(202, true, 0, "navigation command queued", "");
}

static bool parseNavigationPoints(const String& encoded, NavigationWaypoint* points,
    uint8_t& count)
{
    count = 0;
    const char* cursor = encoded.c_str();
    const char* const end = cursor + encoded.length();
    while (cursor < end) {
        if (count >= MAX_NAVIGATION_WAYPOINTS) {
            return false;
        }
        char* parsed = nullptr;
        const float x = strtof(cursor, &parsed);
        if (parsed == cursor || parsed >= end || *parsed != ',' ||
            x != x || fabsf(x) > 5000.0f) {
            return false;
        }
        cursor = parsed + 1;
        const float y = strtof(cursor, &parsed);
        if (parsed == cursor || y != y ||
            fabsf(x) > 5000.0f || fabsf(y) > 5000.0f) {
            return false;
        }
        int16_t animationId = PROGRAM_NONE;
        cursor = parsed;
        if (cursor < end && *cursor == ',') {
            cursor++;
            char* animationEnd = nullptr;
            const long parsedAnimation = strtol(cursor, &animationEnd, 10);
            if (animationEnd == cursor || parsedAnimation < -32768L ||
                parsedAnimation > 32767L) {
                return false;
            }
            animationId = (int16_t)parsedAnimation;
            cursor = animationEnd;
        }
        points[count].xCm = x;
        points[count].yCm = y;
        points[count].animationId = animationId;
        ++count;
        if (cursor == end) {
            break;
        }
        if (*cursor != ';') {
            return false;
        }
        ++cursor;
        if (cursor == end) {
            return false;
        }
    }
    return count > 0;
}

static void handleApiNavigationTarget()
{
    if (!server.hasArg("command")) {
        sendJsonError(400, 4020, "missing navigation target command");
        return;
    }

    const String command = server.arg("command");
    NavigationTargetKind kind = NAV_TARGET_NONE;
    NavigationWaypoint points[MAX_NAVIGATION_WAYPOINTS];
    uint8_t count = 0;
    if (command == "goto") {
        kind = NAV_TARGET_POINT;
    } else if (command == "route") {
        kind = NAV_TARGET_ROUTE;
    } else if (command == "home") {
        kind = NAV_TARGET_HOME;
    } else {
        sendJsonError(400, 4021, "invalid navigation target command");
        return;
    }

    if (kind == NAV_TARGET_POINT || kind == NAV_TARGET_ROUTE) {
        if (!server.hasArg("points") ||
            !parseNavigationPoints(server.arg("points"), points, count) ||
            (kind == NAV_TARGET_POINT && count != 1)) {
            sendJsonError(400, 4022, "invalid navigation target coordinates");
            return;
        }
    }

    if (!queueNavigationTarget(kind, points, count)) {
        sendJsonError(409, 4023,
            "target unavailable, outside the current map, or inside a known obstacle");
        return;
    }
    sendJsonResponse(202, true, 0, "navigation target queued", "");
}

static void handleApiAction()
{
    int actionId = PROGRAM_NONE;
    if (!resolveActionRequest(actionId)) {
        sendJsonError(400, 4001, "invalid action");
        return;
    }

    Servo_PROGRAM = actionId;
    setCurrentAction(actionId, false);

    String data = "{";
    data += "\"id\":";
    data += String(actionId);
    data += ",\"action\":\"";
    data += getActionName(actionId);
    data += "\",\"label\":\"";
    data += getActionLabel(actionId);
    data += "\"}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiCalibrationGet()
{
    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiCalibrationPost()
{
    int8_t offsets[ALLSERVOS];
    if (!parseCalibrationOffsets(offsets)) {
        sendJsonError(400, 4002, "invalid calibration payload");
        return;
    }

    for (int i = 0; i < ALLSERVOS; i++) {
        writeKeyValue(i, offsets[i]);
    }

    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiSteeringCalibrationGet()
{
    String data = "{\"degrees_per_meter\":";
    data += String(getForwardSteeringCorrection());
    data += "}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static bool isJsonIntegerProperty(const String& body, const char* property)
{
    String token = "\"";
    token += property;
    token += "\"";
    const int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }
    const int colonPos = body.indexOf(':', keyPos);
    if (colonPos < 0) {
        return false;
    }
    int cursor = colonPos + 1;
    while (cursor < body.length() && isspace(body[cursor])) {
        ++cursor;
    }
    if (cursor < body.length() && body[cursor] == '-') {
        ++cursor;
    }
    const int digitStart = cursor;
    while (cursor < body.length() && isdigit(body[cursor])) {
        ++cursor;
    }
    if (cursor == digitStart) {
        return false;
    }
    while (cursor < body.length() && isspace(body[cursor])) {
        ++cursor;
    }
    return cursor < body.length() &&
        (body[cursor] == ',' || body[cursor] == '}');
}

static bool parseSteeringCalibrationValue(int& degreesPerMeter)
{
    const String body = server.arg("plain");
    return isJsonIntegerProperty(body, "degrees_per_meter") &&
        extractJsonInt(body, "degrees_per_meter", degreesPerMeter);
}

static void handleApiSteeringCalibrationPost()
{
    int degreesPerMeter = 0;
    if (!parseSteeringCalibrationValue(degreesPerMeter) ||
        degreesPerMeter < -90 || degreesPerMeter > 90) {
        sendJsonError(400, 4005,
            "degrees_per_meter must be a whole number from -90 to 90");
        return;
    }
    if (!setForwardSteeringCorrection(degreesPerMeter)) {
        sendJsonError(500, 5005, "could not persist steering calibration");
        return;
    }

    String data = "{\"degrees_per_meter\":";
    data += String(getForwardSteeringCorrection());
    data += "}";
    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiRotationCalibrationGet()
{
    String data = "{\"observed_360_degrees\":";
    data += String(getObserved360RotationDegrees());
    data += ",\"calibration_equivalent_degrees\":";
    data += String(getRotationCalibrationEquivalentDegrees());
    data += "}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiRotationCalibrationPost()
{
    const String body = server.arg("plain");
    int observed360 = 0;
    if (!isJsonIntegerProperty(body, "observed_360_degrees") ||
        !extractJsonInt(body, "observed_360_degrees", observed360) ||
        observed360 < 300 || observed360 > 420) {
        sendJsonError(400, 4006,
            "observed_360_degrees must be a whole number from 300 to 420");
        return;
    }
    if (!setObserved360RotationDegrees((uint16_t)observed360)) {
        sendJsonError(500, 5006, "could not persist turn calibration");
        return;
    }

    String data = "{\"observed_360_degrees\":";
    data += String(getObserved360RotationDegrees());
    data += ",\"calibration_equivalent_degrees\":";
    data += String(getRotationCalibrationEquivalentDegrees());
    data += "}";
    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiServo()
{
    int servoId = -1;
    int servoValue = 0;
    if (!parseServoCommand(servoId, servoValue)) {
        sendJsonError(400, 4003, "invalid servo payload");
        return;
    }

    GPIO_ID = servoId;
    ival = String(servoValue);

    String data = "{";
    data += "\"servo\":";
    data += String(servoId);
    data += ",\"value\":";
    data += String(servoValue);
    data += "}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiStop()
{
    int targetProgram = PROGRAM_STANDBY;
    if (!resolveStopMode(targetProgram)) {
        sendJsonError(400, 4004, "invalid stop mode");
        return;
    }

    Servo_PROGRAM = PROGRAM_NONE;
    requestRobotStop(targetProgram);

    String data = "{";
    data += "\"mode\":\"";
    data += getActionName(targetProgram);
    data += "\",\"target_program\":";
    data += String(targetProgram);
    data += "}";

    sendJsonResponse(200, true, 0, "stop requested", data);
}

static void handleDebugUiOpen()
{
    g_debugUiEnabled = true;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

static void handleDebugUiClose()
{
    g_debugUiEnabled = false;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

// Save calibration offset: key=servo index (0-7), value=offset (-124~124)
// key=100 resets all offsets to 0
void handleSave()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String key   = server.arg("key");
    String value = server.arg("value");

    const int keyInt = key.toInt();
    const int valueInt = value.toInt();

    delay(50);

    if (keyInt == 100) {
        for (int i = 0; i < 8; i++) writeKeyValue(i, 0);
    } else if (keyInt >= 0 && keyInt < ALLSERVOS &&
        valueInt >= -124 && valueInt <= 124) {
        writeKeyValue((int8_t)keyInt, (int8_t)valueInt);
    } else {
        server.send(400, "text/plain", "Invalid servo calibration");
        return;
    }

    delay(10);
    server.send(200, "text/plain", "OK");
}

// Robot motion control: pm=program number, or servo=index&value=angle
void handleController()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String pm    = server.arg("pm");
    String servo = server.arg("servo");

    if (pm != "") {
        int requestedProgram = pm.toInt();
        if (findRobotActionById(requestedProgram) == nullptr) {
            server.send(400, "text/plain", "INVALID_ACTION");
            return;
        }
        Servo_PROGRAM = requestedProgram;
    }

    if (servo != "") {
        int requestedServo = servo.toInt();
        String value = server.arg("value");

        if (requestedServo < 0 || requestedServo >= ALLSERVOS || value == "") {
            server.send(400, "text/plain", "INVALID_SERVO");
            return;
        }

        GPIO_ID = requestedServo;
        ival    = value;
    }

    server.send(200, "text/plain", "OK");
}

// ── Route registration ────────────────────────────────────────────────────────

void enableWebServer()
{
    server.on("/",           HTTP_GET, handleIndex);
    server.on("/debug/open", HTTP_GET, handleDebugUiOpen);
    server.on("/debug/close", HTTP_GET, handleDebugUiClose);
    server.on("/controller", HTTP_GET, handleController);
    server.on("/save",       HTTP_GET, handleSave);
    server.on("/setting1",   HTTP_GET, handleSetting1);
    server.on("/api/v1/info",        HTTP_GET, handleApiInfo);
    server.on("/api/v1/state",       HTTP_GET, handleApiState);
    server.on("/api/v1/navigation", HTTP_GET, handleApiNavigation);
    server.on("/api/v1/navigation/control", HTTP_POST, handleApiNavigationControl);
    server.on("/api/v1/navigation/target", HTTP_POST, handleApiNavigationTarget);
    server.on("/api/v1/action",      HTTP_POST, handleApiAction);
    server.on("/api/v1/calibration", HTTP_GET, handleApiCalibrationGet);
    server.on("/api/v1/calibration", HTTP_POST, handleApiCalibrationPost);
    server.on("/api/v1/calibration/steering", HTTP_GET,
        handleApiSteeringCalibrationGet);
    server.on("/api/v1/calibration/steering", HTTP_POST,
        handleApiSteeringCalibrationPost);
    server.on("/api/v1/calibration/rotation", HTTP_GET,
        handleApiRotationCalibrationGet);
    server.on("/api/v1/calibration/rotation", HTTP_POST,
        handleApiRotationCalibrationPost);
    server.on("/api/v1/servo",       HTTP_POST, handleApiServo);
    server.on("/api/v1/stop",        HTTP_POST, handleApiStop);

    server.begin();
}
