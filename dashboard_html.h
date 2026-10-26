#ifndef dashboard_html_h
#define dashboard_html_h

// Dashboard page stored in program flash and included at compile time.
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
      <div class="nav-note">Right-click the map (or long-press on touch) for Go here, animations, routes, pause/resume, and Return to home. Route points can be added with an animation; the robot plays it and continues, pausing at the final point. A single animated destination resumes coverage after the animation. All scanning and movement turns use complete turn gaits. Coverage follows 8 cm serpentine lanes spaced one step apart. The mini-map grows with observed area up to its firmware limit; unknown cells mean unobserved, not clear. Map targets stay anchored as the map recenters.</div>
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
      <p class="hint">Correction is applied during the forward gait by changing left/right foot-swing stride, not by adding a turn afterward. Positive values increase left stride and reduce right stride to counter left drift; negative values do the reverse. Enter measured drift in degrees per meter (-360 to 360). Stride bias is capped for safety, so large values eventually reach the same maximum. This is open-loop and cannot sense heading.</p>
      <div class="steering-calibration">
        <label for="steeringCorrection">Drift (degrees per meter)</label>
        <input type="number" id="steeringCorrection" value="30">
        <button class="set-btn" onclick="saveSteeringCorrection()">SET</button>
      </div>
      <div class="panel-title">Turn calibration</div>
      <p class="hint">Enter the physical angle the robot actually turned after a 360&deg; command (for example, 45, 180, 360, 450, 720, or 1935). Calibration accounts for the number of complete gait steps used, so run another 360&deg; test after saving to refine it. The correction applies equally to left and right turns and assumes both directions behave similarly. All turns use complete turn gaits.</p>
      <div class="steering-calibration">
        <label for="rotation360Observed">Measured angle</label>
        <input type="number" id="rotation360Observed" value="360">
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
      if(reason.indexOf('coverage complete') >= 0)
        return 'I have finished the reachable zig-zag coverage and stopped at the room boundary.';
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
function drawNavMap(grid, width, heading, mapOffsetX, mapOffsetY){
  var canvas = document.getElementById('navMap');
  var ctx = canvas.getContext('2d');
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
      drawNavMap(nav.grid, nav.map_width, nav.heading_deg,
        nav.map_offset_x, nav.map_offset_y);
    })
    .catch(function(){ showNavigationNotice('Telemetry unavailable; check the Wi-Fi connection', true); });
}

function mapPointFromEvent(event){
  if(!latestNavigation) return null;
  var canvas = document.getElementById('navMap');
  var rect = canvas.getBoundingClientRect();
  var width = latestNavigation.map_width;
  var x = Math.floor((event.clientX - rect.left) * width / rect.width);
  var y = Math.floor((event.clientY - rect.top) * width / rect.height);
  if(x < 0 || x >= width || y < 0 || y >= width) return null;
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
    if(latestNavigation) drawNavMap(latestNavigation.grid,
      latestNavigation.map_width, latestNavigation.heading_deg,
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
  drawNavMap(latestNavigation.grid, latestNavigation.map_width,
    latestNavigation.heading_deg,
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
        'Turn calibration saved: ' + response.data.observed_360_degrees +
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

#endif
