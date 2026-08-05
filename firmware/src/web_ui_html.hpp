#pragma once

static const char WEB_UI_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>HoloDex Control</title>
<style>
:root{
  --bg0:#0e141b;--bg1:#16202a;--line:#2a3a4a;--text:#e7eef5;--muted:#8aa0b5;
  --accent:#3dd6c6;--warn:#ffb020;--danger:#ff5a6a;--ok:#5adf8a;
  --pad:14px;--r:12px;
  font-family:"IBM Plex Sans",ui-sans-serif,system-ui,sans-serif;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:radial-gradient(1200px 600px at 20% -10%,#1b3344 0%,var(--bg0) 55%);color:var(--text);min-height:100vh}
header{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:14px var(--pad);border-bottom:1px solid var(--line);flex-wrap:wrap}
.brand{font-weight:700;letter-spacing:.04em;font-size:1.25rem}
.brand span{color:var(--accent)}
.ver{font-size:.75rem;color:var(--muted);font-variant-numeric:tabular-nums}
.badge{font-size:.75rem;padding:4px 10px;border:1px solid var(--line);border-radius:999px;color:var(--muted)}
.badge.on{color:var(--ok);border-color:#2f6b49}
.badge.off{color:var(--danger);border-color:#6b2f38}
.tabs{display:flex;gap:6px;padding:10px var(--pad) 0;max-width:960px;margin:0 auto;flex-wrap:wrap}
.tab{border:1px solid var(--line);background:#13202c;color:var(--muted);border-radius:8px;padding:8px 12px;font-weight:600;cursor:pointer}
.tab.active{color:var(--text);border-color:var(--accent);background:#1a2e3a}
main{display:none;gap:14px;padding:var(--pad);max-width:960px;margin:0 auto}
main.active{display:grid}
@media(min-width:820px){#pane-drive.active{grid-template-columns:1.15fr .85fr}}
.card{background:linear-gradient(180deg,rgba(255,255,255,.03),transparent),var(--bg1);border:1px solid var(--line);border-radius:var(--r);padding:14px}
h2{margin:0 0 10px;font-size:.85rem;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);font-weight:600}
.joys{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:end}
@media(max-width:520px){.joys{grid-template-columns:1fr}}
.joy-block{text-align:center}
.joy-block .cap{font-size:.75rem;color:var(--muted);margin-bottom:6px;letter-spacing:.06em;text-transform:uppercase}
.pad{touch-action:none;border-radius:50%;background:radial-gradient(circle at 50% 50%,#1c2b38 0%,#121c26 70%);border:1px solid var(--line);position:relative;margin:0 auto}
.pad.move{width:min(42vw,240px);height:min(42vw,240px)}
.pad.turn{width:min(36vw,180px);height:min(36vw,180px)}
.pad .knob{position:absolute;border-radius:50%;background:var(--accent);left:50%;top:50%;transform:translate(-50%,-50%)}
.pad.move .knob{width:64px;height:64px}
.pad.turn .knob{width:52px;height:52px;background:#7ec8ff}
.row{display:flex;gap:10px;align-items:center;margin-top:10px}
label{font-size:.8rem;color:var(--muted);min-width:88px}
input[type=range]{flex:1}
.val{min-width:72px;text-align:right;font-variant-numeric:tabular-nums;font-size:.85rem;color:var(--text)}
.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.metric{background:#0f1720;border:1px solid var(--line);border-radius:8px;padding:8px}
.metric b{display:block;font-size:1.05rem;font-variant-numeric:tabular-nums}
.metric small{color:var(--muted)}
.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
button{appearance:none;border:1px solid var(--line);background:#13202c;color:var(--text);border-radius:8px;padding:10px 12px;font-weight:600;cursor:pointer}
button.danger{background:#3a1720;border-color:#6b2f38;color:#ffc1c8}
button.ok{background:#143226;border-color:#2f6b49;color:#b7f0cc}
input[type=text],input[type=password],input[type=number],input[type=file]{width:100%;background:#0f1720;border:1px solid var(--line);border-radius:8px;color:var(--text);padding:10px}
.hint{color:var(--muted);font-size:.78rem;line-height:1.35}
.bar{height:8px;background:#0f1720;border-radius:99px;overflow:hidden;border:1px solid var(--line)}
.bar>i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--accent),#5adf8a)}
.motor-row{margin-top:12px;padding-top:10px;border-top:1px solid var(--line)}
.motor-row .val{min-width:64px}
.seg{display:flex;gap:4px;flex-wrap:wrap;margin:8px 0 10px}
.seg button{flex:1;min-width:88px;padding:8px 6px;font-size:.72rem;letter-spacing:.03em;text-transform:uppercase}
.seg button.active{border-color:var(--accent);color:var(--accent);background:#1a2e3a}
.odom-table{width:100%;border-collapse:collapse;font-size:.78rem;font-variant-numeric:tabular-nums;margin-top:8px}
.odom-table th,.odom-table td{border-bottom:1px solid var(--line);padding:6px 4px;text-align:right}
.odom-table th:first-child,.odom-table td:first-child{text-align:left;color:var(--muted)}
.odom-table th{color:var(--muted);font-weight:600;font-size:.68rem;text-transform:uppercase;letter-spacing:.04em}
.odom-table tr.sel td{color:var(--accent)}
</style>
</head>
<body>
<header>
  <div>
    <div class="brand">HOLO<span>DEX</span></div>
    <div class="ver" id="fwVer">fw —</div>
  </div>
  <div id="wsBadge" class="badge off">WS offline</div>
</header>
<nav class="tabs">
  <button class="tab active" data-pane="drive">Drive</button>
  <button class="tab" data-pane="motors">Motor test</button>
  <button class="tab" data-pane="config">Config / OTA</button>
</nav>

<main id="pane-drive" class="active">
  <section class="card">
    <h2>Teleoperation</h2>
    <div class="joys">
      <div class="joy-block">
        <div class="cap">Move · XY</div>
        <div class="pad move" id="joyMove"><div class="knob" id="knobMove"></div></div>
      </div>
      <div class="joy-block">
        <div class="cap">Turn · yaw</div>
        <div class="pad turn" id="joyTurn"><div class="knob" id="knobTurn"></div></div>
      </div>
    </div>
    <div class="row">
      <label>Max speed</label>
      <input id="maxSpeed" type="range" min="5" max="100" step="1" value="70"/>
      <span class="val" id="maxSpeedVal">70%</span>
    </div>
    <div class="btns">
      <button class="danger" id="estop">E-STOP</button>
      <button id="clearEstop">Clear E-Stop</button>
      <button id="stop">Stop</button>
      <button class="ok" id="resetOdom">Reset odom</button>
    </div>
    <p class="hint" id="estopHint">Left = XY translate · Right X = yaw (mecanum, spin about center). Order: M0 FL · M1 RL · M2 FR · M3 RR (left motors flipped by default).</p>
  </section>
  <section class="card">
    <h2>Odometry</h2>
    <div class="seg" id="odomViewSeg">
      <button type="button" data-view="wheels">Pure · steppers</button>
      <button type="button" data-view="enc">+ Encoders</button>
      <button type="button" data-view="full" class="active">+ IMU</button>
    </div>
    <p class="hint" id="odomViewHint">Full: wheels+enc rates, yaw from IMU (+ Kalman). Compare all three below.</p>
    <div class="metrics">
      <div class="metric"><small>x (m)</small><b id="ox">0.000</b></div>
      <div class="metric"><small>y (m)</small><b id="oy">0.000</b></div>
      <div class="metric"><small>θ (deg)</small><b id="oth">0.0</b></div>
      <div class="metric"><small>vx</small><b id="ovx">0.00</b></div>
      <div class="metric"><small>vy</small><b id="ovy">0.00</b></div>
      <div class="metric"><small>ω</small><b id="ow">0.00</b></div>
    </div>
    <table class="odom-table" id="odomCompare">
      <thead>
        <tr><th>Stream</th><th>x</th><th>y</th><th>θ°</th><th>ω</th></tr>
      </thead>
      <tbody>
        <tr data-view="wheels"><td>Pure (steppers)</td><td id="cx_w">—</td><td id="cy_w">—</td><td id="cth_w">—</td><td id="cw_w">—</td></tr>
        <tr data-view="enc"><td>+ Encoders</td><td id="cx_e">—</td><td id="cy_e">—</td><td id="cth_e">—</td><td id="cw_e">—</td></tr>
        <tr data-view="full" class="sel"><td>+ IMU</td><td id="cx_f">—</td><td id="cy_f">—</td><td id="cth_f">—</td><td id="cw_f">—</td></tr>
      </tbody>
    </table>
    <div class="row" style="margin-top:12px"><label>Src</label><b id="src">none</b></div>
    <div class="row"><label>Odom</label><span id="odomFusion">-</span></div>
    <div class="row"><label>Wheels</label><span id="wheelSrc">-</span></div>
    <div class="row"><label>Cmd</label><span id="cmdDbg">-</span></div>
    <div class="row"><label>STA</label><span id="sta">-</span></div>
    <div class="row"><label>Zenoh</label><span id="zenoh">-</span></div>
    <h2 style="margin-top:14px">Cmd wheel rates (Hz)</h2>
    <div id="wheels" class="metrics"></div>
    <h2 style="margin-top:14px">Encoders · AS5600 <span id="encBadge" class="badge off">—</span></h2>
    <div id="encoders" class="metrics"></div>
    <p class="hint" id="encHint">Mux ch0–3 · angle + measured Hz (odom source when fresh)</p>
    <h2 style="margin-top:14px">IMU · BMI160 <span id="imuBadge" class="badge off">—</span></h2>
    <div class="metrics">
      <div class="metric"><small>gx (°/s)</small><b id="igx">—</b></div>
      <div class="metric"><small>gy (°/s)</small><b id="igy">—</b></div>
      <div class="metric"><small>gz (°/s)</small><b id="igz">—</b></div>
      <div class="metric"><small>ax (m/s²)</small><b id="iax">—</b></div>
      <div class="metric"><small>ay (m/s²)</small><b id="iay">—</b></div>
      <div class="metric"><small>az (m/s²)</small><b id="iaz">—</b></div>
    </div>
    <p class="hint" id="imuHint">BMI160 on mux ch6 · yaw fusion uses gz (rad/s)</p>
  </section>
</main>

<main id="pane-motors">
  <section class="card" style="grid-column:1/-1">
    <h2>Individual motor test</h2>
    <p class="hint">Idle motors are <b>disabled</b> (freewheel) so one wheel can spin alone. Teleop keeps all enabled.</p>
    <div id="motorSliders"></div>
    <div class="row"><label>Test %</label><input id="testScale" type="range" min="5" max="80" step="5" value="30"/><span class="val" id="testScaleVal">30%</span></div>
    <div class="btns">
      <button class="ok" id="testAllFwd">All +</button>
      <button id="testAllRev">All −</button>
      <button id="testYawCCW">Yaw CCW (IK)</button>
      <button id="testYawCW">Yaw CW (IK)</button>
      <button class="danger" id="testStop">Stop all</button>
    </div>
    <p class="hint" id="testDbg">cmd: —</p>
    <p class="hint">Yaw IK: |Hz| equal with left/right signs. Pins: M0(FL) P19/D18/E17 · M1(RL) P27/D14/E13 · M2(FR) P23/D22/E32 · M3(RR) P33/D25/E26</p>
  </section>
</main>

<main id="pane-config">
  <section class="card" style="grid-column:1/-1">
    <h2>SoftAP</h2>
    <div class="metrics">
      <div class="metric"><small>SSID</small><b id="apSsid">HoloDex</b></div>
      <div class="metric"><small>Password</small><b id="apPass">holodex123</b></div>
      <div class="metric"><small>URL</small><b id="apUrl">http://192.168.4.1/</b></div>
    </div>
    <p class="hint" style="margin-top:10px">LED: slow=idle · fast=drive · double=E-STOP · flicker=OTA</p>

    <h2 style="margin-top:18px">Configuration</h2>
    <form id="cfgForm">
      <div class="row"><label>STA SSID</label><input id="sta_ssid"/></div>
      <div class="row"><label>STA pass</label><input id="sta_password" type="password" placeholder="leave blank to keep"/></div>
      <div class="row"><label>Zenoh</label><input id="zenoh_endpoint" placeholder="tcp/192.168.1.100:7447"/></div>
      <div class="row"><label>r (m)</label><input id="wheel_radius_m" type="number" step="0.001"/></div>
      <p class="hint">Wheel <b>radius</b> (diameter 10 cm → r = 0.05). Used for odometry with steps/rev.</p>
      <div class="row"><label>lx (m)</label><input id="lx_m" type="number" step="0.001"/></div>
      <div class="row"><label>ly (m)</label><input id="ly_m" type="number" step="0.001"/></div>
      <div class="row"><label>steps/rev</label><input id="steps_per_rev" type="number" step="1"/></div>
      <div class="row"><label>Max step Hz</label><input id="max_step_hz" type="number" step="100"/></div>
      <div class="row"><label>Max accel</label><input id="max_accel_hz" type="number" step="100"/></div>
      <p class="hint">Firmware ceiling at Max speed 100%. Effective Hz = slider% × this value (e.g. 35% of 12000 = 4200 Hz).</p>
      <div class="row"><label>Inv fwd</label><input id="invert_vx" type="checkbox"/></div>
      <div class="row"><label>Inv strafe</label><input id="invert_vy" type="checkbox"/></div>
      <div class="row"><label>Inv yaw</label><input id="invert_omega" type="checkbox"/></div>
      <div class="row"><label>Flip motors</label>
        <span>
          <input id="ms0" type="checkbox"/>M0 FL
          <input id="ms1" type="checkbox"/>M1 RL
          <input id="ms2" type="checkbox"/>M2 FR
          <input id="ms3" type="checkbox"/>M3 RR
        </span>
      </div>

      <h2 style="margin-top:18px">Encoders · PCA9548A map</h2>
      <p class="hint">Mux channel (0–7) per motor. Default: M0←0 · M1←1 · M2←4 · M3←5 (IMU on ch6). Flip enc if shaft sense is inverted vs IK.</p>
      <div class="row"><label>M0 FL ← ch</label><input id="ech0" type="number" min="0" max="7" step="1"/></div>
      <div class="row"><label>M1 RL ← ch</label><input id="ech1" type="number" min="0" max="7" step="1"/></div>
      <div class="row"><label>M2 FR ← ch</label><input id="ech2" type="number" min="0" max="7" step="1"/></div>
      <div class="row"><label>M3 RR ← ch</label><input id="ech3" type="number" min="0" max="7" step="1"/></div>
      <div class="row"><label>Flip enc</label>
        <span>
          <input id="es0" type="checkbox"/>0
          <input id="es1" type="checkbox"/>1
          <input id="es2" type="checkbox"/>2
          <input id="es3" type="checkbox"/>3
        </span>
      </div>
      <div class="row"><label>Enc odom</label><input id="enc_odom" type="checkbox"/></div>
      <div class="row"><label>Speed FB</label><input id="enc_speed_fb" type="checkbox"/></div>
      <div class="row"><label>Enc blend</label><input id="enc_blend" type="number" min="0" max="1" step="0.05"/></div>
      <div class="row"><label>Yaw Kalman</label><input id="yaw_kalman" type="checkbox"/></div>
      <p class="hint">Enc odom: AS5600 rates (logical IK frame) blended with stepper. Speed FB: body-twist PI then re-IK (keeps spin about center). Yaw Kalman: gyro + wheel ω.</p>
      <div class="btns"><button class="ok" type="submit">Save &amp; apply</button></div>
    </form>

    <h2 style="margin-top:18px">Firmware OTA</h2>
    <form id="otaForm">
      <div class="row"><label>firmware.bin</label><input id="otaFile" type="file" accept=".bin"/></div>
      <div class="btns"><button class="ok" type="submit">Upload &amp; reboot</button></div>
      <p class="hint" id="otaStatus">Or: <code>pio run -e esp32doit-ota -t upload</code> (password holodex)</p>
    </form>
  </section>
</main>

<script>
let activePane='drive', estopLocal=false, maxStepHz=12000, lastSentActive=false, ws;
let mx=0, my=0, yaw=0;
let lxM=0.19, lyM=0.195;
let motorSign=[1,1,1,1];
let odomView='full';
let lastOdomMsg=null;
const badge=document.getElementById('wsBadge');
const wheels=document.getElementById('wheels');
const encoders=document.getElementById('encoders');
const motorPct=[0,0,0,0];
const maxSpeedEl=document.getElementById('maxSpeed');
const maxSpeedVal=document.getElementById('maxSpeedVal');
const odomHints={
  wheels:'Pure: stepper command rates only, yaw from wheel FK (no encoders, no IMU).',
  enc:'+ Encoders: AS5600 rates blended with steppers, yaw from wheel FK (no IMU).',
  full:'+ IMU: wheels+enc rates, yaw from gyro (+ Kalman bias). Primary / Zenoh odom.'
};
document.querySelectorAll('#odomViewSeg button').forEach(btn=>{
  btn.onclick=()=>{
    odomView=btn.dataset.view;
    document.querySelectorAll('#odomViewSeg button').forEach(b=>b.classList.toggle('active', b===btn));
    document.querySelectorAll('#odomCompare tr[data-view]').forEach(tr=>tr.classList.toggle('sel', tr.dataset.view===odomView));
    odomViewHint.textContent=odomHints[odomView]||'';
    if(lastOdomMsg) applyOdom(lastOdomMsg);
  };
});

function bindPad(pad, knob, onChange, axisMode){
  // axisMode: 'xy' | 'x' (yaw uses horizontal only)
  let dragging=false;
  function setKnob(nx,ny){
    const r=pad.clientWidth/2, kr=knob.clientWidth/2;
    knob.style.left=(r+nx*(r-kr))+'px';
    knob.style.top=(r-ny*(r-kr))+'px';
  }
  function pointerToVec(e){
    const rect=pad.getBoundingClientRect();
    const cx=rect.left+rect.width/2, cy=rect.top+rect.height/2;
    let x=(e.clientX-cx)/(rect.width/2), y=-(e.clientY-cy)/(rect.height/2);
    if(axisMode==='x'){ y=0; }
    const n=Math.hypot(x,y); if(n>1){x/=n;y/=n;}
    return [x,y];
  }
  function down(e){dragging=true;pad.setPointerCapture(e.pointerId);move(e)}
  function move(e){
    if(!dragging) return;
    const [x,y]=pointerToVec(e);
    setKnob(x,y);
    onChange(x,y,true);
  }
  function up(e){
    dragging=false; setKnob(0,0);
    try{pad.releasePointerCapture(e.pointerId)}catch(_){}
    onChange(0,0,false);
  }
  pad.addEventListener('pointerdown',down);
  pad.addEventListener('pointermove',move);
  pad.addEventListener('pointerup',up);
  pad.addEventListener('pointercancel',up);
  setKnob(0,0);
  return {reset(){ setKnob(0,0); onChange(0,0,false); }};
}

let moveActive=false, turnActive=false;
const movePad=bindPad(joyMove, knobMove, (x,y,active)=>{ mx=x; my=y; moveActive=active; if(!active) pushCmd(true); }, 'xy');
const turnPad=bindPad(joyTurn, knobTurn, (x,y,active)=>{ yaw=x; turnActive=active; if(!active) pushCmd(true); }, 'x');

maxSpeedEl.oninput=()=>{
  const pct=+maxSpeedEl.value;
  maxSpeedVal.textContent=pct+'% ('+Math.round(pct/100*maxStepHz)+' Hz)';
};
maxSpeedEl.oninput();

const motorCorner=['FL','RL','FR','RR'];
for(let i=0;i<4;i++){
  const d=document.createElement('div');
  d.className='metric';
  d.innerHTML=`<small>M${i} ${motorCorner[i]}</small><b id="w${i}">0</b><div class="bar"><i id="wb${i}"></i></div>`;
  wheels.appendChild(d);
  const e=document.createElement('div');
  e.className='metric';
  e.innerHTML=`<small id="el${i}">M${i} ${motorCorner[i]}←ch?</small><b id="ea${i}">—</b><small id="eh${i}" style="display:block;margin-top:2px">0 Hz</small><div class="bar"><i id="eb${i}"></i></div>`;
  encoders.appendChild(e);
  const row=document.createElement('div');
  row.className='motor-row';
  row.innerHTML=`<div class="row"><label>M${i} ${motorCorner[i]}</label><input id="msl${i}" type="range" min="-100" max="100" step="1" value="0"/><span class="val" id="msv${i}">0%</span></div>`;
  motorSliders.appendChild(row);
  document.getElementById('msl'+i).addEventListener('input', (e)=>{
    motorPct[i]=+e.target.value;
    document.getElementById('msv'+i).textContent=motorPct[i]+'%';
  });
}
testScale.oninput=()=>{testScaleVal.textContent=testScale.value+'%'};

document.querySelectorAll('.tab').forEach(btn=>{
  btn.onclick=()=>{
    document.querySelectorAll('.tab').forEach(b=>b.classList.remove('active'));
    document.querySelectorAll('main').forEach(p=>p.classList.remove('active'));
    btn.classList.add('active');
    activePane=btn.dataset.pane;
    document.getElementById('pane-'+activePane).classList.add('active');
    if(activePane!=='motors'){ motorPct.fill(0); for(let i=0;i<4;i++){ document.getElementById('msl'+i).value=0; document.getElementById('msv'+i).textContent='0%'; } pushMotors([0,0,0,0], false); }
    if(activePane!=='drive'){ mx=0;my=0;yaw=0;moveActive=false;turnActive=false; movePad.reset(); turnPad.reset(); }
  };
});

/**
 * Mecanum mix (matches firmware HolonomicKinematics).
 * Motor order: M0 FL, M1 RL, M2 FR, M3 RR.
 * Stick-normalized: L=1 so full yaw matches full translate before peak scale.
 *   FL =  vx − vy − ω
 *   RL =  vx + vy − ω
 *   FR =  vx + vy + ω
 *   RR =  vx − vy + ω
 */
function mecanumMix(vx, vy, w){
  return [vx-vy-w, vx+vy-w, vx+vy+w, vx-vy+w];
}

function motorsFromStick(sx, sy, rot, speedFrac){
  if(estopLocal) return [0,0,0,0];
  // sy=forward (vx), sx=strafe (vy), rot=yaw (ω)
  let dirs=mecanumMix(sy, sx, rot);
  const peak=Math.max(...dirs.map(v=>Math.abs(v)), 1e-9);
  if(peak>1) dirs=dirs.map(v=>v/peak);
  const hz=speedFrac*maxStepHz;
  return dirs.map(v=>Math.round(v*hz));
}
function motorsFromTest(){
  if(estopLocal) return [0,0,0,0];
  const s=(+testScale.value)/100;
  return motorPct.map(p=>Math.round((p/100)*s*maxStepHz));
}
function motorsYawSpin(speedFrac, sense){
  return mecanumMix(0, 0, sense).map(v=>Math.round(v*speedFrac*maxStepHz));
}

function pickOdom(m, view){
  if(view==='wheels'){
    return {
      x:m.x_wheels!=null?m.x_wheels:m.x_step,
      y:m.y_wheels!=null?m.y_wheels:m.y_step,
      th:m.theta_wheels!=null?m.theta_wheels:m.theta_step,
      vx:m.vx_wheels, vy:m.vy_wheels, w:m.omega_wheels,
      fusion:m.odom_wheels_fusion||'wheels'
    };
  }
  if(view==='enc'){
    return {
      x:m.x_wheels_enc!=null?m.x_wheels_enc:m.x_enc,
      y:m.y_wheels_enc!=null?m.y_wheels_enc:m.y_enc,
      th:m.theta_wheels_enc!=null?m.theta_wheels_enc:m.theta_enc,
      vx:m.vx_wheels_enc, vy:m.vy_wheels_enc, w:m.omega_wheels_enc,
      fusion:m.odom_wheels_enc_fusion||'wheels+enc'
    };
  }
  return {
    x:m.x_full!=null?m.x_full:m.x,
    y:m.y_full!=null?m.y_full:m.y,
    th:m.theta_full!=null?m.theta_full:m.theta,
    vx:m.vx_full!=null?m.vx_full:m.vx,
    vy:m.vy_full!=null?m.vy_full:m.vy,
    w:m.omega_full!=null?m.omega_full:m.omega,
    fusion:m.odom_fusion||'wheels+enc+imu'
  };
}
function fmtN(v,d){ return (v==null||Number.isNaN(Number(v)))?'—':Number(v).toFixed(d); }
function applyOdom(m){
  lastOdomMsg=m;
  const r2d=180/Math.PI;
  const sel=pickOdom(m, odomView);
  ox.textContent=fmtN(sel.x,3); oy.textContent=fmtN(sel.y,3);
  oth.textContent=fmtN(sel.th!=null?sel.th*r2d:null,1);
  ovx.textContent=fmtN(sel.vx,2); ovy.textContent=fmtN(sel.vy,2); ow.textContent=fmtN(sel.w,2);
  const w=pickOdom(m,'wheels'), e=pickOdom(m,'enc'), f=pickOdom(m,'full');
  cx_w.textContent=fmtN(w.x,3); cy_w.textContent=fmtN(w.y,3); cth_w.textContent=fmtN(w.th!=null?w.th*r2d:null,1); cw_w.textContent=fmtN(w.w,2);
  cx_e.textContent=fmtN(e.x,3); cy_e.textContent=fmtN(e.y,3); cth_e.textContent=fmtN(e.th!=null?e.th*r2d:null,1); cw_e.textContent=fmtN(e.w,2);
  cx_f.textContent=fmtN(f.x,3); cy_f.textContent=fmtN(f.y,3); cth_f.textContent=fmtN(f.th!=null?f.th*r2d:null,1); cw_f.textContent=fmtN(f.w,2);
  src.textContent=m.estop?'ESTOP':(m.driving?'DRIVE':(m.source||'none'));
  odomFusion.textContent=(sel.fusion||'-')+(m.gyro_bias!=null && odomView==='full'?' · bias '+Number(m.gyro_bias).toFixed(4):'');
  wheelSrc.textContent=m.wheel_src||'-';
  sta.textContent=m.sta||'-'; zenoh.textContent=m.zenoh||'-';
  if(m.version) fwVer.textContent='fw '+m.version+(m.build?' ('+m.build+')':'');
  if(m.estop){ estopHint.textContent='E-STOP LATCHED — Clear E-Stop (LED double-blink)'; estopHint.style.color='var(--danger)'; }
  else { estopHint.textContent='Left stick = translate. Right stick X = rotate. Max speed scales all wheel rates together.'; estopHint.style.color=''; }
  if(m.cmd_n!==undefined){
    cmdDbg.textContent=`n=${m.cmd_n}  [${(m.wheels||[]).map(v=>Math.round(v)).join(' ')}]`;
  }
  for(let i=0;i<4;i++){
    const hz=(m.wheels&&m.wheels[i])||0;
    const ok=!m.motor_ok||m.motor_ok[i];
    document.getElementById('w'+i).textContent=ok?Math.round(hz):'FAIL';
    document.getElementById('wb'+i).style.width=Math.min(100,(Math.abs(hz)/Math.max(maxStepHz,1))*100)+'%';
  }
  const encN=m.enc_n|0;
  encBadge.textContent=m.mux?(encN+'/4'+(m.enc_fresh?' live':' —')):(encN?'bus':'no mux');
  encBadge.className='badge '+(encN>0?'on':'off');
  const encDef=[0,1,4,5];
  encHint.textContent=m.mux
    ?`Mux 0,1,4,5 · ${encN}/4 OK · odom uses encoder Hz when fresh, else stepper ramp`
    :`No PCA9548A · encoders unavailable · odom uses stepper ramp`;
  for(let i=0;i<4;i++){
    const ok=m.enc_ok&&m.enc_ok[i];
    const ch=(m.enc_ch&&m.enc_ch[i]!=null)?m.enc_ch[i]:encDef[i];
    document.getElementById('el'+i).textContent=`M${i} ${motorCorner[i]}←ch${ch}`;
    const deg=(m.enc_deg&&m.enc_deg[i]!=null)?Number(m.enc_deg[i]):NaN;
    const ehz=(m.enc_hz&&m.enc_hz[i])||0;
    document.getElementById('ea'+i).textContent=ok&&!Number.isNaN(deg)?deg.toFixed(1)+'°':'—';
    document.getElementById('eh'+i).textContent=ok?Math.round(ehz)+' Hz':'no sensor';
    document.getElementById('eb'+i).style.width=ok?Math.min(100,(Math.abs(ehz)/Math.max(maxStepHz,1))*100)+'%':'0%';
  }
  imuBadge.textContent=m.imu_ok?(m.mux?'mux ch6':'ok'):'down';
  imuBadge.className='badge '+(m.imu_ok?'on':'off');
  imuHint.textContent=m.imu_ok
    ?`gz ${Number(m.gz||0).toFixed(3)} rad/s · yaw fusion active`
    :`IMU down · yaw from wheel FK`;
  const fmt=(v,d)=> (v==null||Number.isNaN(Number(v)))?'—':Number(v).toFixed(d);
  igx.textContent=m.imu_ok?fmt(m.gx_dps,1):'—';
  igy.textContent=m.imu_ok?fmt(m.gy_dps,1):'—';
  igz.textContent=m.imu_ok?fmt(m.gz_dps,1):'—';
  iax.textContent=m.imu_ok?fmt(m.ax,2):'—';
  iay.textContent=m.imu_ok?fmt(m.ay,2):'—';
  iaz.textContent=m.imu_ok?fmt(m.az,2):'—';
  if(m.ap_ssid) apSsid.textContent=m.ap_ssid;
  if(m.ap_password) apPass.textContent=m.ap_password;
  if(m.ap_url) apUrl.textContent=m.ap_url;
  if(m.max_step_hz) maxStepHz=+m.max_step_hz;
}
function applyCfg(m){
  if(m.version) fwVer.textContent='fw '+m.version+(m.build?' ('+m.build+')':'');
  sta_ssid.value=m.sta_ssid||'';
  zenoh_endpoint.value=m.zenoh_endpoint||'';
  wheel_radius_m.value=m.wheel_radius_m;
  lx_m.value=m.lx_m; ly_m.value=m.ly_m;
  steps_per_rev.value=m.steps_per_rev;
  if(m.max_step_hz){ max_step_hz.value=m.max_step_hz; maxStepHz=+m.max_step_hz; }
  if(m.max_accel_hz) max_accel_hz.value=m.max_accel_hz;
  if(m.lx_m) lxM=+m.lx_m; if(m.ly_m) lyM=+m.ly_m;
  invert_vx.checked=!!m.invert_vx; invert_vy.checked=!!m.invert_vy; invert_omega.checked=!!m.invert_omega;
  const ms=m.motor_sign||[1,1,1,1];
  motorSign=ms.map(v=>v<0?-1:1);
  ms0.checked=ms[0]<0; ms1.checked=ms[1]<0; ms2.checked=ms[2]<0; ms3.checked=ms[3]<0;
  const ech=m.enc_ch||[0,1,4,5];
  ech0.value=ech[0]; ech1.value=ech[1]; ech2.value=ech[2]; ech3.value=ech[3];
  const es=m.enc_sign||[1,1,1,1];
  es0.checked=es[0]<0; es1.checked=es[1]<0; es2.checked=es[2]<0; es3.checked=es[3]<0;
  enc_odom.checked=m.enc_odom!==false;
  enc_speed_fb.checked=m.enc_speed_fb!==false;
  enc_blend.value=(m.enc_blend!=null)?m.enc_blend:0.85;
  yaw_kalman.checked=m.yaw_kalman!==false;
  if(m.ap_ssid) apSsid.textContent=m.ap_ssid;
  if(m.ap_password) apPass.textContent=m.ap_password;
  if(m.ap_url) apUrl.textContent=m.ap_url;
}
function connect(){
  ws=new WebSocket(`ws://${location.hostname}:81/`);
  ws.onopen=()=>{badge.textContent='WS ok'; badge.className='badge on'};
  ws.onclose=()=>{badge.textContent='HTTP'; badge.className='badge on'; setTimeout(connect,1500)};
  ws.onmessage=(ev)=>{ try{ const m=JSON.parse(ev.data); if(m.type==='odom') applyOdom(m); else if(m.type==='cfg') applyCfg(m);}catch(e){} };
}
let httpInflight=false;
function pushMotors(motors, testMode){
  const msg=`M ${motors.join(' ')}${testMode?' [test]':''}`;
  cmdDbg.textContent=msg;
  if(testDbg) testDbg.textContent='cmd: '+msg;
  if(ws&&ws.readyState===1){
    ws.send(JSON.stringify({type:'motors', test:!!testMode, m0:motors[0], m1:motors[1], m2:motors[2], m3:motors[3]}));
    return;
  }
  if(httpInflight) return;
  httpInflight=true;
  const t=testMode?'&test=1':'';
  fetch(`/api/cmd?m0=${motors[0]}&m1=${motors[1]}&m2=${motors[2]}&m3=${motors[3]}${t}&t=${Date.now()}`,{cache:'no-store'})
    .then(r=>r.json()).then(applyOdom).catch(()=>{}).finally(()=>{httpInflight=false;});
}
function pushCmd(forceStop){
  if(activePane!=='drive') return;
  const speedFrac=(+maxSpeedEl.value)/100;
  const active=forceStop||moveActive||turnActive||Math.hypot(mx,my)>0.02||Math.abs(yaw)>0.02;
  if(!active && !lastSentActive) return;
  const motors=active?motorsFromStick(mx, my, yaw, speedFrac):[0,0,0,0];
  lastSentActive=active && motors.some(v=>v!==0);
  pushMotors(motors, false);
}
function pushTest(force){
  if(activePane!=='motors') return;
  const speedFrac=(+testScale.value)/100;
  let motors;
  if(yawTest!==0) motors=motorsYawSpin(speedFrac, yawTest);
  else motors=motorsFromTest();
  const active=force||motors.some(v=>v!==0);
  if(!active && !lastSentActive) return;
  lastSentActive=active;
  // Yaw IK test uses teleop path (motor_sign applied) so Flip M3 is visible
  pushMotors(active?motors:[0,0,0,0], yawTest===0);
}
estop.onclick=()=>{ estopLocal=true; mx=0;my=0;yaw=0;moveActive=false;turnActive=false;movePad.reset();turnPad.reset();motorPct.fill(0);lastSentActive=false;
  for(let i=0;i<4;i++){ document.getElementById('msl'+i).value=0; document.getElementById('msv'+i).textContent='0%'; }
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'estop'}));
  fetch('/api/estop').then(r=>r.json()).then(applyOdom);
};
clearEstop.onclick=()=>{ estopLocal=false;
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'clear_estop'}));
  fetch('/api/clear_estop').then(r=>r.json()).then(applyOdom);
};
stop.onclick=()=>{mx=0;my=0;yaw=0;moveActive=false;turnActive=false;movePad.reset();turnPad.reset();pushCmd(true)};
resetOdom.onclick=()=>{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'reset_odom'})); fetch('/api/reset_odom').then(r=>r.json()).then(applyOdom); };
testStop.onclick=()=>{ motorPct.fill(0); for(let i=0;i<4;i++){ document.getElementById('msl'+i).value=0; document.getElementById('msv'+i).textContent='0%'; } pushMotors([0,0,0,0], true); lastSentActive=false; };
testAllFwd.onclick=()=>{ for(let i=0;i<4;i++){ motorPct[i]=100; document.getElementById('msl'+i).value=100; document.getElementById('msv'+i).textContent='100%'; } };
testAllRev.onclick=()=>{ for(let i=0;i<4;i++){ motorPct[i]=-100; document.getElementById('msl'+i).value=-100; document.getElementById('msv'+i).textContent='-100%'; } };
let yawTest=0;
testYawCCW.onclick=()=>{ yawTest=1; motorPct.fill(0); for(let i=0;i<4;i++){ document.getElementById('msl'+i).value=0; document.getElementById('msv'+i).textContent='0%'; } };
testYawCW.onclick=()=>{ yawTest=-1; motorPct.fill(0); for(let i=0;i<4;i++){ document.getElementById('msl'+i).value=0; document.getElementById('msv'+i).textContent='0%'; } };
testStop.addEventListener('click',()=>{ yawTest=0; });

cfgForm.onsubmit=async (e)=>{
  e.preventDefault();
  const body={
    sta_ssid:sta_ssid.value, sta_password:sta_password.value, zenoh_endpoint:zenoh_endpoint.value,
    wheel_radius_m:+wheel_radius_m.value, lx_m:+lx_m.value, ly_m:+ly_m.value, steps_per_rev:+steps_per_rev.value,
    max_step_hz:+max_step_hz.value, max_accel_hz:+max_accel_hz.value,
    invert_vx:invert_vx.checked, invert_vy:invert_vy.checked, invert_omega:invert_omega.checked,
    motor_sign:[ms0.checked?-1:1, ms1.checked?-1:1, ms2.checked?-1:1, ms3.checked?-1:1],
    enc_ch:[+ech0.value,+ech1.value,+ech2.value,+ech3.value],
    enc_sign:[es0.checked?-1:1, es1.checked?-1:1, es2.checked?-1:1, es3.checked?-1:1],
    enc_odom:enc_odom.checked, enc_speed_fb:enc_speed_fb.checked,
    enc_blend:+enc_blend.value, yaw_kalman:yaw_kalman.checked
  };
  maxStepHz=body.max_step_hz||maxStepHz;
  await fetch('/api/cfg',{method:'POST',headers:{'Content-Type':'text/plain'},body:JSON.stringify(body)});
  if(ws&&ws.readyState===1) ws.send(JSON.stringify(Object.assign({type:'save_cfg'}, body)));
};
otaForm.onsubmit=async (e)=>{
  e.preventDefault();
  const f=otaFile.files&&otaFile.files[0];
  if(!f){ otaStatus.textContent='Choose firmware.bin first'; return; }
  otaStatus.textContent='Uploading '+f.name+'...';
  const fd=new FormData(); fd.append('firmware', f, 'firmware.bin');
  try{
    const r=await fetch('/api/update',{method:'POST', body:fd});
    otaStatus.textContent=(await r.text())+(r.ok?' — reconnect after reboot':'');
  }catch(err){ otaStatus.textContent='Error / rebooting: '+err; }
};
connect();
fetch('/api/cfg').then(r=>r.json()).then(applyCfg).catch(()=>{});
setInterval(()=>{ if(activePane==='drive') pushCmd(false); if(activePane==='motors') pushTest(false); }, 50);
setInterval(()=>{ if(ws&&ws.readyState===1) return; fetch('/api/odom',{cache:'no-store'}).then(r=>r.json()).then(applyOdom).catch(()=>{}); }, 250);
</script>
</body>
</html>)HTML";
