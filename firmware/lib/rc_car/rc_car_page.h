#ifndef RC_CAR_PAGE_H
#define RC_CAR_PAGE_H

#include <Arduino.h>

// the whole controller ui, served straight out of flash. left pad steers,
// right pad throttles, both spring back to centre when you let go.
static const char RC_CAR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#0b0f14">
<meta name="mobile-web-app-capable" content="yes">
<title>Gorur Gari RC</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{height:100%;overflow:hidden;overscroll-behavior:none}
body{
  background:#0b0f14;color:#e6edf3;
  font:600 14px/1.3 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;
  display:flex;flex-direction:column;
  padding:max(8px,env(safe-area-inset-top)) max(8px,env(safe-area-inset-right)) max(8px,env(safe-area-inset-bottom)) max(8px,env(safe-area-inset-left));
  gap:8px;
}
header{display:flex;align-items:center;gap:8px;flex-wrap:wrap;flex:none}
.dot{width:9px;height:9px;border-radius:50%;background:#f85149;box-shadow:0 0 8px currentColor;color:#f85149;transition:.2s}
.dot.on{background:#3fb950;color:#3fb950}
.name{font-size:13px;letter-spacing:.04em;color:#7d8590;text-transform:uppercase}
.chips{display:flex;gap:6px;margin-left:auto;flex-wrap:wrap}
.chip{background:#161b22;border:1px solid #21262d;border-radius:7px;padding:4px 8px;display:flex;gap:6px;align-items:baseline;min-width:64px}
.chip b{font-size:10px;color:#7d8590;font-weight:600;letter-spacing:.05em}
.chip span{font-variant-numeric:tabular-nums;font-size:13px;margin-left:auto}
main{flex:1;display:flex;gap:8px;min-height:0}
.pad{
  position:relative;flex:1;background:#0f141a;border:1px solid #21262d;border-radius:16px;
  touch-action:none;overflow:hidden;
}
.pad.live{border-color:#1f6feb}
.pad::after{
  content:attr(data-label);position:absolute;top:8px;left:0;right:0;text-align:center;
  font-size:10px;letter-spacing:.14em;color:#484f58;pointer-events:none
}
.rail{position:absolute;background:#21262d;border-radius:2px}
#steerPad .rail{left:12%;right:12%;top:50%;height:2px;margin-top:-1px}
#throttlePad .rail{top:12%;bottom:12%;left:50%;width:2px;margin-left:-1px}
.tick{position:absolute;background:#30363d}
#steerPad .tick{left:50%;top:38%;bottom:38%;width:2px;margin-left:-1px}
#throttlePad .tick{top:50%;left:38%;right:38%;height:2px;margin-top:-1px}
.knob{
  position:absolute;left:50%;top:50%;width:58px;height:58px;margin:-29px 0 0 -29px;border-radius:50%;
  background:radial-gradient(circle at 35% 30%,#3d444d,#1c2128);border:2px solid #58a6ff;
  box-shadow:0 0 16px rgba(88,166,255,.28);pointer-events:none;
  display:flex;align-items:center;justify-content:center;
  font-size:12px;font-variant-numeric:tabular-nums;color:#8b949e;
  transition:transform .16s cubic-bezier(.2,1.4,.4,1)
}
.knob.held{transition:none;border-color:#79c0ff;box-shadow:0 0 22px rgba(121,192,255,.5)}
footer{display:flex;gap:8px;flex:none}
button{
  flex:1;font:inherit;font-size:13px;letter-spacing:.08em;padding:13px;border-radius:11px;
  border:1px solid #21262d;background:#161b22;color:#e6edf3;cursor:pointer;touch-action:manipulation
}
button:active{background:#21262d}
#stop{background:#3d1417;border-color:#7d2b31;color:#ffa198;flex:2}
#stop.armed{background:#161b22;border-color:#21262d;color:#7d8590}
@media (max-width:520px) and (orientation:portrait){
  .chips{width:100%;margin-left:0}
  .chip{flex:1}
}
</style>
</head>
<body>
<header>
  <i class="dot" id="dot"></i>
  <span class="name" id="state">connecting</span>
  <div class="chips">
    <div class="chip"><b>RPM</b><span id="rpm">0</span></div>
    <div class="chip"><b>TICKS</b><span id="ticks">0</span></div>
    <div class="chip"><b>RSSI</b><span id="rssi">-</span></div>
  </div>
</header>

<main>
  <div class="pad" id="steerPad" data-label="STEERING">
    <div class="rail"></div><div class="tick"></div>
    <div class="knob" id="steerKnob">0</div>
  </div>
  <div class="pad" id="throttlePad" data-label="THROTTLE">
    <div class="rail"></div><div class="tick"></div>
    <div class="knob" id="throttleKnob">0</div>
  </div>
</main>

<footer>
  <button id="stop">E-STOP</button>
  <button id="trim">CENTER STEER</button>
  <button id="zero">ZERO ODO</button>
</footer>

<script>
(() => {
  const $ = id => document.getElementById(id);
  const clamp = v => v < -1 ? -1 : v > 1 ? 1 : v;

  let armed = true, socket = null, live = false;
  const input = {steer: 0, throttle: 0};

  // a pad reports -1..1 along one axis and springs back to 0 on release.
  function pad(padId, knobId, axis, invert){
    const el = $(padId), knob = $(knobId);
    let id = null;

    const travel = () => (axis === 'x' ? el.clientWidth : el.clientHeight) / 2 - 34;

    const move = e => {
      const box = el.getBoundingClientRect();
      const offset = axis === 'x'
        ? e.clientX - (box.left + box.width / 2)
        : e.clientY - (box.top + box.height / 2);
      const value = clamp(offset / travel()) * (invert ? -1 : 1);
      input[axis === 'x' ? 'steer' : 'throttle'] = value;
      render(value);
    };

    const render = value => {
      const shift = value * (invert ? -1 : 1) * travel();
      knob.style.transform = axis === 'x' ? `translateX(${shift}px)` : `translateY(${shift}px)`;
      knob.textContent = value.toFixed(2);
    };

    el.addEventListener('pointerdown', e => {
      if(id !== null) return;
      id = e.pointerId;
      el.setPointerCapture(id);
      el.classList.add('live'); knob.classList.add('held');
      move(e);
    });
    el.addEventListener('pointermove', e => { if(e.pointerId === id) move(e); });

    const release = e => {
      if(e.pointerId !== id) return;
      id = null;
      el.classList.remove('live'); knob.classList.remove('held');
      input[axis === 'x' ? 'steer' : 'throttle'] = 0;
      render(0);
    };
    el.addEventListener('pointerup', release);
    el.addEventListener('pointercancel', release);

    return {reset: () => render(0)};
  }

  const steer = pad('steerPad', 'steerKnob', 'x', false);
  const throttle = pad('throttlePad', 'throttleKnob', 'y', true); // up is forward

  function connect(){
    socket = new WebSocket(`ws://${location.host}/ws`);

    socket.onopen = () => {
      live = true;
      $('dot').classList.add('on');
      $('state').textContent = 'connected';
    };

    socket.onclose = () => {
      live = false;
      $('dot').classList.remove('on');
      $('state').textContent = 'reconnecting';
      setTimeout(connect, 1000);
    };

    socket.onerror = () => socket.close();

    socket.onmessage = e => {
      const t = JSON.parse(e.data);
      $('rpm').textContent = t.rpm.toFixed(0);
      $('ticks').textContent = t.ticks;
      $('rssi').textContent = t.rssi ? `${t.rssi}` : 'ap';
    };
  }

  const send = obj => { if(live && socket.readyState === 1) socket.send(JSON.stringify(obj)); };

  // 20hz is plenty to keep the firmware failsafe fed and feels instant.
  setInterval(() => send({
    t: 'drive',
    s: +input.steer.toFixed(3),
    p: armed ? +input.throttle.toFixed(3) : 0
  }), 50);

  $('stop').addEventListener('click', e => {
    armed = !armed;
    e.target.classList.toggle('armed', !armed);
    e.target.textContent = armed ? 'E-STOP' : 'ARM';
    if(!armed){ input.throttle = 0; throttle.reset(); }
  });

  $('trim').addEventListener('click', () => { input.steer = 0; steer.reset(); });
  $('zero').addEventListener('click', () => send({t: 'zero'}));

  connect();
})();
</script>
</body>
</html>
)rawliteral";

#endif // RC_CAR_PAGE_H
