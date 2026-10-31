// Tiny, framework-free dashboard glue.
//   - listens to SSE events from the harness
//   - wires form controls to POST /command
//   - keeps minimal derived state in the DOM

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

// -- command sending --------------------------------------------------

async function send(cmd) {
  try {
    await fetch('/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cmd),
    });
  } catch (err) {
    console.error('send failed', err);
  }
}

// Buttons with data-cmd attribute fire that JSON verbatim.
$$('button[data-cmd]').forEach((btn) => {
  btn.addEventListener('click', () => send(JSON.parse(btn.dataset.cmd)));
});

// Pilot preset buttons.
$$('button[data-pilot]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const mv = parseInt(btn.dataset.pilot, 10);
    $('#pilot').value = mv;
    $('#pilot-out').value = `${mv} mV`;
    send({ cmd: 'set_pilot_mv', mv });
  });
});

// Debounced slider → command helpers.
function bindSlider(id, outId, cmdFn, unit = '') {
  const el = $('#' + id);
  const out = $('#' + outId);
  el.addEventListener('input', () => {
    out.value = `${el.value}${unit}`;
  });
  let debounce = null;
  el.addEventListener('change', () => {
    clearTimeout(debounce);
    debounce = setTimeout(() => send(cmdFn(parseFloat(el.value))), 50);
  });
}

bindSlider('pilot', 'pilot-out', (v) => ({ cmd: 'set_pilot_mv', mv: Math.round(v) }), ' mV');
bindSlider('temp', 'temp-out', (v) => ({ cmd: 'set_temp_c', temp: v }));

// Mains sliders: share a single command with all three values.
function sendMains() {
  send({
    cmd: 'set_mains',
    l1: parseFloat($('#l1').value),
    l2: parseFloat($('#l2').value),
    l3: parseFloat($('#l3').value),
  });
}
['l1', 'l2', 'l3'].forEach((id) => {
  const el = $('#' + id);
  const out = $('#' + id + '-out');
  el.addEventListener('input', () => { out.value = el.value; });
  el.addEventListener('change', sendMains);
});

// Toggles.
$('#plug').addEventListener('change', (e) => send({ cmd: 'plug_ev', connected: e.target.checked }));
$('#ext-switch').addEventListener('change', (e) => send({ cmd: 'ext_switch', on: e.target.checked }));

// Mode.
$('#mode').addEventListener('change', (e) => send({ cmd: 'set_mode', mode: e.target.value }));

// Override current (publishes via MQTT path to exercise the clamp).
$('#send-override').addEventListener('click', () => {
  const dA = parseInt($('#override').value, 10) || 0;
  send({
    cmd: 'mqtt_set',
    topic: 'SmartEVSE-12345/Set/CurrentOverride',
    payload: String(dA),
  });
});

// Free-form MQTT send.
$('#send-mqtt').addEventListener('click', () => {
  const topic = $('#mqtt-topic').value.trim();
  const payload = $('#mqtt-payload').value;
  if (!topic) return;
  send({ cmd: 'mqtt_set', topic, payload });
});

// Autoplay: wall-clock driven ticks at a multiplier.
let autoplayTimer = null;
$('#autoplay').addEventListener('change', (e) => {
  const mult = parseInt(e.target.value, 10);
  clearInterval(autoplayTimer);
  if (mult > 0) {
    autoplayTimer = setInterval(() => send({ cmd: 'tick', ms: 100 * mult }), 100);
  }
});

// -- recording / export ----------------------------------------------

$('#rec-start').addEventListener('click', async () => {
  await fetch('/recording/start', { method: 'POST' });
  $('#rec-start').disabled = true;
  $('#rec-stop').disabled = false;
});

$('#rec-stop').addEventListener('click', async () => {
  const res = await fetch('/recording/stop', { method: 'POST' });
  const data = await res.json();
  const yaml = toScenarioYaml(data.commands);
  downloadText(`scenario-${Date.now()}.scenario.yaml`, yaml);
  $('#rec-start').disabled = false;
  $('#rec-stop').disabled = true;
});

function toScenarioYaml(commands) {
  const lines = ['# generated from dashboard session', 'steps:'];
  for (const { cmd } of commands) {
    lines.push(`  - ${JSON.stringify(cmd)}`);
  }
  return lines.join('\n') + '\n';
}

function downloadText(filename, text) {
  const blob = new Blob([text], { type: 'text/yaml' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  a.click();
  URL.revokeObjectURL(a.href);
}

// -- event stream -----------------------------------------------------

const transitions = $('#transitions');
const logBody = $('#log-body');
const mqttBody = $('#mqtt-body');
const mqttRows = new Map(); // topic -> <tr>
const showTicks = () => $('#filter-tick').checked;

function addTransition(ev) {
  const li = document.createElement('li');
  li.innerHTML = `<span class="t">${ev.t} ms</span>
                  <span>${ev.from}</span>
                  <span class="arrow">→</span>
                  <span><b>${ev.to}</b></span>
                  <span class="sub">${ev.reason}</span>`;
  transitions.prepend(li);
  while (transitions.children.length > 50) transitions.lastChild.remove();
}

function addLog(ev) {
  if (ev.kind === 'log' && ev.level === 'stderr' && !ev.msg) return;
  if (ev.kind === 'tick' && !showTicks()) return;
  const li = document.createElement('li');
  const level = ev.level || '';
  if (level === 'warn') li.classList.add('warn');
  if (level === 'error' || ev.kind === 'error') li.classList.add('error');
  const summary = summarize(ev);
  li.innerHTML = `<span class="t">${ev.t} ms</span>
                  <span class="k">${ev.kind}</span>
                  <span>${summary}</span>`;
  logBody.prepend(li);
  while (logBody.children.length > 500) logBody.lastChild.remove();
}

function summarize(ev) {
  const { t, kind, ...rest } = ev;
  return Object.entries(rest)
    .map(([k, v]) => `${k}=${typeof v === 'string' ? v : JSON.stringify(v)}`)
    .join(' ');
}

function updateContactor(ev) {
  const el = $('#ph' + ev.phase);
  if (!el) return;
  el.classList.toggle('closed', !!ev.closed);
}

function updateFromSnapshot(ev) {
  $('#b-state').textContent = 'STATE ' + ev.state;
  $('#b-mode').textContent = 'Mode ' + ev.mode;
  $('#mode').value = ev.mode;
  $('#plug').checked = ev.ev_connected;
  $('#pilot').value = ev.pilot_mv;
  $('#pilot-out').value = `${ev.pilot_mv} mV`;
  $('#ichg').textContent = ev.charge_current_a.toFixed(1) + ' A';
  $('#ichg-sub').textContent = `target ${ev.max_current_a} A · mains ${ev.max_mains_a} A`;
  $('#pwm').textContent = ev.pwm_duty + '%';
  ev.contactor.forEach((c, i) => {
    const el = $('#ph' + (i + 1));
    if (el) el.classList.toggle('closed', !!c);
  });
  updateErrors(ev.errors);
}

function updateErrors(flags) {
  const box = $('#flags');
  box.innerHTML = '';
  if (!flags.length) {
    box.textContent = 'none';
    $('#b-error').classList.add('hidden');
    return;
  }
  $('#b-error').classList.remove('hidden');
  $('#b-error').textContent = flags.join(' · ');
  for (const f of flags) {
    const span = document.createElement('span');
    span.className = 'flag';
    span.textContent = f;
    box.appendChild(span);
  }
}

let errorSet = new Set();
function applyError(ev) {
  if (ev.set) errorSet.add(ev.flag); else errorSet.delete(ev.flag);
  updateErrors([...errorSet]);
}

function upsertMqtt(ev) {
  let tr = mqttRows.get(ev.topic);
  if (!tr) {
    tr = document.createElement('tr');
    tr.innerHTML = `<td></td><td></td>`;
    mqttBody.prepend(tr);
    mqttRows.set(ev.topic, tr);
  }
  tr.children[0].textContent = ev.topic;
  tr.children[1].textContent = ev.payload;
}

function dispatch(ev) {
  $('#b-time').textContent = `t=${ev.t} ms`;
  switch (ev.kind) {
    case 'snapshot': updateFromSnapshot(ev); break;
    case 'state':    addTransition(ev); $('#b-state').textContent = 'STATE ' + ev.to; break;
    case 'contactor': updateContactor(ev); break;
    case 'error':    applyError(ev); break;
    case 'mqtt_publish': upsertMqtt(ev); break;
    case 'pwm':
      $('#pwm').textContent = ev.duty + '%';
      $('#pwm-sub').textContent = ev.advertised_a != null
        ? `advertised ${ev.advertised_a.toFixed(1)} A`
        : 'advertised —';
      break;
    case 'reset':
      transitions.innerHTML = '';
      mqttBody.innerHTML = '';
      mqttRows.clear();
      errorSet.clear();
      break;
  }
  addLog(ev);
}

$('#clear-log').addEventListener('click', () => { logBody.innerHTML = ''; });

const es = new EventSource('/events');
es.onmessage = (e) => {
  try { dispatch(JSON.parse(e.data)); } catch { /* keepalive */ }
};
es.onerror = () => console.warn('SSE connection lost, browser will retry');
