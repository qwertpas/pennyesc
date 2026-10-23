import {SerialConnection, TimeoutError} from './serial.js';
import {DUTY_LIMIT, PennyClient} from './protocol.js';
import {captureCalibration, reduceCapturePoints, solveCapture, uploadCalibration,
  readCalibration, captureRows, calibrationPlots} from './calibration.js';
import {flashBridge, flashEsc} from './flash.js';
import {LiveData, Telemetry} from './telemetry.js';
import {defaultControl, controlText, checkResult, runCommand} from './commands.js';
import {Plot} from './plots.js';
import {DutyKeys} from './keyboard.js';

const $ = id => document.getElementById(id);
const connection = new SerialConnection();
const live = new LiveData();
let client, telemetry, manifest, session, raw = [], reduced = [], solved;
let control = defaultControl(), busy = false, paused = false, dirty = true;
let currentMode = 'duty';
let connectedBoard;
const keys = new DutyKeys({
  value: () => Math.abs(Number($('target').value)),
  async send(duty) {
    if (!client || !connection.connected) return;
    const status = await client.setDuty(duty);
    control = {...defaultControl(), kf: duty, clip: control.clip};
    updateControl(); showStatus(status); log(`Duty ${duty}`);
  },
  onError(error) { $('key-control').checked = false; message(error.message, true); log(error.message); state(); },
});
const plots = {
  magnetic: new Plot($('magnetic'), 'Magnetic field', 'Time (s)', 'Raw field'),
  position: new Plot($('position'), 'Position', 'Time (s)', 'rad'),
  velocity: new Plot($('velocity'), 'Velocity', 'Time (s)', 'rpm'),
  xy: new Plot($('xy'), 'Magnetic field XY', 'X', 'Y', true, true),
  field: new Plot($('field-step'), 'Magnetic field vs step', 'Step', 'Raw field', true),
  residual: new Plot($('residual'), 'Residual vs step', 'Step', 'Error (°)', true),
};

for (const id of ['address', 'verify-address']) {
  $(id).replaceChildren(...Array.from({length: 16}, (_, i) => new Option(String(i), i)));
  $(id).value = '1';
}

function log(text, target = 'log') {
  const element = $(target);
  const lines = (element.textContent + String(text) + '\n').split('\n');
  element.textContent = lines.slice(-300).join('\n');
  element.scrollTop = element.scrollHeight;
}

function message(text, error = false) {
  $('message').textContent = text;
  $('message').classList.toggle('error', error);
  $('message').hidden = !text;
}

function state() {
  $('connect').textContent = connection.connected ? 'Disconnect' : 'Connect';
  $('connection').textContent = connection.connected ? 'Connected' : 'Disconnected';
  $('connection').classList.toggle('connected', connection.connected);
  $('connect').disabled = busy || !manifest || !('serial' in navigator);
  document.querySelectorAll('[data-connected]').forEach(button => { button.disabled = busy || !connection.connected; });
  $('address').disabled = busy;
  $('board').disabled = busy;
  $('send').disabled = busy;
  for (const id of ['mode','kp','kd','kv','kf','clip','target','target-unit','rx','tx','gnd','default-pins','esc-image','verify-address','firmware-file']) $(id).disabled = busy;
  const unused = $('mode').value === 'duty' ? ['kp','kd','kv','kf'] : $('mode').value === 'pos' ? ['kv'] : ['kp'];
  for (const id of ['kp','kd','kv','kf']) {
    $(id).disabled = busy || unused.includes(id);
    $(id).closest('label').classList.toggle('inactive', unused.includes(id));
  }
  $('key-control').disabled = busy || !connection.connected || $('mode').value !== 'duty';
  $('key-control').closest('label').classList.toggle('inactive', $('key-control').disabled);
  keys.enabled = $('key-control').checked && !$('key-control').disabled;
  $('flash-bridge').disabled = busy || !manifest || !('serial' in navigator);
  $('export-session').disabled = !session || busy;
  $('export-calibration').disabled = !session || busy;
  $('pause').textContent = paused ? 'Resume' : 'Pause';
}

function setRate(value) {
  $('rate').value = `${Math.round(value)} samples/s`;
  $('rate').classList.toggle('slow', value > 0 && value < 100);
}

function statusText(status) {
  return `result=${status.result} mode=${status.mode} flags=0x${status.flags.toString(16)} faults=0x${status.faults.toString(16)} raw=(${status.x},${status.y},${status.z}) angle=${status.angle_turn16} position=${status.position_turn32} velocity=${status.velocity_turn32_per_s} duty=${status.duty} mct_faults=${status.mct_fault_count}`;
}

function showStatus(status) {
  $('motor').textContent = `${status.calibrated ? 'Calibrated' : 'Uncalibrated'}${status.faults ? ` · faults 0x${status.faults.toString(16)}` : ''}`;
}

function startTelemetry() {
  if (!client || !connection.connected || paused || busy) return;
  telemetry = new Telemetry(client, {
    onSample(status, now) { live.add(status, now); dirty = true; }, onRate: setRate,
    onError(error) { paused = true; message(error.message, true); log(error.message); state(); },
  });
  telemetry.start();
}

async function operation(action, {resume = true} = {}) {
  if (busy) return;
  busy = true; state();
  try { await keys.release(); await telemetry?.stop(); message(''); return await action(); }
  catch (error) { if (error.name !== 'NotFoundError' && error.name !== 'AbortError') { message(error.message, true); log(error.message); } }
  finally { busy = false; state(); if (resume) startTelemetry(); }
}

function updateControl() {
  for (const key of ['kp','kd','kv','kf','clip']) $(key).value = control[key];
}

function setMode(mode) {
  if (mode !== currentMode) $('target').value = '0';
  currentMode = mode;
  $('mode').value = mode;
  const units = mode === 'vel' ? [['vel','rad/s'], ['rpm','rpm']] : mode === 'pos' ? [['pos','rad']] : [['duty','duty']];
  $('target-unit').replaceChildren(...units.map(([value, label]) => new Option(label, value)));
  $('target-unit').hidden = mode !== 'vel';
  $('target-unit-text').hidden = mode === 'vel';
  $('target-unit-text').textContent = units[0][1];
  $('target-label').textContent = {duty: 'Duty', pos: 'Position', vel: 'Velocity'}[mode];
  $('target').step = mode === 'duty' ? '1' : 'any';
  if (mode === 'duty') { $('target').min = -DUTY_LIMIT; $('target').max = DUTY_LIMIT; }
  else { $('target').removeAttribute('min'); $('target').removeAttribute('max'); $('key-control').checked = false; }
}

async function command(text) {
  log(`> ${text}`);
  const result = await runCommand(client, text, control);
  if (result.control) { control = result.control; updateControl(); }
  if (result.mode) {
    setMode(result.mode);
    if (result.mode === 'pos') $('target').value = result.status.position_rad;
    if (/^duty\s/i.test(text)) $('target').value = result.control.kf;
  }
  if (result.status) { showStatus(result.status); log(statusText(result.status)); }
  if (result.message) log(result.message);
}

function board() { return manifest?.bridges.find(item => item.id === $('board').value); }

function selectPins(info = board()) {
  if (!info) return;
  for (const [id, values] of [['rx', info.inputPins], ['tx', info.outputPins], ['gnd', [-1, ...info.groundPins]]]) {
    $(id).replaceChildren(...values.map(pin => new Option(pin === -1 ? 'GND' : `GPIO ${pin}`, pin)));
    $(id).value = String(info[id]);
  }
}

async function readPins() {
  let response;
  try {
    response = await connection.shell('pins', {expected: line => line.startsWith('# pins ')});
  } catch (error) {
    if (error.message === 'err') throw new Error('Update bridge firmware: Firmware → select board → Flash bridge.');
    throw error;
  }
  const fields = Object.fromEntries([...response.matchAll(/(\w+)=([^\s]+)/g)].map(m => [m[1], m[2]]));
  connectedBoard = manifest.bridges.find(item => item.id === fields.board);
  if (!connectedBoard) throw new Error(`Unknown bridge board: ${fields.board}`);
  $('board').value = connectedBoard.id;
  selectPins({...connectedBoard, rx: Number(fields.rx), tx: Number(fields.tx), gnd: Number(fields.gnd)});
}

async function scan() {
  const found = [];
  for (let address = 0; address < 16; address++) {
    $('motor').textContent = `Scanning ${address}…`;
    const probe = new PennyClient(connection, address);
    try { checkResult(await probe.status(35)); found.push(address); }
    catch (error) { if (!(error instanceof TimeoutError)) throw error; }
  }
  const address = found.includes(Number($('address').value)) ? Number($('address').value) : found[0];
  for (const option of $('address').options) option.textContent = `${option.value}${found.includes(Number(option.value)) ? ' •' : ''}`;
  if (address === undefined) {
    client = new PennyClient(connection, Number($('address').value));
    paused = true; $('motor').textContent = 'No ESC'; throw new Error('No ESC found');
  }
  $('address').value = address;
  client = new PennyClient(connection, address);
  paused = false;
  showStatus(checkResult(await client.status()));
  log(`ESC addresses: ${found.join(', ')}`);
}

$('connect').onclick = () => operation(async () => {
  if (connection.connected) {
    try { await connection.mode('off'); }
    finally { await connection.disconnect(); client = null; connectedBoard = null; }
    $('motor').textContent = '—'; setRate(0);
  } else {
    try {
      await connection.connect();
      await readPins();
      await connection.mode('app');
      paused = false;
    } catch (error) {
      await connection.disconnect();
      throw error;
    }
    await scan();
  }
});
connection.addEventListener('disconnect', event => {
  $('key-control').checked = false;
  void keys.disable();
  client = null; connectedBoard = null; paused = true; setRate(0);
  message(event.error?.message ?? 'Serial port disconnected', true);
  void operation(() => connection.disconnect(), {resume: false});
  state();
});
$('scan').onclick = () => operation(scan);
$('address').onchange = () => operation(async () => {
  if (!connection.connected) return;
  client = new PennyClient(connection, Number($('address').value));
  showStatus(checkResult(await client.status()));
});
$('brake').onclick = () => operation(() => command('brake'));
$('zero').onclick = () => operation(() => command('zero'));
$('mode').onchange = () => {
  const mode = $('mode').value;
  $('mode').value = currentMode;
  void operation(() => command(`mode ${mode}`));
};
$('set-target').onclick = () => {
  if ($('target').reportValidity()) void operation(() => command(`${$('target-unit').value} ${$('target').value}`));
};
$('control-form').onsubmit = event => {
  event.preventDefault();
  void operation(async () => {
    const next = Object.fromEntries(['kp','kd','kv','kf','clip'].map(key => [key, Number($(key).value)]));
    if ($('mode').value === 'duty') Object.assign(next, {kp: 0, kd: 0, kv: 0, kf: control.kf});
    if ($('mode').value === 'pos') next.kv = 0;
    if ($('mode').value === 'vel') next.kp = 0;
    showStatus(checkResult(await client.setControl(next)));
    control = next; log(controlText(control));
  });
};
const history = []; let historyIndex = 0;
$('command-form').onsubmit = event => {
  event.preventDefault();
  const text = $('command').value.trim();
  if (!text || busy) return;
  history.push(text); historyIndex = history.length;
  $('command').value = '';
  void operation(() => command(text));
};
$('command').onkeydown = event => {
  if (!['ArrowUp','ArrowDown'].includes(event.key)) return;
  event.preventDefault();
  historyIndex = Math.max(0, Math.min(history.length, historyIndex + (event.key === 'ArrowUp' ? -1 : 1)));
  $('command').value = history[historyIndex] ?? '';
};
$('clear-log').onclick = () => { $('log').textContent = ''; };
$('key-control').onchange = () => {
  if (!$('key-control').checked) void keys.disable();
  state();
};
window.addEventListener('keydown', event => keys.keyDown(event));
window.addEventListener('keyup', event => keys.keyUp(event));
window.addEventListener('blur', () => { void keys.release(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) void keys.release(); });
$('target').addEventListener('input', () => { void keys.release(); });
$('pause').onclick = async () => {
  if (busy) return;
  paused = !paused;
  if (paused) await telemetry?.stop(); else startTelemetry();
  state();
};
$('clear-live').onclick = () => { live.clear(); dirty = true; };
$('window').onchange = () => { dirty = true; };
$('show-magnetic').onchange = () => {
  $('magnetic').hidden = !$('show-magnetic').checked;
  document.querySelector('.live-plots').classList.toggle('with-magnetic', $('show-magnetic').checked);
  for (const name of ['position','velocity','magnetic']) plots[name].resize();
  dirty = true;
};

for (const button of document.querySelectorAll('[role=tab]')) {
  button.onclick = () => {
    for (const tab of document.querySelectorAll('[role=tab]')) {
      const selected = button === tab;
      tab.setAttribute('aria-selected', String(selected));
      $(tab.getAttribute('aria-controls')).hidden = !selected;
    }
    for (const plot of Object.values(plots)) plot.resize();
    dirty = true;
  };
  button.onkeydown = event => {
    if (!['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) return;
    event.preventDefault();
    const tabs = [...document.querySelectorAll('[role=tab]')];
    const index = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1 :
      (tabs.indexOf(button) + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length;
    tabs[index].click(); tabs[index].focus();
  };
}

function drawLive() {
  const data = live.data(Number($('window').value));
  plots.magnetic.set(['X','Y','Z'].map((name, i) => ({name, x: data[0], y: data[i + 1], color: ['#d65053','#198568','#4382cd'][i]})));
  plots.position.set([{name: 'Position', x: data[0], y: data[4], color: '#8052b4'}]);
  plots.velocity.set([{name: 'Velocity', x: data[0], y: data[5], color: '#c17c16'}]);
  const s = live.latest;
  if (s) {
    const field = $('show-magnetic').checked ? `X ${s.x} · Y ${s.y} · Z ${s.z} · ` : '';
    $('values').value = `${field}${s.position_rad.toFixed(3)} rad · ${s.velocity_rpm.toFixed(1)} rpm · duty ${s.duty}`;
    showStatus(s);
  } else $('values').value = 'Position — · Velocity —';
}

function drawCalibration() {
  const data = calibrationPlots(raw, reduced, solved);
  plots.xy.set(data.xy); plots.field.set(data.field); plots.residual.set(data.residual);
}

function updateSession() {
  if (!session) { $('session-state').value = 'No session'; $('session-data').textContent = ''; state(); return; }
  $('session-state').value = session.status;
  $('session-data').textContent = JSON.stringify({...session, static: session.static ? {
    ...session.static, raw_points: `${raw.length} points`, reduced_points: `${reduced.length} points`, angle_lut: `${solved?.angle_lut?.length ?? 0} values`,
  } : undefined}, null, 2);
  state();
}

$('calibrate').onclick = () => operation(async () => {
  session = {mode: 'static', port: connection.port.getInfo(), address: client.address, started_at: new Date().toISOString(), status: 'Capturing'};
  control = {...defaultControl(), clip: control.clip};
  setMode('duty'); updateControl();
  raw = []; reduced = []; solved = undefined;
  $('tab-calibration').click();
  try {
    raw = await captureCalibration(client, {
      onProgress(progress) {
        $('cal-progress').value = progress.done * 60 / progress.total;
        $('cal-progress-text').value = `${progress.sweep_dir ? 'Reverse' : 'Forward'} · rotation ${progress.rotation} · ${progress.done}/${progress.total}`;
      },
      onSample(point) {
        const index = raw.findIndex(value => value.index === point.index);
        if (index < 0) raw.push(point); else raw[index] = point;
        dirty = true;
      },
    });
    reduced = reduceCapturePoints(raw);
    session.status = 'Solving'; updateSession();
    solved = solveCapture(reduced); dirty = true;
    session.static = {raw_points: captureRows(raw, true), reduced_points: captureRows(reduced), ...solved, blob: undefined, blob_size: solved.blob.length};
    session.status = 'Uploading';
    await uploadCalibration(client, solved, {onProgress({stage, done, total}) {
      $('cal-progress').value = stage === 'verify' ? 95 + done * 5 / total : 60 + done * 35 / total;
      $('cal-progress-text').value = stage === 'verify' ? 'Verifying…' : `Upload ${done}/${total}`;
    }});
    session.status = 'Calibrated';
    $('cal-state').textContent = `Fit ${solved.fit_max_error_deg.toFixed(2)}° · sweep ${solved.sweep_delta_deg.toFixed(2)}° · sign ${solved.forward_angle_sign > 0 ? '+' : ''}${solved.forward_angle_sign}`;
    $('cal-progress').value = 100;
    $('cal-progress-text').value = 'Verified';
    log(`Calibration verified · CRC32 ${solved.blob_crc32.toString(16).padStart(8,'0')}`);
  } catch (error) {
    session.status = 'Failed'; session.error = error.message;
    session.static ??= {raw_points: captureRows(raw, true), reduced_points: captureRows(reduced)};
    $('cal-state').textContent = error.message;
    throw error;
  } finally {
    session.ended_at = new Date().toISOString(); updateSession(); dirty = true;
  }
});
$('check-motor').onclick = () => operation(async () => {
  const status = checkResult(await client.status());
  const info = await client.calInfo();
  showStatus(status); log(statusText(status)); log(JSON.stringify(info));
  $('cal-state').textContent = info.valid ? `CRC32 ${info.blob_crc32.toString(16).padStart(8,'0')} · ${info.blob_size} bytes` : 'Uncalibrated';
});
$('read-calibration').onclick = () => operation(async () => {
  const result = await readCalibration(client);
  log(JSON.stringify(result, (key, value) => key === 'blob' || key === 'angle_lut' ? undefined : value));
  if (!result.valid) throw new Error('No valid calibration');
  download(result.blob, 'calibration.bin', 'application/octet-stream');
  $('cal-state').textContent = `Verified CRC32 ${result.blob_crc32.toString(16).padStart(8,'0')}`;
});
$('clear-calibration').onclick = () => {
  if (busy) return;
  raw = []; reduced = []; solved = undefined; session = undefined;
  $('cal-progress').value = 0; $('cal-progress-text').value = ''; $('cal-state').textContent = '—';
  updateSession(); dirty = true;
};

$('board').onchange = () => { selectPins(); localStorage.setItem('penny-board', $('board').value); };
$('default-pins').onclick = () => selectPins();
$('apply-pins').onclick = () => operation(async () => {
  if (connectedBoard?.id !== board().id) throw new Error('Select the connected board');
  const pins = ['rx','tx','gnd'].map(id => Number($(id).value));
  if (pins[0] === pins[1] || (pins[2] >= 0 && pins.slice(0,2).includes(pins[2]))) throw new Error('Choose a different GPIO for each pin');
  const response = await connection.shell(`pins ${pins.join(' ')}`, {expected: line => line.startsWith('# pins ')});
  if (response.includes('error=')) throw new Error(response);
  log(response);
  await readPins();
  message('Pins saved');
});

const flashProgress = (stage, percent) => {
  $('flash-progress').value = percent;
  $('flash-state').value = `${stage} ${Math.round(percent)}%`;
};
const flashLog = text => log(text, 'flash-log');
$('flash-bridge').onclick = () => operation(async () => {
  const target = board();
  client = null;
  $('flash-log').textContent = '';
  await flashBridge(connection, target, {progress: flashProgress, onLog: flashLog});
  connectedBoard = null;
  $('flash-state').value = 'Bridge flashed';
  message('Bridge flashed. Connect to use it.');
}, {resume: false});
$('esc-image').onchange = () => { $('firmware-file').hidden = $('esc-image').value !== 'file'; };
async function flashPenny(recover) {
  let image;
  let verifyAddress = Number($('verify-address').value);
  if ($('esc-image').value === 'file') {
    const file = $('firmware-file').files[0];
    if (!file) throw new Error('Choose a firmware file');
    image = new Uint8Array(await file.arrayBuffer());
  } else {
    const response = await fetch(new URL(manifest.pennyesc.path, new URL('firmware/manifest.json', location.href)));
    if (!response.ok) throw new Error('Firmware download failed');
    image = new Uint8Array(await response.arrayBuffer());
  }
  $('flash-log').textContent = '';
  try {
    await flashEsc(client, image, {recover, verifyAddress, progress: flashProgress, onLog: flashLog,
      onResetNeeded() { $('flash-state').value = 'Reset PennyESC now'; }});
    await connection.mode('app');
  } catch (error) { paused = true; throw error; }
  $('address').value = verifyAddress;
  client = new PennyClient(connection, verifyAddress);
  $('flash-state').value = 'PennyESC flashed and verified';
  showStatus(checkResult(await client.status()));
}
$('flash-esc').onclick = () => operation(() => flashPenny(false));
$('recover-esc').onclick = () => operation(() => flashPenny(true));

function download(data, name, type) {
  const url = URL.createObjectURL(data instanceof Blob ? data : new Blob([data], {type}));
  const a = document.createElement('a'); a.href = url; a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$('export-live').onclick = () => download(live.csv(), 'telemetry.csv', 'text/csv');

function csv(rows) {
  if (!rows.length) return '';
  const keys = Object.keys(rows[0]);
  return [keys.join(','), ...rows.map(row => keys.map(key => JSON.stringify(row[key] ?? '')).join(','))].join('\n') + '\n';
}

async function exportSession() {
  if (!session) return;
  try {
    const parent = await window.showDirectoryPicker({mode: 'readwrite'});
    const directory = await parent.getDirectoryHandle(`penny-${session.started_at.replace(/[:.]/g,'-')}`, {create: true});
    const files = {'run_metadata.json': JSON.stringify(session, null, 2), 'static_raw.csv': csv(captureRows(raw,true)),
      'static_reduced.csv': csv(captureRows(reduced)), 'telemetry.csv': live.csv()};
    if (solved) {
      files['static_summary.json'] = JSON.stringify({...solved, blob: undefined, blob_size: solved.blob.length}, null, 2);
      files['calibration.bin'] = solved.blob;
    }
    drawLive(); drawCalibration();
    for (const [key, plot] of Object.entries(plots)) files[`${key}.png`] = await plot.png();
    for (const [name, data] of Object.entries(files)) {
      const handle = await directory.getFileHandle(name, {create: true});
      const stream = await handle.createWritable(); await stream.write(data); await stream.close();
    }
    message('Session exported');
  } catch (error) { if (error.name !== 'AbortError') message(error.message, true); }
}
$('export-session').onclick = exportSession;
$('export-calibration').onclick = exportSession;

let lastDraw = 0;
function render(now) {
  if (dirty && now - lastDraw >= 1000 / 30) {
    if (!$('telemetry').hidden) drawLive();
    if (!$('calibration').hidden) drawCalibration();
    dirty = false; lastDraw = now;
  }
  requestAnimationFrame(render);
}

async function initialize() {
  setMode('duty');
  state(); drawLive(); drawCalibration(); updateSession();
  if (!('serial' in navigator)) message('Web Serial is unavailable', true);
  try {
    const response = await fetch('firmware/manifest.json');
    if (!response.ok) throw new Error('Firmware manifest unavailable');
    manifest = await response.json();
    $('board').replaceChildren(...manifest.bridges.map(item => new Option(item.name, item.id)));
    const saved = localStorage.getItem('penny-board');
    if (manifest.bridges.some(item => item.id === saved)) $('board').value = saved;
    $('verify-address').value = manifest.pennyesc.escAddress;
    selectPins(); state();
  } catch (error) { message(error.message, true); }
  requestAnimationFrame(render);
}
void initialize();
