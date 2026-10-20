import assert from 'node:assert/strict';
import test from 'node:test';
import {DutyKeys} from '../docs/keyboard.js';

const tick = () => new Promise(resolve => setImmediate(resolve));
const target = (tagName = 'BUTTON', type = '', editable = false) => ({tagName, type,
  isContentEditable: editable, closest: () => ['TEXTAREA', 'SELECT'].includes(tagName) || editable});
function event(key, options = {}) {
  return {key, code: key === '-' ? 'Minus' : 'Equal', target: target(),
    preventDefault() { this.prevented = true; }, ...options};
}
function setup(value = 240) {
  const sent = [], errors = [];
  const keys = new DutyKeys({send: async duty => { sent.push(duty); }, value: () => value,
    onError: error => errors.push(error)});
  keys.enabled = true;
  return {keys, sent, errors};
}

test('held plus and minus send signed target duty and release sends zero', async () => {
  const {keys, sent} = setup(275);
  const plus = event('+');
  keys.keyDown(plus); await tick();
  assert.equal(plus.prevented, true);
  assert.deepEqual(sent, [275]);
  keys.keyUp(event('+')); await tick();
  assert.deepEqual(sent, [275, 0]);
  keys.keyDown(event('-')); await tick();
  assert.deepEqual(sent, [275, 0, -275]);
  await keys.release();
  assert.deepEqual(sent, [275, 0, -275, 0]);
});

test('simultaneous directions cancel, releasing one restores the remaining direction', async () => {
  const {keys, sent} = setup(180);
  keys.keyDown(event('+')); await tick();
  keys.keyDown(event('-')); await tick();
  keys.keyUp(event('+')); await tick();
  keys.keyUp(event('-')); await tick();
  assert.deepEqual(sent, [180, 0, -180, 0]);
});

test('repeated keydown events do not flood commands and keypad keys are supported', async () => {
  const {keys, sent} = setup();
  keys.keyDown(event('+', {code: 'NumpadAdd'})); await tick();
  for (let i = 0; i < 20; i++) keys.keyDown(event('+', {code: 'NumpadAdd', repeat: true}));
  await tick();
  assert.deepEqual(sent, [240]);
  keys.keyUp(event('+', {code: 'NumpadAdd'})); await tick();
  keys.keyDown(event('-', {code: 'NumpadSubtract'})); await tick();
  keys.keyUp(event('-', {code: 'NumpadSubtract'})); await tick();
  assert.deepEqual(sent, [240, 0, -240, 0]);
});

test('typing fields and modifiers do not drive the motor; checkbox focus permits key control', async () => {
  const {keys, sent} = setup();
  for (const field of [target('INPUT', 'number'), target('INPUT', 'text'), target('TEXTAREA'), target('SELECT'), target('DIV', '', true)]) {
    keys.keyDown(event('+', {target: field}));
  }
  for (const modifier of ['ctrlKey', 'metaKey', 'altKey']) keys.keyDown(event('+', {[modifier]: true}));
  keys.keyDown(event('a')); await tick();
  assert.deepEqual(sent, []);
  keys.keyDown(event('+', {target: target('INPUT', 'checkbox')})); await tick();
  assert.deepEqual(sent, [240]);
  // A focus change must never hide the matching release.
  keys.keyUp(event('=', {target: target('INPUT', 'number')})); await tick();
  assert.deepEqual(sent, [240, 0]);
});

test('release waits for an in-flight press and sends zero without overlapping writes', async () => {
  let completePress, active = 0, maxActive = 0;
  const sent = [];
  const keys = new DutyKeys({value: () => 300, onError(error) { throw error; },
    async send(duty) {
      active++; maxActive = Math.max(maxActive, active); sent.push(duty);
      if (duty) await new Promise(resolve => { completePress = resolve; });
      active--;
    }});
  keys.enabled = true;
  keys.keyDown(event('+')); await tick();
  let stopped = false;
  keys.keyUp(event('+'));
  const release = keys.release().then(() => { stopped = true; });
  await tick();
  assert.equal(stopped, false);
  assert.deepEqual(sent, [300]);
  completePress(); await release;
  assert.deepEqual(sent, [300, 0]);
  assert.equal(maxActive, 1);
  assert.equal(stopped, true);
});

test('a release queued in the press completion microtask cannot leave duty latched', async () => {
  let complete;
  const pending = new Promise(resolve => { complete = resolve; });
  const sent = [];
  const keys = new DutyKeys({value: () => 200, onError(error) { throw error; },
    send(duty) { sent.push(duty); return duty ? pending : Promise.resolve(); }});
  keys.enabled = true;
  keys.keyDown(event('+')); await tick();
  pending.then(() => keys.keyUp(event('+')));
  complete(); await tick();
  assert.deepEqual(sent, [200, 0]);
  assert.equal(keys.sent, 0);
});

test('disable disarms immediately, flushes zero after a pending press and blocks new presses', async () => {
  let complete;
  const sent = [];
  const keys = new DutyKeys({value: () => 500, onError(error) { throw error; },
    send(duty) { sent.push(duty); return duty ? new Promise(resolve => { complete = resolve; }) : Promise.resolve(); }});
  keys.enabled = true;
  keys.keyDown(event('-')); await tick();
  const disabled = keys.disable();
  assert.equal(keys.enabled, false);
  keys.keyDown(event('+'));
  complete(); await disabled;
  assert.deepEqual(sent, [-500, 0]);
  keys.keyDown(event('-')); await tick();
  assert.deepEqual(sent, [-500, 0]);
});

test('a failed press disarms key control and attempts one zero command', async () => {
  const failure = new Error('Disconnected'), sent = [], errors = [];
  const keys = new DutyKeys({value: () => 100, onError: error => errors.push(error),
    async send(duty) { sent.push(duty); if (duty) throw failure; }});
  keys.enabled = true;
  keys.keyDown(event('+')); await tick();
  assert.deepEqual(sent, [100, 0]);
  assert.deepEqual(errors, [failure]);
  assert.equal(keys.enabled, false);
  assert.equal(keys.held.size, 0);
});

test('invalid target values disarm and stop an existing held command', async () => {
  let value = 250;
  const sent = [], errors = [];
  const keys = new DutyKeys({value: () => value, onError: e => errors.push(e), send: async duty => sent.push(duty)});
  keys.enabled = true;
  keys.keyDown(event('+')); await tick();
  value = NaN;
  keys.keyDown(event('+', {code: 'NumpadAdd'})); await tick();
  assert.deepEqual(sent, [250, 0]);
  assert.equal(keys.enabled, false);
  assert.match(errors[0].message, /0 to 799/);
});

test('keyboard permits duty 799 in each direction and rejects 800 before sending', async () => {
  const allowed = setup(799);
  allowed.keys.keyDown(event('+')); await tick();
  allowed.keys.keyUp(event('+')); await tick();
  allowed.keys.keyDown(event('-')); await tick();
  await allowed.keys.release();
  assert.deepEqual(allowed.sent, [799, 0, -799, 0]);
  for (const value of [800, -1, -800, 1.5, Infinity]) {
    const invalid = setup(value);
    invalid.keys.keyDown(event('+')); await tick();
    assert.deepEqual(invalid.sent, [], String(value));
    assert.equal(invalid.keys.enabled, false);
    assert.match(invalid.errors[0].message, /0 to 799/);
  }
});
