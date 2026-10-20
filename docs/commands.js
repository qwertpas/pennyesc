import {round, checkDuty, checkClip} from './protocol.js';

export const defaultControl = () => ({kp: 0, kd: 0, kv: 0, kf: 0, clip: 150});
export const controlText = c => `kp ${c.kp} | kd ${c.kd} | kv ${c.kv} | kf ${c.kf} | clip ${c.clip}`;
export const help = 'status | control | kp/kd/kv <gain> | kf <duty> | clip <duty> | pos <rad> | vel <rad/s> | rpm <rpm> | duty <value> | mode pos|vel|duty [duty] | zero | brake | advance <deg>';

function numeric(text, integer = false) {
  let n;
  if (integer && typeof text === 'string') {
    if (!/^[+-]?(?:0[xX][\da-fA-F]+|0[bB][01]+|0[oO][0-7]+|0+|[1-9]\d*)$/.test(text)) throw new Error('Invalid integer');
    n = (text.startsWith('-') ? -1 : 1) * Number(text.replace(/^[+-]/, ''));
  } else n = Number(text);
  if (text === undefined || !Number.isFinite(n) || (integer && !Number.isInteger(n))) throw new Error('Invalid number');
  return n;
}

export function checkResult(status) {
  const names = ['ok', 'bad state', 'bad argument', 'not calibrated', 'busy', 'range', 'flash', 'CRC'];
  if (status.result !== 0) throw new Error(names[status.result] ?? `Result ${status.result}`);
  return status;
}

export async function runCommand(client, text, control) {
  const parts = text.trim().split(/\s+/);
  const command = parts[0].toLowerCase();
  const value = parts[1];
  if (command === 'help') return {message: help + '\n' + controlText(control)};
  if (command === 'control') return {message: controlText(control)};
  if (!client) throw new Error('Connect first');
  const one = () => { if (parts.length !== 2) throw new Error(`Usage: ${command} <value>`); };
  let next;
  let status;
  let mode;
  switch (command) {
    case 'status': status = await client.status(); break;
    case 'brake':
      status = checkResult(await client.brake());
      return {status, control: {...defaultControl(), clip: control.clip}, mode: 'duty'};
    case 'zero': case 'zero_position': case 'zeropos':
      if (parts.length !== 1) throw new Error('Usage: zero');
      status = await client.zero(); break;
    case 'kp': case 'kd': case 'kv': case 'kf': case 'clip':
      one(); next = {...control, [command]: numeric(value, ['kf', 'clip'].includes(command))}; break;
    case 'duty':
      one(); next = {...defaultControl(), kf: checkDuty(numeric(value, true)), clip: control.clip}; mode = 'duty'; break;
    case 'mode':
      mode = value?.toLowerCase();
      if (parts.length < 2 || parts.length > 3 || !['pos','vel','duty'].includes(mode) || (parts.length === 3 && mode !== 'duty')) throw new Error('Usage: mode pos|vel|duty [duty]');
      if (mode === 'pos') {
        status = checkResult(await client.brake());
        checkResult(await client.setVelocity(0));
        checkResult(await client.setPosition(status.position_turn32));
        next = {kp: 100, kd: 0.1, kv: 0, kf: 0, clip: 150};
      } else if (mode === 'vel') {
        checkResult(await client.setVelocity(0));
        next = {kp: 0, kd: 0.1, kv: 0.06, kf: 0, clip: 150};
      } else {
        next = {...defaultControl(), kf: parts.length === 3 ? checkDuty(numeric(parts[2], true)) : 0, clip: control.clip};
      }
      break;
    case 'advance': one(); status = await client.setAdvance(numeric(value, true)); break;
    case 'pos': case 'position': {
      one(); const n = numeric(value);
      const target = round(n * (65536 / (2 * Math.PI)));
      if (target < -2147483648 || target > 2147483647) throw new Error('Position out of range');
      await client.sendPosition(target);
      return {message: `Position sent ${n} rad`};
    }
    case 'vel': case 'rpm': {
      one(); const n = numeric(value);
      const target = round(command === 'rpm' ? n * 65536 / 60 : n * (65536 / (2 * Math.PI)));
      if (target < -2147483648 || target > 2147483647) throw new Error('Velocity out of range');
      status = await client.setVelocity(target); break;
    }
    default: throw new Error('Unknown command');
  }
  if (next) {
    checkDuty(next.kf); checkClip(next.clip);
    status = await client.setControl(next);
  }
  checkResult(status);
  return {status, control: next, mode, message: next ? controlText(next) : undefined};
}
