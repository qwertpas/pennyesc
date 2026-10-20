import {DUTY_LIMIT} from './protocol.js';

function direction(event) {
  if (['+', '='].includes(event.key) || event.code === 'NumpadAdd') return 1;
  if (event.key === '-' || event.code === 'NumpadSubtract') return -1;
  return 0;
}

function editing(target) {
  return target?.isContentEditable || target?.closest?.('textarea,select,[contenteditable="true"]') ||
    (target?.tagName === 'INPUT' && !['checkbox', 'button', 'submit'].includes(target.type));
}

export class DutyKeys {
  constructor({send, value, onError}) {
    Object.assign(this, {send, value, onError});
    this.enabled = false;
    this.held = new Map();
    this.desired = 0;
    this.sent = 0;
    this.task = null;
  }

  keyDown(event) {
    const sign = direction(event);
    if (!this.enabled || !sign || event.ctrlKey || event.metaKey || event.altKey || editing(event.target)) return;
    event.preventDefault();
    if (event.repeat) return;
    this.held.set(event.code || event.key, sign);
    this.update();
  }

  keyUp(event) {
    const key = event.code || event.key;
    if (!this.held.has(key)) return;
    event.preventDefault();
    this.held.delete(key);
    this.update();
  }

  update() {
    const signs = new Set(this.held.values());
    if (signs.size !== 1) { this.set(0); return; }
    const value = this.value();
    if (!Number.isInteger(value) || value < 0 || value > DUTY_LIMIT) {
      this.enabled = false; this.held.clear(); this.set(0);
      this.onError(new Error(`Choose a duty from 0 to ${DUTY_LIMIT}`));
      return;
    }
    this.set([...signs][0] * value);
  }

  set(value) {
    this.desired = value;
    if (!this.task && this.sent !== this.desired) {
      this.task = Promise.resolve().then(() => this.run());
    }
  }

  async run() {
    try {
      while (this.sent !== this.desired) {
        const value = this.desired;
        try { await this.send(value); this.sent = value; }
        catch (error) {
          this.enabled = false;
          this.held.clear(); this.desired = 0;
          if (value !== 0) {
            try { await this.send(0); } catch (stopError) { this.onError(stopError); }
          }
          this.sent = 0;
          this.onError(error);
        }
      }
    } finally { this.task = null; }
  }

  async release() {
    this.held.clear(); this.set(0);
    await this.task;
  }

  async disable() {
    this.enabled = false;
    await this.release();
  }
}
