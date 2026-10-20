export class LiveData {
  constructor(capacity = 12000) {
    if (!Number.isInteger(capacity) || capacity < 1) throw new Error('Sample capacity must be a positive integer');
    this.capacity = capacity;
    this.rows = Array.from({length: 6}, () => new Float64Array(capacity));
    this.clear();
  }

  clear() {
    this.count = 0;
    this.total = 0;
    this.next = 0;
    this.started = null;
    this.latest = null;
  }

  add(status, now) {
    this.started ??= now;
    const values = [(now - this.started) / 1000, status.x, status.y, status.z,
      status.position_rad, status.velocity_rpm];
    values.forEach((value, i) => { this.rows[i][this.next] = value; });
    this.next = (this.next + 1) % this.capacity;
    this.count = Math.min(this.count + 1, this.capacity);
    this.total++;
    this.latest = status;
  }

  data(seconds = Infinity) {
    const out = this.rows.map(() => []);
    if (!this.count) return out;
    const last = this.rows[0][(this.next + this.capacity - 1) % this.capacity];
    for (let i = 0; i < this.count; i++) {
      const index = (this.next - this.count + i + this.capacity) % this.capacity;
      if (this.rows[0][index] < last - seconds) continue;
      this.rows.forEach((row, j) => out[j].push(row[index]));
    }
    return out;
  }

  csv() {
    const data = this.data();
    return ['time_s,x,y,z,position_rad,velocity_rpm',
      ...data[0].map((_, i) => data.map(row => row[i]).join(','))].join('\n') + '\n';
  }
}

export class Telemetry {
  constructor(client, {onSample, onRate, onError, rate = 200}) {
    if (!Number.isFinite(rate) || rate <= 0) throw new Error('Sample rate must be positive');
    this.client = client;
    this.onSample = onSample;
    this.onRate = onRate;
    this.onError = onError;
    this.interval = 1000 / rate;
    this.running = false;
    this.session = 0;
    this.task = Promise.resolve();
  }

  start() {
    if (this.running) return;
    this.running = true;
    const session = ++this.session;
    this.task = this.task.then(() => this.run(session));
  }

  async stop() {
    this.running = false;
    await this.task;
    if (!this.running) this.onRate(0);
  }

  async run(session) {
    let next = performance.now();
    let measuredAt = next;
    let samples = 0;
    try {
      while (this.running && session === this.session) {
        const status = await this.client.status(150);
        const now = performance.now();
        this.onSample(status, now);
        samples++;
        if (now - measuredAt >= 1000) {
          this.onRate(samples * 1000 / (now - measuredAt));
          samples = 0;
          measuredAt = now;
        }
        next = Math.max(next + this.interval, now);
        const wait = next - performance.now();
        if (wait > 0 && this.running && session === this.session) await new Promise(resolve => setTimeout(resolve, wait));
      }
    } catch (error) {
      if (session === this.session) {
        this.running = false;
        this.onRate(0);
        this.onError(error);
      }
    }
  }
}
