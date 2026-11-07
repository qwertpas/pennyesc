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

// Streams ESC status through the bridge. Silence is reported through onStale
// and the stream is requested again, so the plot resumes after an ESC or
// bridge restart.
export class Telemetry {
  constructor(client, {onSample, onRate, onError, onStale = () => {}, rate = 200}) {
    if (!Number.isFinite(rate) || rate <= 0) throw new Error('Sample rate must be positive');
    Object.assign(this, {client, onSample, onRate, onError, onStale, rate});
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
    this.client.connection.sampleWake?.();
    await this.task;
    if (!this.running) this.onRate(0);
  }

  async run(session) {
    const connection = this.client.connection;
    const active = () => this.running && session === this.session;
    let measuredAt = performance.now(), samples = 0;
    try {
      await connection.startTelemetry(this.client.address, this.rate);
      while (active()) {
        const sample = await connection.nextTelemetry(250);
        const now = performance.now();
        if (sample) { this.onSample(sample.status, sample.now); samples++; }
        else if (active()) {
          this.onStale(); this.onRate(0); samples = 0; measuredAt = now;
          // A bridge that restarted has forgotten the stream, so ask again.
          try { await connection.startTelemetry(this.client.address, this.rate); }
          catch (error) { if (error.name !== 'TimeoutError') throw error; }
          continue;
        }
        if (now - measuredAt >= 1000) {
          this.onRate(samples * 1000 / (now - measuredAt));
          samples = 0;
          measuredAt = now;
        }
      }
    } catch (error) {
      if (session === this.session) {
        this.running = false;
        this.onRate(0);
        this.onError(error);
      }
    } finally {
      if (connection.connected) {
        try { await connection.stopTelemetry(); } catch { /* The connection reports its own failure. */ }
      }
    }
  }
}
