import uPlot from './vendor/uPlot.esm.js';

const axis = label => ({label, labelSize: 16, size: 46, stroke: '#6b7788',
  font: '11px system-ui', labelFont: '11px system-ui',
  grid: {stroke: '#e9edf1', width: 1}, ticks: {stroke: '#dce1e7', width: 1}});

export function equalRanges(x, y, width, height) {
  const unit = Math.max((x.max - x.min) / width, (y.max - y.min) / height);
  return Object.fromEntries([['x', x, width], ['y', y, height]].map(([key, range, pixels]) => {
    const middle = (range.min + range.max) / 2, half = unit * pixels / 2;
    return [key, {min: middle - half, max: middle + half}];
  }));
}

function keepAspect(plot) {
  // Run after uPlot commits its measured drawing area, including resize and zoom.
  queueMicrotask(() => {
    if (!plot.root.isConnected) return;
    const {x, y} = plot.scales, {width, height} = plot.bbox;
    if (x.min == null || y.min == null || !width || !height) return;
    const xUnit = (x.max - x.min) / width, yUnit = (y.max - y.min) / height;
    if (Math.abs(xUnit - yUnit) <= Math.max(xUnit, yUnit) * 1e-7) return;
    const ranges = equalRanges(x, y, width, height);
    plot.batch(() => { plot.setScale('x', ranges.x); plot.setScale('y', ranges.y); });
  });
}

export function markerPaths(shape) {
  const circles = uPlot.paths.points();
  return (plot, index) => {
    const [xs, ys] = plot.data[index];
    // uPlot's shared time-series indices do not apply to independent XY series.
    if (shape === 'circle') return circles(plot, index, 0, xs.length - 1);
    const path = new Path2D(), clip = new Path2D();
    const ratio = plot.ctx.canvas.width / plot.width;
    const size = plot.series[index].points.size * ratio / 2;
    const padding = size + plot.series[index].points.width * ratio / 2;
    const {left, top, width, height} = plot.bbox;
    clip.rect(left - padding, top - padding, width + padding * 2, height + padding * 2);
    xs.forEach((value, i) => {
      if (!Number.isFinite(value) || !Number.isFinite(ys[i])) return;
      const x = plot.valToPos(value, 'x', true), y = plot.valToPos(ys[i], 'y', true);
      if (shape === 'cross') {
        path.moveTo(x - size, y - size); path.lineTo(x + size, y + size);
        path.moveTo(x - size, y + size); path.lineTo(x + size, y - size);
      } else {
        path.moveTo(x, y - size); path.lineTo(x + size, y + size);
        path.lineTo(x - size, y + size); path.closePath();
      }
    });
    return {stroke: path, fill: shape === 'triangle' ? path : null, clip, flags: 3};
  };
}

export class Plot {
  constructor(element, title, xLabel, yLabel, xy = false, equal = false) {
    Object.assign(this, {element, title, xLabel, yLabel, xy, equal});
    this.series = [];
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(element);
  }

  set(series) {
    if (!series.length) series = [{name: 'forward', color: '#198568', x: [], y: []}];
    const key = series.map(s => s.name).join('|');
    if (!this.plot || key !== this.key) {
      this.plot?.destroy();
      this.key = key;
      const definitions = series.map(s => ({label: s.name, class: s.marker === 'cross' ? 'cross' : s.pointsOnly ? 'raw' : s.marker,
        stroke: s.color, width: s.pointsOnly ? 0 : 1.5,
        points: {show: this.xy && !!s.marker, size: s.size ?? 4, width: s.marker === 'cross' ? 1.5 : 1,
          stroke: s.pointColor ?? s.color, fill: s.pointColor ?? s.color, paths: markerPaths(s.marker ?? 'circle')},
        ...(this.xy ? {facets: [{scale: 'x', sorted: 0}, {scale: 'y'}]} : {})}));
      const options = {title: this.title, mode: this.xy ? 2 : 1, width: 300, height: 200,
        series: [{label: this.xLabel}, ...definitions], legend: {show: true, live: !this.xy},
        scales: {x: {time: false}, y: {}}, axes: [axis(this.xLabel), axis(this.yLabel)],
        cursor: {drag: {x: true, y: this.xy}, ...(this.xy ? {} : {sync: {key: 'live'}})},
        hooks: this.equal ? {draw: [keepAspect]} : {},
        padding: [8, 14, 0, 0]};
      this.plot = new uPlot(options, this.data(series), this.element);
      if (this.xy) this.plot.root.querySelectorAll('.u-legend .u-marker').forEach((marker, i) => {
        marker.style.color = series[i].color;
      });
      this.resize();
    } else {
      this.plot.setData(this.data(series));
    }
    this.series = series;
  }

  data(series) {
    return this.xy ? [null, ...series.map(s => [s.x, s.y])] : [series[0]?.x ?? [], ...series.map(s => s.y)];
  }

  resize() {
    if (!this.plot || !this.element.clientWidth) return;
    this.plot.root.style.width = `${this.element.clientWidth - 16}px`;
    const extra = this.plot.root.querySelector('.u-title').offsetHeight +
      this.plot.root.querySelector('.u-legend').offsetHeight + 12;
    this.plot.setSize({width: this.element.clientWidth - 16, height: Math.max(100, this.element.clientHeight - 16 - extra)});
  }

  async png() {
    const source = this.plot.ctx.canvas;
    const scale = devicePixelRatio;
    const canvas = document.createElement('canvas');
    canvas.width = source.width;
    canvas.height = source.height + 60 * scale;
    const context = canvas.getContext('2d');
    context.fillStyle = 'white'; context.fillRect(0, 0, canvas.width, canvas.height);
    context.drawImage(source, 0, 30 * scale);
    context.scale(scale, scale);
    context.font = '13px system-ui'; context.fillStyle = '#253144';
    context.fillText(this.title, 12, 20);
    context.font = '11px system-ui';
    let x = 12;
    for (const series of this.series) {
      context.fillStyle = series.color; context.fillText(series.name, x, canvas.height / scale - 12);
      x += context.measureText(series.name).width + 22;
    }
    return new Promise(resolve => canvas.toBlob(resolve));
  }
}
