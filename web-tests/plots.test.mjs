import assert from 'node:assert/strict';
import {after, test} from 'node:test';
import {equalRanges, markerPaths} from '../docs/plots.js';
import {calibrationPlots} from '../docs/calibration.js';

class Path {
  constructor() { this.commands = []; }
  moveTo(...args) { this.commands.push(['moveTo', ...args]); }
  lineTo(...args) { this.commands.push(['lineTo', ...args]); }
  closePath() { this.commands.push(['closePath']); }
  rect(...args) { this.commands.push(['rect', ...args]); }
}
const originalPath = globalThis.Path2D;
globalThis.Path2D = Path;
after(() => { if (originalPath) globalThis.Path2D = originalPath; else delete globalThis.Path2D; });

function plot(data) {
  return {data, series: [{}, ...data.slice(1).map(() => ({points: {size: 9, width: 1.5}}))],
    ctx: {canvas: {width: 600}}, width: 300, bbox: {left: 20, top: 30, width: 400, height: 200},
    valToPos(value, axis, canvas) {
      assert.equal(canvas, true);
      return axis === 'x' ? 20 + value * 2 : 130 - value * 3;
    }};
}

test('cross markers draw every independent XY point without shared time indices', () => {
  const chart = plot([null, [[-8, 0, 13, 21], [2, 5, null, -3]], [[100], [7]]]);
  const paths = markerPaths('cross')(chart, 1, undefined, undefined);
  const centers = [[4, 124], [20, 115], [62, 139]];
  assert.deepEqual(paths.stroke.commands, centers.flatMap(([x, y]) => [
    ['moveTo', x - 9, y - 9], ['lineTo', x + 9, y + 9],
    ['moveTo', x - 9, y + 9], ['lineTo', x + 9, y - 9],
  ]));
  assert.equal(paths.fill, null);
  assert.equal(paths.flags, 3);
  assert.deepEqual(paths.clip.commands, [['rect', 9.5, 19.5, 421, 221]]);
  // A second series has its own extent, independent of the first one.
  assert.equal(markerPaths('cross')(chart, 2).stroke.commands.length, 4);
});

test('triangle markers draw and fill each valid independent XY point', () => {
  const chart = plot([null, [[0, 1], [2, 3]], [[4, 5, 6], [7, 8, 9]]]);
  const paths = markerPaths('triangle')(chart, 2);
  assert.equal(paths.fill, paths.stroke);
  assert.deepEqual(paths.stroke.commands, [[28, 109], [30, 106], [32, 103]].flatMap(([x, y]) => [
    ['moveTo', x, y - 9], ['lineTo', x + 9, y + 9], ['lineTo', x - 9, y + 9], ['closePath'],
  ]));
  assert.equal(paths.flags, 3);
});

test('custom markers omit missing and nonfinite coordinates and include stroke in clipping', () => {
  const chart = plot([null, [[0, null, 2, NaN, 4, 5, Infinity], [1, 2, undefined, 3, NaN, 6, 7]]]);
  for (const shape of ['cross', 'triangle']) {
    const paths = markerPaths(shape)(chart, 1);
    assert.equal(paths.stroke.commands.length, 8);
    assert.ok(paths.stroke.commands.flat().filter(value => typeof value === 'number').every(Number.isFinite));
    const [, left, top, width, height] = paths.clip.commands[0];
    const outer = 9 + 1.5; // Radius plus half a 1.5 px stroke at pixel ratio 2.
    assert.equal(left, chart.bbox.left - outer);
    assert.equal(top, chart.bbox.top - outer);
    assert.equal(width, chart.bbox.width + outer * 2);
    assert.equal(height, chart.bbox.height + outer * 2);
  }
});

function close(actual, expected) {
  assert.ok(Math.abs(actual - expected) < 1e-10, `${actual} differs from ${expected}`);
}

test('equal ranges give square, wide and tall plots equal pixels per data unit', () => {
  for (const [width, height] of [[400, 400], [800, 200], [200, 800], [640, 240]]) {
    const x = {min: 3, max: 13}, y = {min: -17, max: 1};
    const result = equalRanges(x, y, width, height);
    close(width / (result.x.max - result.x.min), height / (result.y.max - result.y.min));
    close((result.x.min + result.x.max) / 2, 8);
    close((result.y.min + result.y.max) / 2, -8);
    assert.ok(result.x.min <= x.min && result.x.max >= x.max);
    assert.ok(result.y.min <= y.min && result.y.max >= y.max);
    assert.deepEqual(x, {min: 3, max: 13});
    assert.deepEqual(y, {min: -17, max: 1});
    assert.deepEqual(equalRanges(x, y, width * 2, height * 2), result);
  }
});

test('equal ranges retain zoom centers and leave matching aspect ratios unchanged', () => {
  const zoom = {x: {min: 7, max: 9}, y: {min: -9, max: -7}};
  const result = equalRanges(zoom.x, zoom.y, 640, 240);
  close((result.x.min + result.x.max) / 2, 8);
  close((result.y.min + result.y.max) / 2, -8);
  close(result.y.min, zoom.y.min); close(result.y.max, zoom.y.max);
  const repeated = equalRanges(result.x, result.y, 640, 240);
  for (const axis of ['x', 'y']) for (const edge of ['min', 'max']) close(repeated[axis][edge], result[axis][edge]);
  assert.deepEqual(equalRanges({min: -20, max: 20}, {min: -5, max: 5}, 800, 200),
    {x: {min: -20, max: 20}, y: {min: -5, max: 5}});
  assert.deepEqual(equalRanges({min: 8, max: 8}, {min: -3, max: 3}, 200, 400),
    {x: {min: 6.5, max: 9.5}, y: {min: -3, max: 3}});
});

test('calibration plots keep raw circles, reduced crosses and X/Y field markers explicit', () => {
  const raw = [{index: 0, step_index: 1, sweep_dir: 0, x: 10, y: 20},
    {index: 108, step_index: 2, sweep_dir: 1, x: 30, y: 40}];
  const reduced = raw.map(point => ({...point, x: point.x + 100, y: point.y + 100}));
  const plots = calibrationPlots(raw, reduced);
  for (const name of ['forward', 'reverse']) {
    const samples = plots.xy.find(series => series.name === `${name} raw`);
    assert.equal(samples.pointsOnly, true);
    assert.equal(samples.marker, 'circle');
    assert.equal(samples.size, 4);
    assert.equal(samples.pointColor, samples.color + '66');
    const anchors = plots.xy.find(series => series.name === `${name} reduced`);
    assert.equal(!!anchors.pointsOnly, false);
    assert.equal(anchors.marker, 'cross');
    assert.equal(anchors.size, 9);
    const x = plots.field.find(series => series.name === `${name} X`);
    const y = plots.field.find(series => series.name === `${name} Y`);
    assert.equal(x.marker, 'circle'); assert.equal(y.marker, 'triangle');
    assert.equal(x.size, 4); assert.equal(y.size, 4);
    assert.equal(x.y[0], anchors.x[0]); assert.equal(y.y[0], anchors.y[0]);
  }
});
