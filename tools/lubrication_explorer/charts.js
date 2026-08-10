(function(){
//=================================================================================================
//  charts.js — uPlot wiring for the lubrication explorer (replaces the old canvas plot.js).
//
//  Renders the 4x2 scope array as one instrument: every panel is a uPlot chart, all sharing a
//  linked crosshair (cursor.sync) and a common gap axis (x-zoom on one panel zooms all). Adds
//  drag-box zoom, wheel zoom, and double-click-resets-everything, plus annotated vertical rules
//  at the saturation onset h_c and the outer cutoff h_cut.
//
//  Contract with app.js: Charts.render(frame) where frame = {
//    signature,                         // structural key; changes force a rebuild (else setData)
//    xMode:'loggap'|'linax', yMode:'lin'|'log',
//    panels: [ { key, leftRoot, rightRoot,               // DOM mount points
//                xs, markers:[{x,label}],                // shared x + marker positions (x-domain)
//                left:{title,xLabel,yLabel,series:[{label,color,dash,width,values}]},
//                right:{...} } ]
//  }
//=================================================================================================

const SYNC = "lubgap";
const AXIS = "#8493a6", GRID = "rgba(120,140,160,0.12)", MARKER = "#c98bdb";

let instances = new Map();   // cellId -> { u, root }
let lastSig = null;
let zoomLocked = false;      // once the user zooms, keep the view until they double-click
let syncing = false;         // guards x-scale propagation across charts

const allU = () => Array.from(instances.values(), (r) => r.u);

//-------------------------------------------------------------------------------------------------
// Interaction plugins / hooks
//-------------------------------------------------------------------------------------------------
// Wheel = zoom x about the cursor (Shift = zoom y). Works in linear and log scale space.
function wheelZoomPlugin() {
   return { hooks: { ready: (u) => {
      u.over.addEventListener("wheel", (e) => {
         e.preventDefault();
         zoomLocked = true;
         const axisKey = e.shiftKey ? "y" : "x";
         const sc = u.scales[axisKey];
         const rect = u.over.getBoundingClientRect();
         const pos = axisKey === "x" ? (e.clientX - rect.left) : (e.clientY - rect.top);
         const cur = u.posToVal(pos, axisKey);
         const z = e.deltaY < 0 ? 0.82 : 1 / 0.82;   // in: shrink range; out: grow
         let nmin, nmax;
         if (sc.distr === 3) {
            const lo = Math.log10(sc.min), hi = Math.log10(sc.max), c = Math.log10(cur);
            nmin = 10 ** (c - (c - lo) * z); nmax = 10 ** (c + (hi - c) * z);
         } else {
            nmin = cur - (cur - sc.min) * z; nmax = cur + (sc.max - cur) * z;
         }
         u.setScale(axisKey, { min: nmin, max: nmax });
      }, { passive: false });
   } } };
}

// Vertical marker rules at h_c (saturation) and h_cut (cutoff), read from u.__markers.
function markersDraw(u) {
   const marks = u.__markers; if (!marks) return;
   const ctx = u.ctx, dpr = u.pxRatio || 1;
   const top = u.bbox.top, height = u.bbox.height;
   ctx.save();
   ctx.strokeStyle = MARKER; ctx.fillStyle = MARKER; ctx.lineWidth = dpr;
   ctx.font = `${11 * dpr}px ui-monospace, monospace`;
   for (const m of marks) {
      if (!(m.x > u.scales.x.min && m.x < u.scales.x.max)) continue;
      const cx = Math.round(u.valToPos(m.x, "x", true));
      ctx.setLineDash([4 * dpr, 3 * dpr]);
      ctx.globalAlpha = 0.85;
      ctx.beginPath(); ctx.moveTo(cx, top); ctx.lineTo(cx, top + height); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1;
      ctx.fillText(m.label, cx + 3 * dpr, top + 12 * dpr);
   }
   ctx.restore();
}

// Propagate an x-range change on one chart to all the others (shared gap axis).
function syncXHook(u, key) {
   if (key !== "x" || syncing) return;
   syncing = true;
   const { min, max } = u.scales.x;
   for (const c of allU()) if (c !== u && c.scales.x) c.setScale("x", { min, max });
   syncing = false;
}

function resetAllZoom() {
   zoomLocked = false;
   syncing = true;                       // batch: avoid cross-talk while all reset
   for (const c of allU()) c.setData(c.data, true);   // true = autoscale to full data
   syncing = false;
}

//-------------------------------------------------------------------------------------------------
// Data + chart construction
//-------------------------------------------------------------------------------------------------
function toData(xs, series, yMode) {
   const ys = series.map((s) => s.values.map((v) => {
      if (!Number.isFinite(v)) return null;
      if (yMode === "log") { const a = Math.abs(v); return a > 0 ? a : null; }
      return v;
   }));
   return [xs, ...ys];
}

// Compact numeric tick formatters (tiny forces need sci notation; a_ref/h is plain).
// uPlot passes null for minor log-axis ticks it wants left blank — return "" for those.
const fmtSci = (v) => {
   if (v == null || !Number.isFinite(v)) return "";
   if (v === 0) return "0";
   const a = Math.abs(v);
   return (a >= 1e4 || a < 1e-2) ? v.toExponential(1).replace("e+", "e") : String(+v.toPrecision(3));
};
const fmtPlain = (v) => (v == null || !Number.isFinite(v) ? "" : v === 0 ? "0" : String(+v.toPrecision(3)));

function axisOpts(label, isX, valFmt) {
   return {
      stroke: AXIS,
      grid: { stroke: GRID, width: 1 },
      ticks: { stroke: GRID, width: 1, size: 4 },
      font: "11px ui-monospace, monospace",
      labelFont: "11px ui-monospace, monospace",
      labelSize: isX ? 30 : 34,
      label,
      size: isX ? 48 : 64,
      values: (u, splits) => splits.map(valFmt),
   };
}

function makeChart(root, pane, xs, markers, xMode, yMode) {
   root.textContent = "";
   const opts = {
      width: Math.max(160, root.clientWidth || 360),
      height: 208,
      title: pane.title,
      scales: {
         // Identity range: honor exact zoom bounds and avoid uPlot's log-decade snapping,
         // so wheel/box zoom narrows smoothly instead of jumping decade to decade.
         x: { distr: xMode === "loggap" ? 3 : 1, time: false, range: (u, dmin, dmax) => [dmin, dmax] },
         y: { distr: yMode === "log" ? 3 : 1 },
      },
      axes: [
         axisOpts(pane.xLabel, true, xMode === "loggap" ? fmtSci : fmtPlain),
         axisOpts(pane.yLabel, false, fmtSci),
      ],
      series: [
         { label: pane.xLabel },
         ...pane.series.map((s) => ({
            label: s.label, stroke: s.color, width: s.width || 1.6,
            dash: s.dash || null, points: { show: false },
         })),
      ],
      cursor: {
         sync: { key: SYNC, scales: ["x", null] },
         drag: { x: true, y: true },
         focus: { prox: 16 },
         bind: { dblclick: () => () => { resetAllZoom(); return null; } },
      },
      legend: { live: true },
      hooks: {
         setSelect: [(u) => { if (u.select && u.select.width > 2) zoomLocked = true; }],
         setScale: [syncXHook],
         draw: [markersDraw],
      },
      plugins: [wheelZoomPlugin()],
   };
   const u = new uPlot(opts, toData(xs, pane.series, yMode), root);
   u.__markers = markers;
   return u;
}

//-------------------------------------------------------------------------------------------------
// Public: render a frame (rebuild on structural change, else fast setData)
//-------------------------------------------------------------------------------------------------
function render(frame) {
   if (frame.signature !== lastSig) {
      for (const { u } of instances.values()) u.destroy();
      instances.clear();
      for (const p of frame.panels) {
         instances.set(p.key + "_L", { u: makeChart(p.leftRoot, p.left, p.xs, p.markers, frame.xMode, frame.yMode), root: p.leftRoot });
         instances.set(p.key + "_R", { u: makeChart(p.rightRoot, p.right, p.xs, p.markers, frame.xMode, frame.yMode), root: p.rightRoot });
      }
      lastSig = frame.signature;
      return;
   }
   // Same structure: update data in place, preserving zoom unless the user hasn't zoomed.
   for (const p of frame.panels) {
      const L = instances.get(p.key + "_L"), R = instances.get(p.key + "_R");
      if (L) { L.u.__markers = p.markers; L.u.setData(toData(p.xs, p.left.series, frame.yMode), !zoomLocked); }
      if (R) { R.u.__markers = p.markers; R.u.setData(toData(p.xs, p.right.series, frame.yMode), !zoomLocked); }
   }
}

function resize() {
   for (const { u, root } of instances.values())
      u.setSize({ width: Math.max(160, root.clientWidth || 360), height: 208 });
}

// Debug/testing hook: current x-scale range of every chart (used by the headless harness).
function xRanges() { return allU().map((u) => [u.scales.x.min, u.scales.x.max]); }

window.Charts = { render, resize, resetAllZoom, xRanges };

})();
