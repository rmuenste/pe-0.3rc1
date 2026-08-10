(function(){
//=================================================================================================
//  app.js — UI wiring for the lubrication force explorer.
//  Builds the control desk from a registry that mirrors the live viewer's setupParamRegistry()
//  (tools/live_viewer/live_viewer.cpp) plus the physical/kinematic inputs a force plot needs,
//  then feeds the 4x2 uPlot scope array (charts.js) on every change. L = lubforces.js globals.
//=================================================================================================

//-------------------------------------------------------------------------------------------------
// State (defaults from Params.cpp / the live viewer scenario)
//-------------------------------------------------------------------------------------------------
const S = {
   // Fluid
   viscosity: 8.37e-5,
   // Lubrication model
   enabled: true, model: L.MODEL_KROUPA, scheme: L.SCHEME_SEMI_IMPLICIT,
   tangential: true, twisting: true, slipCorrection: true, resistSeparation: true, wallTerms: true,
   // Cutoffs / regularization
   epsCritical: 0.1, cutoffFactor: 0.5, legacyThreshold: 1e-2,
   contactHysteresis: 1e-9, lubricationHysteresis: 1e-3, minGap: 1e-8,
   // Geometry / kinematics
   wall: true, equalRadii: true, r1: 0.01, r2: 0.01,
   vn: 0.05, vs: 0.02, vcs: 0.01, wtw: 5.0,
   // Per-step (right column only)
   dt: 5e-4, mEff: 1e-3, iEff: 1e-7, alphaImpulseCap: 1.0,
   // View
   normalize: false,     // continuous panels: false = dimensional, true = Fig.7 dimensionless
   xMode: "loggap",      // 'loggap' (log gap h) | 'linax' (linear a_ref/h, Fig.7 framing)
   yMode: "lin",         // 'lin' | 'log' (log plots |value|)
};

//-------------------------------------------------------------------------------------------------
// Control registry — {key, label, type, min, max, log, items, tooltip, enabledIf}
//-------------------------------------------------------------------------------------------------
const REG = [
   { group: "Fluid", controls: [
      { key: "viscosity", label: "viscosity μ", type: "real", min: 1e-6, max: 1, log: true,
        tooltip: "Dynamic viscosity of the surrounding fluid [Pa·s]" },
   ]},
   { group: "Lubrication model", controls: [
      { key: "enabled", label: "enabled", type: "bool", tooltip: "Master switch for lubrication forces" },
      { key: "model", label: "force model", type: "enum", items: ["Kroupa 2016", "legacy"] },
      { key: "scheme", label: "scheme", type: "enum", items: ["semi-implicit", "explicit-capped"],
        tooltip: "Affects the per-step (right) column only" },
      { key: "tangential", label: "tangential terms", type: "bool", tooltip: "Sliding force + sliding torque" },
      { key: "twisting", label: "twisting torque", type: "bool" },
      { key: "slipCorrection", label: "slip correction f*", type: "bool", tooltip: "Vinogradova slip correction" },
      { key: "resistSeparation", label: "resist separation", type: "bool",
        tooltip: "Apply normal suction on separating motion (vₙ<0), not only approach" },
      { key: "wallTerms", label: "wall terms", type: "bool",
        tooltip: "Use the dedicated sphere–plane resistance set (needs 'wall' geometry)" },
   ]},
   { group: "Cutoffs / regularization", controls: [
      { key: "epsCritical", label: "eps_c (critical)", type: "real", min: 1e-5, max: 1e-1, log: true,
        tooltip: "Saturation / slip length scale: eps_c = h_c / a_ref" },
      { key: "cutoffFactor", label: "cutoff factor", type: "real", min: 0, max: 1, log: false,
        tooltip: "Relative outer cutoff eps_cut = h_cut/a_ref; 0 = legacy absolute threshold" },
      { key: "legacyThreshold", label: "legacy threshold", type: "real", min: 1e-6, max: 1e-2, log: true,
        tooltip: "Absolute outer cutoff, active only when cutoff factor == 0" },
      { key: "contactHysteresis", label: "contact hysteresis", type: "real", min: 1e-8, max: 1e-3, log: true,
        tooltip: "Blend ramp-up half-width near contact" },
      { key: "lubricationHysteresis", label: "lubrication hysteresis", type: "real", min: 1e-8, max: 1e-3, log: true,
        tooltip: "Blend ramp-down half-width at the cutoff" },
      { key: "minGap", label: "min eps (gap clamp)", type: "real", min: 1e-10, max: 1e-4, log: true,
        tooltip: "Minimal lubrication gap regularization (minEpsLub)" },
   ]},
   { group: "Geometry / kinematics", controls: [
      { key: "wall", label: "sphere–plane (wall)", type: "bool",
        tooltip: "OFF = sphere–sphere (pair); ON = sphere–plane (wall)" },
      { key: "equalRadii", label: "equal radii (r₂ = r₁)", type: "bool" },
      { key: "r1", label: "radius r₁ [m]", type: "real", min: 1e-4, max: 1e-1, log: true },
      { key: "r2", label: "radius r₂ [m]", type: "real", min: 1e-4, max: 1e-1, log: true,
        enabledIf: (s) => !s.equalRadii && !s.wall },
      { key: "vn", label: "vₙ approach [m/s]", type: "real", min: -0.2, max: 0.2, log: false,
        tooltip: "Approach speed (>0 approaching, <0 separating)" },
      { key: "vs", label: "v_s sliding [m/s]", type: "real", min: -0.2, max: 0.2, log: false,
        tooltip: "Translational sliding surface speed" },
      { key: "vcs", label: "v_cs rot. sliding [m/s]", type: "real", min: -0.2, max: 0.2, log: false,
        tooltip: "Rotational sliding surface speed (from w×r)" },
      { key: "wtw", label: "ω_twist [rad/s]", type: "real", min: -50, max: 50, log: false,
        tooltip: "Relative spin about the contact normal" },
   ]},
   { group: "Per-step (right column)", controls: [
      { key: "dt", label: "dt [s]", type: "real", min: 1e-6, max: 1e-2, log: true },
      { key: "mEff", label: "eff. mass m_eff [kg]", type: "real", min: 1e-6, max: 1, log: true },
      { key: "iEff", label: "eff. inertia I_eff [kg·m²]", type: "real", min: 1e-12, max: 1e-2, log: true },
      { key: "alphaImpulseCap", label: "α impulse cap", type: "real", min: 0, max: 2, log: false,
        tooltip: "Cap factor (explicit-capped scheme only)" },
   ]},
];

//-------------------------------------------------------------------------------------------------
// Components (rows of the scope array)
//-------------------------------------------------------------------------------------------------
const COMPS = [
   { key: "normalF", title: "Normal squeeze force", unit: "N", torque: false, gate: null },
   { key: "slideF", title: "Sliding (tangential) force", unit: "N", torque: false, gate: "tangential" },
   { key: "slideM", title: "Sliding torque", unit: "N·m", torque: true, gate: "tangential" },
   { key: "twistM", title: "Twisting torque", unit: "N·m", torque: true, gate: "twisting" },
];
const COLOR = { cont: "#34d3de", eff: "#ffb454", ref: "rgba(52,211,222,0.34)" };

//-------------------------------------------------------------------------------------------------
// Control rendering
//-------------------------------------------------------------------------------------------------
const t2v = (c, t) => (c.log ? c.min * Math.pow(c.max / c.min, t) : c.min + (c.max - c.min) * t);
const v2t = (c, v) => (c.log ? Math.log(v / c.min) / Math.log(c.max / c.min) : (v - c.min) / (c.max - c.min));
const fmtNum = (v) => {
   if (v === 0) return "0";
   const a = Math.abs(v);
   return (a >= 1e4 || a < 1e-3) ? v.toExponential(3) : (Math.round(v * 1e5) / 1e5).toString();
};

function buildControls() {
   const host = document.getElementById("controls");
   for (const g of REG) {
      const sec = document.createElement("div"); sec.className = "group";
      const h = document.createElement("h3"); h.textContent = g.group; sec.appendChild(h);
      for (const c of g.controls) sec.appendChild(makeControl(c));
      host.appendChild(sec);
   }
}

function makeControl(c) {
   const row = document.createElement("div"); row.className = "ctrl"; row.dataset.key = c.key;
   const lab = document.createElement("label"); lab.textContent = c.label;
   if (c.tooltip) lab.title = c.tooltip;
   row.appendChild(lab);

   if (c.type === "bool") {
      const cb = document.createElement("input"); cb.type = "checkbox"; cb.checked = !!S[c.key];
      cb.addEventListener("change", () => { S[c.key] = cb.checked; onChange(); });
      row.appendChild(cb);
   } else if (c.type === "enum") {
      const sel = document.createElement("select");
      c.items.forEach((it, i) => { const o = document.createElement("option"); o.value = i; o.textContent = it; sel.appendChild(o); });
      sel.value = S[c.key];
      sel.addEventListener("change", () => { S[c.key] = parseInt(sel.value, 10); onChange(); });
      row.appendChild(sel);
   } else { // real
      const rng = document.createElement("input"); rng.type = "range";
      rng.min = 0; rng.max = 1000; rng.step = 1; rng.value = Math.round(v2t(c, S[c.key]) * 1000);
      const num = document.createElement("input"); num.type = "text"; num.className = "num"; num.value = fmtNum(S[c.key]);
      rng.addEventListener("input", () => { S[c.key] = t2v(c, rng.value / 1000); num.value = fmtNum(S[c.key]); onChange(); });
      num.addEventListener("change", () => {
         const v = parseFloat(num.value);
         if (Number.isFinite(v)) { S[c.key] = v; rng.value = Math.round(v2t(c, v) * 1000); onChange(); }
      });
      row.appendChild(rng); row.appendChild(num);
      row._sync = () => { rng.value = Math.round(v2t(c, S[c.key]) * 1000); num.value = fmtNum(S[c.key]); };
   }
   row._ctrl = c;
   return row;
}

function refreshControlStates() {
   document.querySelectorAll(".ctrl").forEach((row) => {
      const c = row._ctrl;
      if (c.enabledIf) {
         const on = c.enabledIf(S);
         row.style.opacity = on ? 1 : 0.4;
         row.querySelectorAll("input,select").forEach((el) => (el.disabled = !on));
      }
      if (row._sync && !row.contains(document.activeElement)) row._sync();
   });
}

//-------------------------------------------------------------------------------------------------
// Derived readouts
//-------------------------------------------------------------------------------------------------
function aRef() { return L.refRadius(S.wall, S.r1, S.equalRadii ? S.r1 : S.r2); }

function updateReadout() {
   const a = aRef();
   const hc = S.epsCritical * a;
   const hCut = L.lubricationCutoff(a, S);
   const sc = L.selfCheck();
   const el = document.getElementById("readout");
   el.textContent = "";
   const stat = (k, v) => {
      const s = document.createElement("span"); s.className = "stat";
      const kk = document.createElement("b"); kk.textContent = k;
      const vv = document.createElement("i"); vv.textContent = v;
      s.appendChild(kk); s.appendChild(vv); return s;
   };
   el.appendChild(stat("a_ref", fmtNum(a) + " m"));
   el.appendChild(stat("h_c", fmtNum(hc) + " m"));
   el.appendChild(stat("h_cut", fmtNum(hCut) + " m"));
   el.appendChild(stat("1/ε_c", fmtNum(1 / S.epsCritical)));
   const chk = document.createElement("span");
   chk.className = "chk " + (sc.pass ? "ok" : "bad");
   chk.textContent = "self-check " + (sc.pass ? "PASS" : "FAIL");
   chk.title = sc.lines.join("\n");
   el.appendChild(chk);
}

//-------------------------------------------------------------------------------------------------
// Formula captions (context-sensitive)
//-------------------------------------------------------------------------------------------------
const useWall = () => S.wall && S.wallTerms;

function cnStr() { return useWall() ? "1/ε − ⅕·lnε − 1/21·ε lnε" : "¼/ε − 9/40·lnε − 3/112·ε lnε"; }
function slideStr() {
   return useWall()
      ? "c_vs=−8/15 lnε−64/375 ε lnε, c_vcs=−2/15 lnε−86/375 ε lnε"
      : "c_vs=−⅙ lnε, c_vcs=−⅙ lnε−1/12 ε lnε";
}
function torqueStr() {
   return useWall()
      ? "c'_vs=−2/15 lnε−86/375 ε lnε, c'_vcs=−⅖ lnε−66/125 ε lnε"
      : "c'_vs=−⅙ lnε−1/12 ε lnε, c'_vcs=−⅕ lnε−47/250 ε lnε";
}
function twistStr() { return useWall() ? "C_t = ½·ε|lnε|" : "C_t = ⅛·ε|lnε|"; }

function continuousFormula(compKey) {
   if (S.model === L.MODEL_LEGACY) {
      if (compKey === "normalF") return `Fₙ = 6πμ·R_eff²·(−vₙ)/max(h,minGap),  R_eff = ${S.wall ? "a_ref" : "a_ref/2"}`;
      return "(off in legacy model)";
   }
   const norm = S.normalize;
   switch (compKey) {
      case "normalF": return norm
         ? `Fₙ/(vₙ·a·μ) = 6π·f*·Cₙ,  Cₙ = ${cnStr()}`
         : `Fₙ = 6πμ·a·f*·Cₙ·vₙ,  Cₙ = ${cnStr()}`;
      case "slideF": return norm
         ? `F_t/(v·a·μ) = 6π·f*·(c_vs·v_s+c_vcs·v_cs)/v,  ${slideStr()}`
         : `F_t = 6πμ·a·f*·(c_vs·v_s + c_vcs·v_cs),  ${slideStr()}`;
      case "slideM": return norm
         ? `M_sl/(v·a²·μ) = 8π·f*·(c'_vs·v_s+c'_vcs·v_cs)/v,  ${torqueStr()}`
         : `M_sl = 8πμ·a²·f*·(c'_vs·v_s + c'_vcs·v_cs),  ${torqueStr()}`;
      case "twistM": return norm
         ? `M_t/(ω·a³·μ) = 8π·f*·C_t/a,  ${twistStr()}`
         : `M_t = 8πμ·a²·f*·C_t·ω,  ${twistStr()}`;
   }
}
function perStepFormula(compKey) {
   if (S.model === L.MODEL_LEGACY)
      return "F_eff = w·min(F, α·w·m|vₙ|)/dt   (legacy, explicit-capped)";
   const semi = S.scheme === L.SCHEME_SEMI_IMPLICIT;
   switch (compKey) {
      case "normalF": return semi
         ? "F_eff = w·m(1−e^(−Kₙ·dt/m))·vₙ / dt"
         : "F_eff = w·min(Fₙ, α·w·m|vₙ|/dt)";
      case "slideF": return semi ? "F_eff = w·F_t·(1−e^(−K_s·dt/m))/(K_s·dt/m)" : "F_eff = w·F_t";
      case "slideM": return semi ? "M_eff = w·M_sl·(1−e^(−K_s·a²·dt/I))/(K_s·a²·dt/I)" : "M_eff = w·M_sl";
      case "twistM": return semi ? "M_eff = w·M_t·(1−e^(−K_t·dt/I))/(K_t·dt/I)" : "M_eff = w·M_t";
   }
}

//-------------------------------------------------------------------------------------------------
// Compute + build a frame for charts.js
//-------------------------------------------------------------------------------------------------
const N = 256;

function normalizeVal(compKey, val, a) {
   const mu = S.viscosity;
   let vref;
   if (compKey === "normalF") vref = Math.abs(S.vn);
   else if (compKey === "twistM") vref = Math.abs(S.wtw) * a;
   else vref = Math.hypot(S.vs, S.vcs);
   if (vref < 1e-14) return NaN;
   const denom = COMPS.find((c) => c.key === compKey).torque ? mu * a * a * vref : mu * a * vref;
   return val / denom;
}

function sampleGaps(a) {
   const hCut = L.lubricationCutoff(a, S);
   const hMin = Math.max(1e-8 * a, 0.05 * S.epsCritical * a);
   const hs = [];
   for (let i = 0; i < N; i++) hs.push(hMin * Math.pow(hCut / hMin, i / (N - 1)));
   return { hs, hMin, hCut };
}

function yUnit(comp) {
   if (S.normalize) return comp.torque ? "M/(v·a²·μ)" : "F/(v·a·μ)";
   return `|${comp.torque ? "M" : "F"}| [${comp.unit}]`;
}

function render() {
   const a = aRef();
   const { hs, hCut } = sampleGaps(a);
   const hc = S.epsCritical * a;
   const kin = { vn: S.vn, vs: S.vs, vcs: S.vcs, wtw: S.wtw };
   const step = { dt: S.dt, mEff: S.mEff, iEff: S.iEff };

   const cont = hs.map((h) => L.components(h, a, S.wall, kin, S));
   const eff = hs.map((h) => L.effectiveComponents(h, a, S.wall, kin, S, step));

   // Shared x axis + marker positions in the chosen x-domain.
   const loggap = S.xMode === "loggap";
   const xLabel = loggap ? "gap h [m]" : "a_ref / h";
   let xs = loggap ? hs.slice() : hs.map((h) => a / h);
   const markers = loggap
      ? [{ x: hc, label: "h_c" }, { x: hCut, label: "cut" }]
      : [{ x: a / hc, label: "1/ε_c" }, { x: a / hCut, label: "cut" }];
   const revIfNeeded = (arr) => (loggap ? arr : arr.slice().reverse());
   if (!loggap) xs = xs.slice().reverse();

   const schemeName = S.scheme === L.SCHEME_SEMI_IMPLICIT ? "semi-implicit" : "explicit-capped";
   const panels = [];
   const visibleKeys = [];

   for (const comp of COMPS) {
      const row = ROWS[comp.key];
      const visible = S.enabled && (!comp.gate || S[comp.gate]) &&
                      !(S.model === L.MODEL_LEGACY && comp.key !== "normalF");
      row.el.style.display = visible ? "" : "none";
      if (!visible) continue;
      visibleKeys.push(comp.key);

      const contDim = cont.map((c) => c[comp.key]);
      const contShown = S.normalize ? cont.map((c) => normalizeVal(comp.key, c[comp.key], a)) : contDim;
      const effVals = eff.map((c) => c[comp.key]);

      row.capL.textContent = continuousFormula(comp.key);
      row.capR.textContent = perStepFormula(comp.key);

      panels.push({
         key: comp.key, leftRoot: row.leftRoot, rightRoot: row.rightRoot, xs, markers,
         left: {
            title: `${comp.title} — continuous`, xLabel, yLabel: yUnit(comp),
            series: [{ label: "continuous", color: COLOR.cont, values: revIfNeeded(contShown) }],
         },
         right: {
            title: `${comp.title} — per-step (${schemeName})`, xLabel,
            yLabel: `|${comp.torque ? "M" : "F"}| [${comp.unit}]`,
            series: [
               { label: "cont (ref)", color: COLOR.ref, dash: [6, 3], width: 1, values: revIfNeeded(contDim) },
               { label: "effective", color: COLOR.eff, values: revIfNeeded(effVals) },
            ],
         },
      });
   }

   const signature = [visibleKeys.join(","), S.xMode, S.yMode, S.normalize, S.scheme, S.model].join("|");
   Charts.render({ signature, xMode: S.xMode, yMode: S.yMode, panels });
}

function onChange() {
   if (S.equalRadii) S.r2 = S.r1;
   refreshControlStates();
   updateReadout();
   render();
}

//-------------------------------------------------------------------------------------------------
// Grid construction (static DOM; charts.js mounts uPlot into the .uroot divs)
//-------------------------------------------------------------------------------------------------
const ROWS = {};
function buildGrid() {
   const grid = document.getElementById("grid");
   for (const comp of COMPS) {
      const row = document.createElement("div"); row.className = "panelrow"; row.dataset.key = comp.key;
      const rec = { el: row };
      for (const side of ["L", "R"]) {
         const cell = document.createElement("div"); cell.className = "cell";
         const root = document.createElement("div"); root.className = "uroot";
         const cap = document.createElement("div"); cap.className = "formula";
         cell.appendChild(root); cell.appendChild(cap);
         row.appendChild(cell);
         rec[side === "L" ? "leftRoot" : "rightRoot"] = root;
         rec[side === "L" ? "capL" : "capR"] = cap;
      }
      grid.appendChild(row);
      ROWS[comp.key] = rec;
   }
}

//-------------------------------------------------------------------------------------------------
// Top bar (axis modes, normalization, reset view)
//-------------------------------------------------------------------------------------------------
function segmented(el, options, current, onPick) {
   el.textContent = "";
   options.forEach(([val, txt]) => {
      const b = document.createElement("button"); b.type = "button"; b.textContent = txt;
      b.className = "seg" + (val === current() ? " on" : "");
      b.addEventListener("click", () => { onPick(val); el.querySelectorAll(".seg").forEach((x) => x.classList.remove("on")); b.classList.add("on"); });
      el.appendChild(b);
   });
}

function buildTopBar() {
   segmented(document.getElementById("xmode"), [["loggap", "log gap"], ["linax", "a_ref/h"]],
      () => S.xMode, (v) => { S.xMode = v; onChange(); });
   segmented(document.getElementById("ymode"), [["lin", "linear"], ["log", "log |·|"]],
      () => S.yMode, (v) => { S.yMode = v; onChange(); });
   document.getElementById("normToggle").addEventListener("change", (e) => { S.normalize = e.target.checked; onChange(); });
   document.getElementById("resetBtn").addEventListener("click", () => Charts.resetAllZoom());
}

//-------------------------------------------------------------------------------------------------
// Init
//-------------------------------------------------------------------------------------------------
buildControls();
buildGrid();
buildTopBar();
onChange();
let rt;
window.addEventListener("resize", () => { clearTimeout(rt); rt = setTimeout(() => Charts.resize(), 120); });

})();
