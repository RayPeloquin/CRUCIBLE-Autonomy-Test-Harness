"""Generate exact-data overview, offline interactive evidence viewer and README.

Run separately from the MuJoCo environment: matplotlib is an optional reporting
dependency. This script never changes the frozen evaluator or campaign evidence.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(path):
    p = Path(path)
    return json.loads(gzip.decompress(p.read_bytes()) if p.suffix == ".gz" else p.read_text())


def make_figure(s):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    bg, fg, muted = "#0a1422", "#f0f5fb", "#a8b8ca"
    legacy, guarded, line = "#ffb579", "#62dec8", "#283c53"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "text.color": fg, "axes.labelcolor": muted,
                         "xtick.color": muted, "ytick.color": muted,
                         "axes.edgecolor": line, "axes.facecolor": bg})
    fig = plt.figure(figsize=(16, 11), facecolor=bg)
    fig.text(.065, .94, "CRUCIBLE", fontsize=36, fontweight="bold")
    fig.text(.395, .944, "v0.2  /  PHYSICS & EVIDENCE", color=guarded, fontsize=13, fontweight="bold")
    fig.text(.065, .902, "Rigid-body models. Declared mission rules. Failures you can inspect and replay.", color=muted, fontsize=15)
    cards = [("34", "independent checks passed"), ("160", "held-out controller runs"),
             ("6 / 6", "exact replays incl. checkpoints"), ("4 / 4", "negative controls detected")]
    for i, (value, label) in enumerate(cards):
        x = .065 + i * .232
        fig.text(x, .83, value, fontsize=27, color=guarded, fontweight="bold")
        fig.text(x, .801, label, fontsize=10, color=muted)

    ax = fig.add_axes([.08, .49, .38, .235])
    labels, pos = [], []
    for i, model in enumerate(("rover", "multirotor")):
        for j, variant in enumerate(("legacy_nav_gated", "guarded_failsafe")):
            data = s["models"][model]["heldout"][variant]
            x = i * 1.3 + j * .4
            p, (lo, hi) = data["failure_fraction"], data["wilson_95"]
            ax.bar(x, p * 100, width=.32, color=legacy if j == 0 else guarded)
            ax.errorbar(x, p * 100, yerr=[[100 * (p - lo)], [100 * (hi - p)]], color=fg, capsize=4, lw=1.2)
            ax.text(x, hi * 100 + 4, f"{data['counts']['VIOLATED']}/40", ha="center", fontsize=11, color=fg)
            pos.append(x)
            labels.append("Legacy" if j == 0 else "Guarded")
    ax.set_xticks(pos, labels)
    ax.set_ylim(0, 104)
    ax.set_yticks([0, 25, 50, 75, 100], ["0%", "25%", "50%", "75%", "100%"])
    ax.set_ylabel("Scenarios with a contract violation")
    ax.set_title("HELD-OUT RESULTS", loc="left", pad=22, fontweight="bold", color=fg)
    ax.text(.20, -.25, "ROVER", transform=ax.transAxes, ha="center", color=muted, fontsize=10)
    ax.text(.80, -.25, "QUADROTOR", transform=ax.transAxes, ha="center", color=muted, fontsize=10)
    ax.yaxis.grid(True, color=line, alpha=.6)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
    fig.text(.08, .409, "95% Wilson intervals • synthetic distribution only", color=muted, fontsize=10)
    fig.text(.08, .387, "Quadrotor pairs: 5 improvements, 1 regression, 34 unchanged.", color=muted, fontsize=9)

    ax2 = fig.add_axes([.57, .49, .365, .235])
    for variant, color, label in (("legacy_nav_gated", legacy, "Legacy"), ("guarded_failsafe", guarded, "Guarded")):
        r = read(ROOT / "results/multirotor" / f"isolated-outage-{variant}.json.gz")
        ax2.plot([row["t"] for row in r["rows"]], [row["speed"] for row in r["rows"]], color=color, lw=2, label=label)
    ax2.axvspan(1.8, 4.1, color="#8193aa", alpha=.13)
    ax2.axvline(2, color=muted, ls=":", lw=1)
    ax2.axvline(5, color=legacy, ls=":", lw=1)
    ax2.axhline(.3, color=guarded, ls="--", lw=1)
    ax2.text(2.08, 1.7, "link loss", color=muted, fontsize=9)
    ax2.text(5.08, 1.7, "deadline", color=legacy, fontsize=9)
    ax2.set(xlim=(0, 8), ylim=(0, 1.9), xlabel="Time (s)", ylabel="Horizontal speed (m/s)")
    ax2.set_title("ISOLATED NAVIGATION OUTAGE", loc="left", pad=22, fontweight="bold", color=fg)
    ax2.spines[["top", "right"]].set_visible(False)
    ax2.grid(color=line, alpha=.5)
    ax2.legend(frameon=False, labelcolor=fg, loc="upper right", fontsize=9)
    fig.text(.57, .409, "Quadrotor development example: legacy violates; guard passes.", color=muted, fontsize=9)
    fig.text(.57, .387, "Full HOLD verdict also checks vertical speed, altitude and drift.", color=muted, fontsize=9)

    ax3 = fig.add_axes([.08, .16, .38, .135])
    reductions = [s["models"][m]["reduction"]["fraction_reduced"] for m in ("rover", "multirotor")]
    ax3.barh([1, 0], [1, 1], height=.35, color=line)
    ax3.barh([1, 0], [1 - r for r in reductions], height=.35, color=guarded)
    ax3.set_yticks([1, 0], ["Rover", "Quadrotor"])
    ax3.set_xlim(0, 1.3)
    ax3.set_xticks([0, .5, 1], ["0", "50%", "100%"])
    for y, reduced in zip([1, 0], reductions):
        ax3.text(1.04, y, f"−{reduced:.1%}", va="center", color=guarded, fontsize=12, fontweight="bold")
    ax3.set_title("SMALLER COUNTEREXAMPLES", loc="left", pad=23, fontweight="bold", color=fg)
    ax3.spines[["top", "right", "left"]].set_visible(False)
    fig.text(.08, .105, "Declared fault-cost score retained after greedy reduction.", color=muted, fontsize=9)
    fig.text(.08, .084, "Violation confirmed on 3 seeds. Cost reduction ≠ reliability gain.", color=muted, fontsize=9)

    fig.text(.57, .325, "EVIDENCE THAT CAN DISAGREE", color=fg, fontsize=12, fontweight="bold")
    checks = [("Independent boolean oracle", "agrees on all 160 held-out runs"),
              ("Checkpoint + message queue + RNG", "exact continuation on both vehicles"),
              ("State perturbation / source mismatch", "divergent / incompatible as expected"),
              ("Physics timestep 5 ms → 2.5 ms", "retained verdicts unchanged (2 cases)")]
    for i, (title, value) in enumerate(checks):
        y = .279 - i * .055
        fig.text(.57, y, title, fontsize=10, color=fg)
        fig.text(.57, y - .022, value, fontsize=9, color=muted)
    fig.text(.065, .026, "SCOPE: MuJoCo demonstrator • uncalibrated plants • manually authored controllers • no PX4/ArduPilot or hardware validation", fontsize=9, color=muted)
    fig.savefig(ROOT / "CRUCIBLE-v0.2.png", dpi=160, facecolor=bg)
    plt.close(fig)


def make_viewer(s):
    data = {"summary": s, "contract": read(ROOT / "contracts/mission.json"), "cases": {}}
    for model in ("rover", "multirotor"):
        directory = ROOT / "results" / model
        index = read(directory / "heldout-index.json")
        guarded = index["guarded_failsafe"]
        worst = min(range(len(guarded)), key=lambda i: guarded[i]["verdict"]["recovery"]["robustness"] if guarded[i]["verdict"]["recovery"]["robustness"] is not None else float("inf"))
        pairs = {
            "nominal": ("Nominal mission", "nominal-legacy_nav_gated.json.gz", "nominal-guarded_failsafe.json.gz"),
            "isolated": ("Isolated outage · development case", "isolated-outage-legacy_nav_gated.json.gz", "isolated-outage-guarded_failsafe.json.gz"),
            "reduced": ("Reduced search counterexample", "reduced.json.gz", "guarded-on-reduced.json.gz"),
            "stress": (f"Held-out stress case {worst:03d}", f"heldout/{worst:03d}-legacy_nav_gated.json.gz", f"heldout/{worst:03d}-guarded_failsafe.json.gz"),
        }
        data["cases"][model] = {key: {"label": label, "runs": [read(directory / a), read(directory / b)]} for key, (label, a, b) in pairs.items()}
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    html = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CRUCIBLE v0.2 · evidence explorer</title>
<style>
:root{color-scheme:dark;--bg:#0a1422;--panel:#101e30;--ink:#eef5fd;--dim:#a9bbcf;--edge:#293e56;--teal:#62dec8;--amber:#ffb579}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.6 system-ui,sans-serif}main{max-width:1200px;margin:auto;padding:46px 24px 60px}h1{font-size:clamp(38px,7vw,68px);line-height:1.1;margin:16px 0}h2{font-size:23px;margin:0 0 16px}h3{font-size:17px;margin:18px 0 6px}p{margin:8px 0 18px}.eyebrow{color:var(--teal);font-size:13px;letter-spacing:.14em;text-transform:uppercase}.lead{font-size:20px;max-width:850px;color:var(--dim)}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin:28px 0}.card,.panel{background:var(--panel);border:1px solid var(--edge);border-radius:14px;padding:22px}.card strong{display:block;font-size:34px;line-height:1.3;color:var(--teal)}.card span,.dim{color:var(--dim)}.panel{margin-top:22px}table{width:100%;border-collapse:collapse;font-size:14px}th,td{text-align:left;padding:12px 10px;border-bottom:1px solid var(--edge);vertical-align:top}th{color:var(--dim)}.scroll{overflow:auto}.note{border-left:3px solid var(--amber);padding-left:16px;color:var(--dim)}.controls{display:flex;gap:14px;flex-wrap:wrap;margin:18px 0}label{display:flex;flex-direction:column;gap:5px;font-size:13px;color:var(--dim)}select{background:var(--bg);color:var(--ink);border:1px solid #526a86;border-radius:7px;padding:10px;font-size:14px}select:focus,input:focus{outline:2px solid var(--teal);outline-offset:3px}.plot{width:100%;height:auto;min-height:240px;display:block}.legend{display:flex;gap:20px}.old{color:var(--amber)}.new{color:var(--teal)}.badge{font-size:12px;padding:2px 7px;border:1px solid currentColor;border-radius:5px;white-space:nowrap}.bad{color:var(--amber)}.good{color:var(--teal)}.columns{display:grid;grid-template-columns:1fr 1fr;gap:20px}.code{font-family:monospace;overflow-wrap:anywhere;font-size:12px}input[type=range]{width:100%;accent-color:var(--teal)}ul{padding-left:20px}a{color:var(--teal)}footer{color:var(--dim);margin-top:25px;font-size:13px}@media(max-width:720px){.cards{grid-template-columns:1fr 1fr}.columns{grid-template-columns:1fr}.panel{padding:16px}.controls{display:grid;grid-template-columns:1fr}th,td{padding:9px 6px}}
</style>
<main>
<div class="eyebrow">Version 0.2 · executable simulation evidence</div><h1>CRUCIBLE</h1>
<p class="lead">Test the mission rule. Find a failure. Strip it down. Replay it—and check that the evaluator can detect a changed run.</p>
<div class="cards"><div class="card"><strong>34</strong><span>checks passed</span></div><div class="card"><strong>160</strong><span>held-out controller runs</span></div><div class="card"><strong>6 / 6</strong><span>exact replay checks</span></div><div class="card"><strong>4 / 4</strong><span>negative controls detected</span></div></div>
<p class="note">These are uncalibrated MuJoCo demonstrations with custom controllers. They establish a runnable evaluation workflow. They do not establish flight readiness, fleet reliability, automatic bug diagnosis, or hardware validity.</p>
<section class="panel"><h2>What happened on held-out scenarios</h2><p class="dim">40 scenarios per vehicle, each run on both controllers with the same exogenous noise seed. All runs were conclusive and exercised the loss-of-link policy. Failure means any declared recovery or safety rule was violated.</p><div class="scroll"><table id="stats"><thead><tr><th>Vehicle</th><th>Controller</th><th>Violations</th><th>Failure fraction</th><th>95% Wilson interval</th></tr></thead><tbody></tbody></table></div><p class="dim" id="pairs"></p><p class="note">0 failures in 40 rover runs still leaves a 95% upper bound of 8.8% under this synthetic distribution. The guarded quadrotor still failed 27 of 40 cases. Its 5 improved pairs and 1 regressed pair do not establish statistical superiority.</p></section>
<section class="panel"><h2>Inspect the evidence</h2><p class="dim">Recorded samples at 50 Hz. Lines connect samples; safety between samples is not established. The worst guarded held-out case is selected by recovery robustness, not a representative sample.</p>
<div class="controls"><label>Vehicle<select id="model"><option value="rover">Differential-drive rover</option><option value="multirotor">Six-DOF quadrotor</option></select></label><label>Scenario<select id="case"></select></label><label>Signal<select id="signal"><option value="speed">Horizontal speed · m/s</option><option value="vertical_speed">Vertical speed · m/s</option><option value="z">Altitude · m</option><option value="post_event_travel">Displacement after link loss · m</option><option value="tilt">Tilt · rad</option><option value="nav_age">Navigation sample age · s</option><option value="est_error">Position estimation error · m</option></select></label></div>
<div class="legend"><span class="old">● Legacy navigation-gated controller</span><span class="new">● Guarded failsafe</span></div>
<svg id="chart" viewBox="0 0 1000 340" class="plot" role="img" aria-label="Recorded controller signal comparison"></svg>
<label for="tick">Inspect time sample<input type="range" id="tick" min="0" max="400" value="100" step="1"></label>
<div id="point" class="dim" aria-live="polite"></div><div class="scroll"><table id="verdicts"><thead><tr><th>Controller</th><th>Overall</th><th>Recovery</th><th>Dwell completion after loss</th><th>First safety breach</th></tr></thead><tbody></tbody></table></div>
<details><summary>Scenario parameters and evidence hashes</summary><pre id="scenario" class="code"></pre></details></section>
<section class="panel"><h2>Mission contracts</h2><p class="dim">The source, tests, models and JSON contract were hashed before the campaign. Development included nominal, long-outage and severe combined cases. Held-out seeds were generated after this internal freeze; this is not third-party preregistration.</p><div id="contracts" class="columns"></div><p class="note">Recovery requires all target predicates to hold together for the complete closed-grid dwell window, completed by the deadline. A brief threshold crossing cannot pass. Missing telemetry remains inconclusive unless a known breach already establishes a violation.</p></section>
<section class="panel"><h2>Search and reduction</h2><div class="scroll"><table id="search"><thead><tr><th>Vehicle</th><th>Method</th><th>Simulation budget</th><th>Recovery violations found</th><th>Best normalized robustness</th></tr></thead><tbody></tbody></table></div><p class="dim">One 12-attempt run per method per vehicle. These results do not establish a search-method advantage. Lower robustness means a stronger violation under the declared scales.</p><div id="reduction" class="columns"></div><p class="dim">Reduction preserves a selected recovery violation across 3 predeclared noise seeds. Cost is an editorial normalized fault-severity score, not physical risk or reliability. Greedy reduction and leave-one-hook-out ablations identify scenario sensitivity; they do not locate an unknown code defect or generate a repair.</p></section>
<section class="panel"><h2>Reproducibility with negative controls</h2><div class="scroll"><table id="replays"><thead><tr><th>Vehicle</th><th>Check</th><th>Observed grade</th></tr></thead><tbody></tbody></table></div><p class="dim">Exact replay and full-state checkpoint continuation are demonstrated only for this recorded source/environment. Deliberate state perturbations diverged. Deliberate source mismatches were rejected as incompatible. Hashes are integrity checks, not signatures or a proof of model correctness.</p><p id="grid" class="dim"></p></section>
<section class="panel"><h2>What changed—and what remains</h2><div class="columns"><div><h3>Implemented</h3><ul><li>MuJoCo rigid-body dynamics: wheel contact and a free six-DOF quadrotor with four rotor forces.</li><li>Lag, actuator limits, unequal authority, air-relative drag, wind and friction uncertainty.</li><li>Noisy delayed GNSS, stale source timestamps, outages/bias, IMU prediction and complementary correction.</li><li>Independent loss-of-link processing, stale/innovation guards and explicit degraded modes.</li><li>Finite-trace monitor plus separately implemented boolean oracle and analytic physics checks.</li></ul></div><div><h3>Open validation work</h3><ul><li>Fit plants, sensors and fault distributions to measured vehicle data.</li><li>Integrate an actual PX4/ArduPilot controller; neither is connected here.</li><li>Replace synthetic attitude aiding with a validated sensor/AHRS model.</li><li>Expand return, land and continue policies; only rover STOP and quadrotor HOLD are implemented.</li><li>Improve quadrotor control, validate on fresh held-out scenarios, then move to SITL/HIL and controlled physical testing.</li></ul></div></div></section>
<footer><div id="provenance" class="code"></div><p>Sources: <a href="https://mujoco.readthedocs.io/en/stable/python.html">MuJoCo Python</a> · <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html">MuJoCo dynamics and reproducibility</a> · <a href="https://docs.px4.io/main/en/debug/failure_injection">PX4 failure-injection support</a>. All viewer data is embedded; no external scripts or network requests.</p></footer>
</main><script id="data" type="application/json">__PAYLOAD__</script>
<script>
'use strict';
const D=JSON.parse(document.getElementById('data').textContent),$=id=>document.getElementById(id),S=D.summary;
const names={rover:'Rover',multirotor:'Quadrotor'},variants=['legacy_nav_gated','guarded_failsafe'];
const labels=['Legacy','Guarded'],colors=['#ffb579','#62dec8'];
const pct=n=>(n*100).toFixed(1)+'%',f=n=>Number(n).toFixed(3);
const badge=s=>'<span class="badge '+(s==='SAT'||s==='EXACT'?'good':'bad')+'">'+s+'</span>';
for(const m of ['rover','multirotor']){
 const M=S.models[m];variants.forEach((v,i)=>{const x=M.heldout[v];$('stats').tBodies[0].insertAdjacentHTML('beforeend',`<tr><td>${names[m]}</td><td>${labels[i]}</td><td>${x.counts.VIOLATED} / ${x.attempts}</td><td>${pct(x.failure_fraction)}</td><td>${x.wilson_95.map(pct).join(' – ')}</td></tr>`)});
 for(const [method,x]of Object.entries(M.search))$('search').tBodies[0].insertAdjacentHTML('beforeend',`<tr><td>${names[m]}</td><td>${method.replace('_',' ')}</td><td>${x.budget}</td><td>${x.recovery_violations}</td><td>${f(x.best_score)}</td></tr>`);
 const c=D.contract.models[m];$('contracts').insertAdjacentHTML('beforeend',`<div><h3>${names[m]} · ${c.policy}</h3><p>Deadline: <strong>${c.deadline_s} s</strong> after loss · dwell <strong>${c.dwell_s} s</strong>.</p><ul>${c.targets.map(x=>`<li>${x.signal.replaceAll('_',' ')} ${x.op==='le'?'≤':'≥'} ${x.bound} ${x.unit}</li>`).join('')}</ul><p class="dim">${c.budget_reason}</p></div>`);
 const r=M.reduction;$('reduction').insertAdjacentHTML('beforeend',`<div><h3>${names[m]}</h3><p>Fault cost ${f(r.initial_cost)} → ${f(r.final_cost)} · <strong>${pct(r.fraction_reduced)} lower</strong>.<br>${r.simulations} simulations used of ${r.simulation_budget}; 3 confirmation seeds.</p><p>Guarded controller on reduced case: ${badge(M.guarded_on_reduced.verdict.overall)}</p></div>`);
 for(const [check,x]of Object.entries(M.replay))$('replays').tBodies[0].insertAdjacentHTML('beforeend',`<tr><td>${names[m]}</td><td>${check.replaceAll('_',' ')}</td><td>${badge(x.grade)}</td></tr>`);
}
$('pairs').textContent='Paired outcomes — rover: 8 legacy violations became passes, no regressions. Quadrotor: 5 legacy violations became passes, 1 legacy pass became a violation; 26 pairs violated both.';
$('grid').textContent=['rover','multirotor'].map(m=>{const x=S.models[m].timestep_sensitivity;return `${names[m]} retained case: halving physics timestep left verdict unchanged; max speed difference ${f(x.max_speed_difference_mps)} m/s, max altitude difference ${(x.max_altitude_difference_m*1000).toFixed(2)} mm.`}).join(' ');
$('provenance').textContent=`Frozen ${S.freeze.created_utc} | MuJoCo ${S.freeze.environment.mujoco} | NumPy ${S.freeze.environment.numpy} | Python ${S.freeze.environment.python}\nSource SHA-256 ${S.freeze.source_hash}\nContract SHA-256 ${S.freeze.contract_hash}`;
let current;
function populate(){const m=$('model').value;$('case').innerHTML=Object.entries(D.cases[m]).map(([key,x])=>`<option value="${key}">${x.label}</option>`).join('');draw()}
function draw(){
 const m=$('model').value,key=$('case').value,signal=$('signal').value;current=D.cases[m][key];const runs=current.runs,rows=runs.map(r=>r.rows);
 const c=D.contract.models[m],limits=c.targets.filter(r=>r.signal===signal),bounds=c.invariants.filter(r=>r.signal===signal);
 let max=Math.max(...rows.flatMap(r=>r.map(x=>x[signal])),...limits.map(x=>x.bound),...bounds.map(x=>x.bound),.1),min=Math.min(...rows.flatMap(r=>r.map(x=>x[signal])),0);
 const pad=(max-min)*.12;max+=pad;min-=pad*.2;const X=t=>62+t/8*905,Y=v=>290-(v-min)/(max-min)*260;
 let svg='';for(let j=0;j<=4;j++){const v=min+(max-min)*j/4,y=Y(v);svg+=`<line x1="62" y1="${y}" x2="967" y2="${y}" stroke="#293e56"/><text x="54" y="${y+4}" fill="#a9bbcf" font-size="12" text-anchor="end">${v.toFixed(2)}</text>`}
 for(let t=0;t<=8;t+=2)svg+=`<text x="${X(t)}" y="315" fill="#a9bbcf" font-size="12" text-anchor="middle">${t}s</text>`;
 for(const r of limits)svg+=`<line x1="62" y1="${Y(r.bound)}" x2="967" y2="${Y(r.bound)}" stroke="#62dec8" stroke-dasharray="6 5"/><text x="966" y="${Y(r.bound)-6}" fill="#62dec8" font-size="12" text-anchor="end">target ${r.bound}</text>`;
 const event=D.contract.event_s,deadline=event+c.deadline_s;for(const [t,label]of [[event,'loss'],[deadline,'deadline']])svg+=`<line x1="${X(t)}" y1="25" x2="${X(t)}" y2="290" stroke="#a9bbcf" stroke-dasharray="3 5"/><text x="${X(t)+5}" y="19" fill="#a9bbcf" font-size="12">${label}</text>`;
 rows.forEach((r,i)=>{svg+=`<polyline points="${r.map(x=>X(x.t).toFixed(2)+','+Y(x[signal]).toFixed(2)).join(' ')}" fill="none" stroke="${colors[i]}" stroke-width="2.2"/>`});
 const tick=+$('tick').value;svg+=`<line x1="${X(rows[0][tick].t)}" x2="${X(rows[0][tick].t)}" y1="25" y2="290" stroke="#eef5fd" opacity=".6"/>`;$('chart').innerHTML=svg;
 $('point').textContent=`t = ${f(rows[0][tick].t)} s · ${$('signal').selectedOptions[0].textContent} · Legacy ${f(rows[0][tick][signal])} (${rows[0][tick].mode}) · Guarded ${f(rows[1][tick][signal])} (${rows[1][tick].mode})`;
 $('verdicts').tBodies[0].innerHTML=runs.map((r,i)=>{const v=r.verdict,breach=v.invariants.filter(x=>x.status==='VIOLATED').sort((a,b)=>a.first_breach_s-b.first_breach_s)[0];return `<tr><td>${labels[i]}</td><td>${badge(v.overall)}</td><td>${badge(v.recovery.status)}</td><td>${v.recovery.response_s===null?'No qualifying window':f(v.recovery.response_s)+' s'}</td><td>${breach?breach.rule.signal+' at '+f(breach.first_breach_s)+' s':'None observed'}</td></tr>`}).join('');
 $('scenario').textContent=JSON.stringify({scenario:runs[0].scenario,legacy_trace_sha256:runs[0].trace_hash,guarded_trace_sha256:runs[1].trace_hash},null,2);
}
$('model').addEventListener('change',populate);$('case').addEventListener('change',draw);$('signal').addEventListener('change',draw);$('tick').addEventListener('input',draw);populate();
</script></html>'''
    (ROOT / "CRUCIBLE-v0.2-Explorer.html").write_text(html.replace("__PAYLOAD__", payload))


def make_readme(s):
    rows = []
    for m in ("rover", "multirotor"):
        x = s["models"][m]["heldout"]
        rows.append(f"| {m} | {x['legacy_nav_gated']['counts']['VIOLATED']}/40 | {x['guarded_failsafe']['counts']['VIOLATED']}/40 | {x['guarded_failsafe']['wilson_95'][0]:.1%}–{x['guarded_failsafe']['wilson_95'][1]:.1%} |")
    text = '''# CRUCIBLE v0.2 — physics and evidence

CRUCIBLE exercises a complete fault-evaluation workflow on two MuJoCo vehicle demonstrators: define a mission rule, run fault scenarios, search for violations, reduce a counterexample, compare manually authored controllers, save evidence, and replay it with negative controls.

**This is executable simulation research, not validated autonomy, an automatic repair engine, or flight-ready software.** The quadrotor still fails many stress cases. v0.1 remains available in the parent repository and its original archive.

![Measured v0.2 results](CRUCIBLE-v0.2.png)

Open [the offline evidence explorer](CRUCIBLE-v0.2-Explorer.html) locally after downloading; all data and scripts are embedded. No server or network connection is needed. This report and chart use recorded data, not generated imagery.

## Recorded results

40 held-out scenarios per model × 2 controllers = **160 runs**. Each pair uses the same scenario and exogenous noise seed. All 160 runs were conclusive/exercised and the quantitative monitor agreed with the separately implemented boolean oracle. These fractions describe only the declared synthetic distribution.

| Model | Legacy violations | Guarded violations | Guarded 95% Wilson interval |
| --- | ---: | ---: | --- |
__ROWS__

The rover has 8 improved pairs and no regressed pairs. The quadrotor has 5 improved pairs, 1 regressed pair, 26 pairs violating both and 8 pairs passing both. These small paired results do not establish statistical superiority. Zero rover failures does not establish zero risk: its interval extends to 8.8% under this distribution.

**34 checks passed**, including a 162-case expected-outcome recovery boundary sweep. Checks include independent Boolean/quantitative monitor comparisons, empty/missing/nonfinite telemetry, preserved known safety breaches, analytic free fall, hover force balance, actual wheel contact, force limits, seeded link-processing behavior, checkpoint persistence, corruption detection and accounting for failed search attempts.

6/6 replay checks were exact (two saved evidence files plus one serialized full-state checkpoint per model). All 4 negative controls were detected: state perturbation caused divergence and source mismatch was incompatible on each model. Hashes are not signatures; repeatability and agreement do not prove correctness or physical validity.

## Run it

Recorded environment: Python 3.12.14, MuJoCo 3.16.0, NumPy 2.5.4 on Linux x86_64. Other environments can run the models, but exact replay is checked only when the recorded environment/source/contract match. Install dependencies from official PyPI in an isolated environment:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m crucible validate
python -m crucible replay results/rover/reduced.json.gz
python -m crucible replay results/multirotor/reduced.json.gz
python -m crucible resume results/rover/checkpoint.json.gz --output resumed.json.gz
python -m crucible campaign --output results-new
```

The campaign refuses to overwrite an existing `FREEZE.json`. Use a fresh output directory. `python -m crucible run --model multirotor --variant guarded_failsafe --output run.json.gz` runs a nominal mission. Pass `--scenario` with a Scenario JSON or saved evidence file to rerun a scenario. `--contract /path/mission.json` precedes the subcommand for another compatible contract. Only supported rover STOP / quadrotor HOLD policy bindings are accepted; RETURN, LAND and CONTINUE are not implemented.

`render_report.py` optionally regenerates this README, overview and HTML from recorded `results/summary.json`; it needs Matplotlib. It is outside the frozen evaluator and makes no simulation changes. The viewer works without Python.

## What the models actually contain

| Layer | Rover | Quadrotor |
| --- | --- | --- |
| Plant | 6.76 kg free chassis, two hinged wheels, two low-friction support spheres, actual ground contact | 1.5 kg free rigid body, all 6 pose/velocity DOFs, 4 offset rotor thrust sites and yaw torques, ground contact |
| Actuation | Individual bounded wheel torque, exact first-order activation filter | Individual 0–7.5 N rotor force, exact first-order activation filter, clipped mixer |
| Disturbance | Tire-friction uncertainty; simple air-relative drag and finite wind interval | Simple quadratic air-relative drag, angular damping and finite wind interval |
| Sensors | 10 Hz GNSS with source timestamps, delayed queue, dropout/stale/bias hooks; 50 Hz simulated IMU/attitude aid and wheel odometry | Same GNSS and simulated IMU/attitude aiding; no wheel odometry |
| Estimation | IMU prediction + complementary GNSS/odometry correction | IMU prediction + complementary GNSS correction |
| Controller | Cruise → STOP / DEGRADED_STOP | Cruise → HOLD / DEGRADED_HOLD, geometric attitude PD and bounded disturbance integral |

World frame: ENU metres/seconds/radians, body x-forward y-left z-up. The accelerometer is derived from consecutive truth velocities, transformed to body specific force and given noise/bias. **Attitude aiding is generated from true attitude plus noise, not an AHRS implementation.** Controller inputs use the simulated sensor bus; monitors use privileged simulator truth. This privileged truth is an explicit simulation assumption and is unavailable directly on a vehicle.

Actuator degradation scales one selected actuator's force/torque gain over [1.8,6.0) s. Activation lag and tire friction are plant uncertainties present from startup. Wind is an air-velocity field over [2.0,6.0) s. Navigation faults act on 10 Hz source samples over [1.8,1.8+duration) s. Stale faults preserve the old source timestamp; arrival time is never substituted. Bias changes world-x GNSS position only. GNSS transport latency is constant per run. Operator-link loss is an independent latched event at 2.0 s, not a navigation failure.

The legacy controller intentionally retains a known defect: it processes link loss only when navigation updates. The guarded variant processes it independently, rejects old/repeated/high-innovation fixes and adds a small bounded quadrotor integral. **These controller changes were manually authored.** A comparison measures their behavior; it is not automatic code diagnosis or synthesis of a repair. The legacy baseline is a deliberately defective demonstration baseline, not a commercial autopilot.

## Mission requirements and monitor semantics

`contracts/mission.json` expresses policy bindings, frame/units, target and safety bounds, normalization scales, response deadline, dwell time, controller parameters and campaign distribution. Each deadline includes a written illustrative budget rationale. **A mission owner must justify/approve real vehicle requirements; none of these values are field standards.**

- Rover: finish a 0.4 s dwell at horizontal speed ≤0.2 m/s within 2.6 s of link loss; safety limits speed, tilt and horizontal displacement from the loss position.
- Quadrotor: finish a 0.5 s dwell within 3.0 s of loss, simultaneously satisfying horizontal speed ≤0.3 m/s, vertical speed ≤0.25 m/s, altitude error ≤0.35 m and horizontal displacement from actual loss position ≤1.5 m. Safety limits altitude, tilt and a wider 6 m containment radius.
- A dwell window includes both sampled endpoints. Its completion must be at or before the deadline. Safety invariants apply over the entire observed 8 s trace, including after recovery.
- SAT needs the required witness and complete safety telemetry. A complete trace with no trigger is NOT_EXERCISED. A missing target sample can leave recovery INCONCLUSIVE; an already witnessed safety breach remains VIOLATED even if other data is absent or invalid. Empty traces never pass.
- The quantitative recovery score is the maximum, over eligible dwell windows ending by the deadline, of the minimum normalized predicate margin in that window. Safety scores are observed minimum normalized margins. They are **sampled** robustness, not continuous-time guarantees or a cross-vehicle risk scale.
- The Boolean oracle uses direct comparisons and independently implemented indexing/window loops. It shares the contract, not the quantitative margin implementation. Hand-specified expected boundary cases and analytic physics checks provide additional reference points; agreement is still not a proof.

Control/monitor samples are 20 ms; physics steps are 5 ms. Numerical sensitivity halves physics steps to 2.5 ms on each retained reduced case. Both retained verdicts were unchanged; rover max speed difference was 0.0217 m/s, quadrotor 0.00183 m/s. This is only a two-case sensitivity experiment.

## Search, reduction and held-out design

`results/FREEZE.json` records the full contract, environment and SHA-256 manifest for the evaluator, tests, plant XMLs and pinned requirements **before generating held-out scenarios**. Nominal, long-dropout and severe combined development examples were inspected beforehand; this is an internally frozen campaign, not external preregistration. No controller/threshold tuning followed inspection of held-out results.

Search compares 12 pseudorandom attempts with 12 shifted-Halton attempts per model, fixed seeds and identical ranges. Every attempt consumes budget. Only conclusive exercised recovery results have an eligible objective. Search optimizes the legacy controller's recovery robustness. One run per method is insufficient to establish algorithm superiority; all attempt ledgers are saved.

Reduction deletes hook groups then weakens them in a fixed greedy order. Each candidate is checked on seeds 4101/4102/4103, preserving `recovery VIOLATED` on all three. Confirmation and final materialization simulations count toward the 72-run cap. The resulting rover cost fell 84.3% in 34 simulations; quadrotor cost fell 76.3% in 31. The reduced rover failure is a 2.1 s navigation dropout. The reduced quadrotor failure is a wind-only recovery failure, and the guarded controller **also fails it**. Neither is a global minimum. Leave-one-hook-out ablations measure scenario sensitivity, not defect location.

Fault cost is a declared, dimensionless editorial sum: navigation duration/2.4 (multiplied by absolute bias/2.5 for bias faults), lost authority/0.55, wind magnitude/5, friction reduction/0.65, added latency/0.2, added lag/1.5, absolute IMU bias/0.06 and absolute initial yaw/0.08. The fixed link-loss trigger is excluded. It is not physical damage, reliability improvement, or a metric to compare across vehicles.

Held-out sampling uses a separate PRNG, seeds 87001 (rover) and 97001 (quadrotor), 40 draws each, after freezing. Coordinates are independent uniform values over the JSON ranges; wind has uniform magnitude/azimuth, navigation kind and weakened actuator index are categorical. Scenario noise seeds are 870010+i and common to each controller pair. All raw held-out traces, parameters, verdicts and hashes are retained. Confidence intervals use only conclusive exercised runs; invalid/incomplete/not-exercised counts are reported separately rather than dropped.

## Replay and evidence layout

`results/{rover,multirotor}/` contains compressed JSON evidence for nominal, original/reduced search cases, guard-on-reduced, isolated-outage development comparisons, no-trigger/short traces and every held-out run; plus search/reduction/ablation/replay/timestep ledgers and held-out indexes. `VALIDATION.json` stores the full check log. `SHA256SUMS.json` covers packaged files.

Replay grades are EXACT, TOLERANT (declared 1e-8 absolute/relative tolerance with equal verdict data), DIVERGENT or INCOMPATIBLE. Exact hashes cover canonical complete sample rows. Checkpoint JSON stores MuJoCo's full integration state including warmstart, current model actuator gains, controller/estimator state, delayed messages, source timestamps, RNG state and evidence prefix. It uses no pickle. A stored environment/source/contract mismatch blocks an exact replay claim. Replay is version/architecture dependent; portability is not asserted.

## Limits and next validation gates

The dynamics, airframe, drag coefficients, sensor model and scenario distribution are illustrative and unfitted. There is no aerodynamic rotor model, propeller interaction, battery, structural flexibility, terrain map, obstacle planning, realistic radio stack, AHRS, production flight stack, hardware-in-the-loop or flight test. Simplified support spheres are not a validated caster model. Discrete monitoring can miss between-sample violations. A common modeling or specification mistake can survive oracle agreement and exact replay.

The next evidence-bearing steps are measured plant/sensor identification, requirements signed off by a mission owner, a real autopilot adapter with verified fault-hook semantics, unseen faults/longer missions, improved fault-tolerant quadrotor control evaluated on a **fresh** held-out set, then SITL/HIL and controlled physical validation. PX4 fault injection is environment-dependent; an accepted command alone does not demonstrate that a fault was applied.

## Primary references

- [MuJoCo official Python bindings and installation](https://mujoco.readthedocs.io/en/stable/python.html)
- [MuJoCo dynamics, integration state and reproducibility](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo actuator/site transmission XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [PX4 failure injection and actual support limitations](https://docs.px4.io/main/en/debug/failure_injection)

Automated falsification and counterexample analysis are established techniques. This release claims an inspectable implementation and bounded recorded results, not novelty of those methods.
'''
    (ROOT / "README.md").write_text(text.replace("__ROWS__", "\n".join(rows)))


if __name__ == "__main__":
    summary = read(ROOT / "results/summary.json")
    make_figure(summary)
    make_viewer(summary)
    make_readme(summary)
    print("Created CRUCIBLE-v0.2.png, CRUCIBLE-v0.2-Explorer.html and README.md")
