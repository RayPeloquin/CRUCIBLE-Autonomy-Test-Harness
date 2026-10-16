#!/usr/bin/env python3
"""Render recorded CRUCIBLE pilot data. Matplotlib is optional, report-only."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def render(results, output):
    summary = json.loads((results / "summary.json").read_text())
    validation = json.loads((Path(__file__).parent / "VALIDATION.json").read_text())
    bg, ink, muted, grid = "#07111c", "#eef6ff", "#a6b8c9", "#273746"
    colors = {"baseline": "#93a5b7", "original": "#ffad66", "reduced": "#d5a6ff", "repaired": "#58ddba"}
    labels = {"baseline": "Link loss only", "original": "Original failure", "reduced": "Reduced failure", "repaired": "Repaired controller"}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "text.color": ink,
                         "axes.labelcolor": muted, "xtick.color": muted, "ytick.color": muted,
                         "axes.edgecolor": grid, "axes.titlecolor": ink})
    fig = plt.figure(figsize=(14, 11), facecolor=bg)
    fig.text(.065, .957, "CRUCIBLE", fontsize=30, weight="bold", va="top")
    fig.text(.065, .914, "FIRST EXECUTABLE PILOT  /  SHARED EVALUATOR, TWO SIMPLE MODELS", fontsize=12, color="#9de5f0")
    fig.text(.065, .887, "A deliberately seeded control defect, fault reduction, and independently rerun evidence.", color=muted, fontsize=11)
    stats = [("2", "machine models"), ("2", "seeded failures found"), ("6 / 6", "exact replay matches"),
             (f"{validation['tests_passed']} / {validation['tests_run']}", "checks passed")]
    for x, (value, label) in zip((.065, .30, .535, .77), stats):
        fig.text(x, .845, value, fontsize=24, weight="bold")
        fig.text(x, .818, label, fontsize=10, color=muted)
    fig.legend(handles=[Line2D([0], [0], color=colors[key], lw=2.5, label=labels[key]) for key in colors],
               loc="upper center", bbox_to_anchor=(.51, .796), ncol=4, frameon=False,
               labelcolor=ink, fontsize=10, handlelength=2.2)
    spec = fig.add_gridspec(2, 2, left=.065, right=.955, top=.728, bottom=.235,
                            hspace=.55, wspace=.24, height_ratios=[2.2, 1])
    for col, (machine, title) in enumerate((("rover", "ROVER"), ("multirotor", "MULTIROTOR"))):
        cases = {}
        for name in colors:
            cases[name] = json.loads((results / machine / (name + ".json")).read_text())["case"]
        config = cases["baseline"]["config"]
        ax = fig.add_subplot(spec[0, col], facecolor=bg)
        for name in colors:
            case = cases[name]
            origin = case["evaluation"]["rules"]["operator_loss_response"]["windows"][0]["trigger_tick"] * .02
            ax.plot([row["t_s"] - origin for row in case["trace"]],
                    [row["metrics"]["horizontal_speed_mps"] for row in case["trace"]],
                    color=colors[name], lw=2.4 if name != "baseline" else 1.7,
                    linestyle="--" if name == "baseline" else "-", zorder=3)
            event = case["evaluation"]["rules"]["operator_loss_response"]["windows"][0]
            target_row = case["trace"][event["trigger_tick"] + event["latency_ticks"]]
            ax.scatter([event["latency_s"]], [target_row["metrics"]["horizontal_speed_mps"]],
                       s=35, color=colors[name], edgecolors=bg, linewidths=.8, zorder=5)
        ax.axvline(0, color=muted, alpha=.5, lw=.8)
        ax.axvline(config["deadline_s"], color=ink, linestyle=":", lw=1.4)
        ax.axhline(config["target_speed_mps"], color="#7dcbd9", lw=.9, alpha=.8)
        ax.text(.98, .16, f"Speed target ≤ {config['target_speed_mps']:.2f} m/s", transform=ax.transAxes,
                ha="right", color="#7dcbd9", fontsize=8)
        ax.text(config["deadline_s"] + .04, config["initial_speed_mps"] * .94,
                f"Deadline\n{config['deadline_s']:.2f} s", fontsize=9, color=ink)
        ax.set_title(title + ("  /  ground translation" if machine == "rover" else "  /  pitch + x/z motion"),
                     loc="left", fontsize=12, weight="bold", pad=14)
        ax.set(xlim=(-.20, 2.2), ylim=(-.04, config["initial_speed_mps"] * 1.12),
               xlabel="Time after operator-link loss (s)", ylabel="Horizontal speed (m/s)")
        ax.grid(axis="y", color=grid, alpha=.7, linewidth=.6)
        ax.spines[["top", "right"]].set_visible(False)
        latency = lambda name: cases[name]["evaluation"]["rules"]["operator_loss_response"]["windows"][0]["latency_s"]
        ax.text(0, -.30, f"Original {latency('original'):.2f} s  ·  Reduced {latency('reduced'):.2f} s  ·  Repaired {latency('repaired'):.2f} s",
                transform=ax.transAxes, fontsize=9, color=ink)
        timeline = fig.add_subplot(spec[1, col], facecolor=bg)
        position = timeline.get_position()
        timeline.set_position([position.x0 + .09, position.y0, position.width - .09, position.height])
        for y, name, kind in ((3, "original", "navigation_freeze"), (2, "original", "braking_derate"),
                              (1, "reduced", "navigation_freeze"), (0, "reduced", "braking_derate")):
            fault = next(f for f in cases[name]["faults"] if f["kind"] == kind)
            start = fault["start"] * .02 - 2
            duration = fault["duration"] * .02
            timeline.barh(y, duration, left=start, height=.5, color=colors[name], alpha=.9)
            timeline.text(start + duration + .04, y, f"{duration:.2f} s", va="center", fontsize=9, color=colors[name])
        timeline.axvline(0, color=muted, alpha=.5, lw=.8)
        timeline.set(xlim=(-.2, 3.6), ylim=(-.6, 3.6), xlabel="Fault timing relative to link loss (s)")
        timeline.set_yticks([3, 2, 1, 0], ["Original: nav freeze", "Original: derate", "Reduced: nav freeze", "Reduced: derate"], fontsize=9)
        timeline.spines[["top", "right", "left"]].set_visible(False)
        timeline.tick_params(axis="y", length=0)
    fig.text(.065, .161, "THE REDUCED CASE STILL FAILS", fontsize=12, weight="bold", color="#d5a6ff")
    fig.text(.065, .133, "Both machines: 1.42 s response against an illustrative 1.40 s deadline. Repair: 0.78 s.", fontsize=11)
    fig.text(.065, .105, "Fault cost: 1.950 → 1.209 severity-weighted seconds (38% lower). 15 search trials + 84 reduction trials per model.", fontsize=10, color=muted)
    fig.text(.065, .077, "Each added disturbance alone passes. Missing triggers and early termination remain separate outcomes.", fontsize=10, color=muted)
    fig.text(.065, .038, "SYNTHETIC DEMONSTRATOR  ·  20 ms sampled reach-by checks  ·  No real controller, hardware or flight-stack validation", fontsize=9, color="#ffbd86")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor=bg)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", type=Path, default=Path("results"))
    p.add_argument("--output", type=Path, default=Path("CRUCIBLE-Pilot-v0.1.png"))
    a = p.parse_args()
    render(a.results, a.output)
