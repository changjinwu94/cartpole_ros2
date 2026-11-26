#!/usr/bin/env python3
"""Run bounded velocity swing trials and score near-upright entry from their JSON."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

RECORDING = "RECORDING: /logs/"


def score(report, inertia, mg):
    summary = report["summary"]
    entry = summary.get("entry_estimate")
    push = next((e for e in report["events"] if e.get("reason") == "reduced_final_push"), None)
    samples = [e for e in report["events"] if e.get("kind") == "velocity_command"
               and (push is None or e["host_monotonic_ns"] >= push["host_monotonic_ns"])]
    # At equal encoder angles, prefer the sample closest to standstill.
    closest = min(samples, key=lambda e: (abs(e["pole_rad"]), abs(e["pole_rad_s"])), default=None)
    row = {
        "outcome": summary.get("outcome"),
        "closest_upright_deg": summary.get("closest_upright_deg"),
        "stop_confirmed": summary.get("stop_confirmed"),
        "final_push_fired": push is not None,
        "final_push_direction": push.get("direction") if push else None,
        "final_push_energy_fraction": push["energy_fraction"] if push else None,
        "final_push_pole_rate_rad_s": push["pole_rate_rad_s"] if push else None,
        "final_push_planned_displacement_mm": (
            1000 * (push["stroke_end_m"] - push["estimated_cart_m"]) if push else None
        ),
        "closest_sample_angle_deg": None, "closest_sample_rate_rad_s": None,
        "closest_sample_energy_fraction": None, "closest_sample_cart_travel_mm": None,
        "score_source": None, "score_energy_fraction": None, "score_energy_error": None,
        "entry_angle_deg": None, "entry_rate_rad_s": None, "energy_fraction": None,
        "energy_error": None, "predicted_upright_rate_rad_s": None, "near_zero_sample": False,
    }
    if closest is not None:
        theta, rate = closest["pole_rad"], closest["pole_rad_s"]
        energy = (1 + math.cos(theta)) / 2 + inertia * rate**2 / (4 * mg)
        row.update(closest_sample_angle_deg=math.degrees(theta), closest_sample_rate_rad_s=rate,
                   closest_sample_energy_fraction=energy,
                   closest_sample_cart_travel_mm=(
                       1000 * (closest["cart_m"] - push["current_cart_m"]) if push else None
                   ),
                   score_source="closest_sample", score_energy_fraction=energy,
                   score_energy_error=abs(energy - 1),
                   near_zero_sample=any(abs(e["pole_rad"]) <= math.radians(5)
                                        and abs(e["pole_rad_s"]) <= 0.5 for e in samples))
    if entry is None:
        return row
    theta = entry["phi_rad"] % math.tau - math.pi
    rate, energy = entry["pole_rate_rad_s"], entry["energy_fraction"]
    if not all(map(math.isfinite, (theta, rate, energy))):
        return row
    row.update(entry_angle_deg=math.degrees(theta), entry_rate_rad_s=rate,
               energy_fraction=energy, energy_error=abs(energy - 1),
               score_source="entry", score_energy_fraction=energy, score_energy_error=abs(energy - 1),
               near_zero_sample=row["near_zero_sample"] or
               (abs(theta) <= math.radians(5) and abs(rate) <= 0.5))
    if theta * rate < 0 and energy >= 1:
        row["predicted_upright_rate_rad_s"] = math.sqrt((energy - 1) * 4 * mg / inertia)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fractions", type=float, nargs="*",
                        default=[0.3044, 0.285, 0.265, 0.245, 0.3044])
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--triggers", type=float, nargs="+", default=[0.83],
                        help="One final-push trigger for all trials, or one per fraction.")
    args = parser.parse_args()
    if len(args.triggers) not in (1, len(args.fractions)) or not all(0 < e <= 1 for e in args.triggers):
        parser.error("--triggers needs one value or one per fraction, each in (0, 1].")
    triggers = args.triggers * len(args.fractions) if len(args.triggers) == 1 else args.triggers
    if not 1 <= len(args.fractions) <= 10 or not all(0 < f <= 1 for f in args.fractions):
        parser.error("Provide 1 to 10 final-push fractions in (0, 1].")

    repo = args.repo.resolve()
    launcher = repo / "cartpole_batch/cartpole_final_push_trial.sh"
    if not launcher.is_file() or not (repo / "logs").is_dir():
        parser.error("Run from the cartpole checkout or pass --repo.")
    import sys
    sys.path.insert(0, str(repo / "experiments/cartpole_position_energy_v4"))
    from energy_core import INERTIA, MG

    out = Path(tempfile.mkdtemp(prefix="velocity_entry_sweep.", dir=repo / "logs"))
    print(f"Results: {out / 'results.csv'}", flush=True)
    env = os.environ | {"CARTPOLE_REPO": str(repo), "CARTPOLE_BATCH": "1"}
    ranked = []
    with (out / "results.csv").open("w", newline="") as output:
        writer = None
        for index, (fraction, trigger) in enumerate(zip(args.fractions, triggers), 1):
            command = ["bash", str(launcher), "--velocity", str(fraction), str(trigger), "8", "0.275", "3.5"]
            print(f"Trial {index}/{len(args.fractions)} fraction={fraction} trigger={trigger}", flush=True)
            recordings = []
            with (out / f"trial_{index:02}.txt").open("w") as transcript:
                proc = subprocess.Popen(command, cwd=repo, env=env, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for line in proc.stdout:
                    transcript.write(line)
                    print(line, end="", flush=True)
                    if line.startswith(RECORDING):
                        recordings.append(repo / "logs" / Path(line[len(RECORDING):].strip()).name)
                status = proc.wait()
            report = None
            for path in reversed(recordings):
                data = json.loads(path.read_text())
                if data["summary"].get("motion_attempted") is True:
                    report = data
                    break
            if report is None:
                raise SystemExit(f"Sweep stopped: no execution log for trial {index}. See {out}")
            row = dict(trial=index, fraction=fraction, trigger=trigger, recording=str(path), **score(report, INERTIA, MG))
            if writer is None:
                writer = csv.DictWriter(output, fieldnames=row)
                writer.writeheader()
            writer.writerow(row)
            output.flush()
            if status or not report["summary"].get("completed") or row["stop_confirmed"] is not True:
                raise SystemExit(f"Sweep stopped after an error or unconfirmed stop. See {out}")
            if row["final_push_fired"] and row["closest_sample_angle_deg"] is not None:
                ranked.append(row)

    if ranked:
        best = min(ranked, key=lambda r: (not r["near_zero_sample"], r["score_energy_error"],
                                          abs(r["closest_upright_deg"] or 0)))
        print(f"Best candidate: fraction={best['fraction']}, trigger={best['trigger']}, "
              f"{best['score_source']} energy={best['score_energy_fraction']:.6f}, "
              f"closest={best['closest_sample_angle_deg']:.2f} deg, "
              f"rate={best['closest_sample_rate_rad_s']:.3f} rad/s.")
    else:
        print("No trial fired the final push and produced scorable telemetry.")
    print(f"Results: {out / 'results.csv'}")


if __name__ == "__main__":
    main()
