#!/usr/bin/env python3
"""Classify recorded battle outcomes without changing vehicle behaviour.

Read previous stage1/defense case logs and their Gazebo logs. A Gazebo
SetWorldPoseCmd replay has no verified physical wall-contact measurement.
"""
import argparse
import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RESULT_RE = re.compile(r"\[BATTLE\] RESULT ([A-Z][A-Z0-9_]*)")
PAIR_RE = re.compile(r"\[BATTLE\] COLLISION[^\n]*?pair=([^\s]+)")
GAP_RE = re.compile(r"\[BATTLE\] CLEARANCE_BREACH[^\n]*?pair=([^\s]+)[^\n]*?actual=([0-9.]+)[^\n]*?required=([0-9.]+)")
DEFENSE_RE = re.compile(r"defense_status=([A-Z_]+)")

def newest_summary(folder):
    entries = list((ROOT / "log" / folder).glob("*/summary.csv"))
    return max(entries, key=lambda p: p.parent.name) if entries else None

def text_file(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""

def classify(case_log, rc):
    log = text_file(case_log)
    # run_battle.sh prints the prepared CSV path even if its result loop fails.
    match = re.search(r"^Log:\s+(.+/prepared/battle_state\.csv)\s*$", log, re.M)
    gzlog = Path(match.group(1)).parent.parent / "gazebo.log" if match else None
    gazebo = text_file(gzlog) if gzlog else ""
    # The Gazebo log is authoritative for all incident lines; fallback
    # to batch-case stdout when the original directory is unavailable.
    complete = gazebo + "\n" + log
    results = RESULT_RE.findall(complete)
    code = results[-1] if results else ""
    contacts = PAIR_RE.findall(complete)
    breaches = GAP_RE.findall(complete)
    status = DEFENSE_RE.findall(complete)
    defense_status = status[-1] if status else "UNKNOWN"
    pair = contacts[-1] if contacts else (breaches[-1][0] if breaches else "")
    detail = ""
    if code == "PASS_BATTLE":
        category = "SUCCESS"
        detail = "Overtake and defense criteria satisfied"
    elif code == "PARTIAL_DEFENSE_UNVERIFIED":
        category = "TASK_DEFENSE_UNVERIFIED"
        detail = "Overtake completed; rear-defense criterion not verified"
    elif code == "FAIL_NO_OVERTAKE":
        category = "TASK_NO_OVERTAKE"
        detail = "Lap completed without a verified overtake"
    elif code == "FAIL_COLLISION":
        category = "VEHICLE_COLLISION"
        detail = "TEAM11 vehicle collision, pair=" + (pair or "UNKNOWN")
    elif code == "FAIL_SCENARIO_COLLISION":
        category = "PRACTICE_VEHICLE_COLLISION"
        detail = "Collision between scripted practice vehicles, pair=" + (pair or "UNKNOWN")
    elif code == "FAIL_CLEARANCE":
        category = "VEHICLE_CLEARANCE_BREACH"
        detail = "Body clearance below required minimum"
        if breaches:
            _, actual, required = breaches[-1]
            detail += f", measured={actual}m required={required}m"
    elif code == "FAIL_TIMEOUT":
        category = "SIMULATION_TIMEOUT"
        detail = "Replay simulation-time limit exceeded"
    elif code:
        category = "OTHER_REPLAY_FAILURE"
        detail = "Replay plugin reported " + code
    elif rc in (124, 137, 143, 130):
        category = "INTERRUPTED_OR_TIMEOUT"
        detail = "No replay result; external interruption or timeout"
    elif "no battle result within 600 seconds" in complete:
        category = "EXECUTION_TIMEOUT"
        detail = "No plugin result after wall-clock 600s"
    elif "signal sequence" in complete or "all-off signal service" in complete:
        category = "START_SIGNAL_ERROR"
        detail = "Traffic-light gate failed"
    elif "plugin not ready" in complete:
        category = "GAZEBO_PLUGIN_NOT_READY"
        detail = "Gazebo plugin never reported readiness"
    elif rc != 0:
        category = "EXECUTION_ERROR"
        detail = "Process failed without a confirmed replay result"
    else:
        category = "UNCONFIRMED"
        detail = "No confirmed replay result in available logs"
    # This kinematic simulation only checks explicit oriented rectangles
    # for cars. Gazebo physical wall-contact sensor is not implemented.
    return dict(
        replay_result=code or "NO_RESULT",
        failure_category=category,
        vehicle_pair=pair,
        defense_status=defense_status,
        wall_contact="NOT_MEASURED",
        detail=detail,
        gazebo_log=str(gzlog) if gzlog else "",
    )

def report(summary):
    with summary.open(newline="", encoding="utf-8") as f:
        cases = list(csv.DictReader(f))
    extra = ("replay_result", "failure_category", "vehicle_pair",
             "defense_status", "wall_contact", "detail", "gazebo_log")
    out = summary.parent / "diagnosis.csv"
    rows = []
    for row in cases:
        path = Path(row.get("log_file", ""))
        try:
            rc = int(row.get("exit_code", "0"))
        except ValueError:
            rc = -1
        row.update(classify(path, rc))
        rows.append(row)
    header = list(cases[0].keys()) if cases else []
    header = [key for key in header if key not in extra] + list(extra)
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)
    print(f"DIAGNOSIS: {summary} ({len(rows)} recorded cases)")
    for row in rows:
        print(f"  {row.get('track','?'):12} grid={row.get('grid','?')} "
              f"{row['replay_result']:30} {row['failure_category']:30} "
              f"pair={row['vehicle_pair'] or '-'} "
              f"defense={row['defense_status']}")
    print(f"REPORT: {out}")
    print("WALL: NOT_MEASURED (actual wall contact cannot be inferred from this replay)")
    return rows

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, action="append",
                        help="One or more existing batch summary.csv files")
    parser.add_argument("--all", action="store_true",
                        help="Latest stage1 and defense summaries")
    parser.add_argument("--case", type=Path, help="One case log")
    parser.add_argument("--exit-code", type=int, default=0)
    args = parser.parse_args()
    if args.case:
        result = classify(args.case, args.exit_code)
        for key, value in result.items():
            print(f"{key}={value}")
        return
    targets = args.summary or []
    if args.all or not targets:
        targets += [p for folder in ("battle_stage1", "battle_defense")
                    if (p := newest_summary(folder)) is not None]
    if not targets:
        raise SystemExit("No batch summary.csv found in log/battle_stage1 or log/battle_defense")
    for path in dict.fromkeys(targets):
        report(path)

if __name__ == "__main__":
    main()
