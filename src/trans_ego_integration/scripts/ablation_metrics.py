#!/usr/bin/env python3
import csv
import argparse
from statistics import mean


def read_rows(path):
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def to_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def main():
    parser = argparse.ArgumentParser(description="Compute ablation metrics for step9.")
    parser.add_argument("--input", required=True, help="CSV with columns: trial,method,success,flight_time,replan_count,jerk_integral")
    parser.add_argument("--output", required=True, help="Output markdown file")
    args = parser.parse_args()

    rows = read_rows(args.input)
    if not rows:
        raise RuntimeError("empty input")

    methods = sorted(set(r["method"] for r in rows))
    table = []
    for m in methods:
        sub = [r for r in rows if r["method"] == m]
        success_rate = mean([to_float(r.get("success", 0.0)) for r in sub])
        flight_time = mean([to_float(r.get("flight_time", 0.0)) for r in sub])
        replan = mean([to_float(r.get("replan_count", 0.0)) for r in sub])
        jerk = mean([to_float(r.get("jerk_integral", 0.0)) for r in sub])
        table.append((m, success_rate, flight_time, replan, jerk))

    with open(args.output, "w", encoding="utf-8") as f:
        f.write("# Step9 消融实验统计\n\n")
        f.write("| Method | Success Rate | Avg Flight Time | Avg Replan Count | Avg Jerk Integral |\n")
        f.write("| --- | ---: | ---: | ---: | ---: |\n")
        for m, s, t, r, j in table:
            f.write(f"| {m} | {s:.3f} | {t:.3f} | {r:.3f} | {j:.3f} |\\n")


if __name__ == "__main__":
    main()
