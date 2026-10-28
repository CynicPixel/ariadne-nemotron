#!/usr/bin/env python3
"""Index every AWS run: id, instance, cost, outcome, local evidence directory and S3 location.

    python3 infra/scripts/runs_index.py [--build DIR] [--out build/runs/runs.csv]

Sources, merged by run id (later sources fill gaps, never overwrite):
  - evaluation/runs/**/ledger_row.csv        per-run ledger rows committed with the evidence
  - <build>/runs/ledger.csv                  spinup.sh's local ledger
  - <build>/slotq/done/*.log, <build>/runs/*/*.log   spinup.sh launch/teardown lines
Every AWS run id has the form YYYYMMDDTHHMMSSZ-<profile>-<6 hex>; its inputs and outputs live under
s3://<artifact bucket>/runs/<id>/{input,output}/.
"""
import argparse
import csv
import glob
import os
import re

ID = re.compile(r"\b(20\d{6}T\d{6}Z-(?:cpux86|cpuarm|cpu|gpu)-[0-9a-f]{6})\b")
LAUNCH = re.compile(r"run (\S+): (\S+) \((spot|on-demand)\) in (\S+) at \$([0-9.]+)/h")
DONE = re.compile(r"done: exit_code=(\d+), (\d+)s, compute ~\$([0-9.]+)")
COLS = ["run_id", "instance", "market", "az", "start_utc", "seconds", "price_per_hour", "compute_cost_usd", "exit_code",
        "job", "evidence", "s3"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build")
    ap.add_argument("--bucket", default=os.environ.get("ARIADNE_ARTIFACT_BUCKET", "ariadne-836505524290-eun1"))
    ap.add_argument("--out", default="build/runs/runs.csv")
    a = ap.parse_args()
    rows = {}

    def put(rid, **kv):
        r = rows.setdefault(rid, {c: "" for c in COLS} | {"run_id": rid})
        for k, v in kv.items():
            if v not in ("", None) and not r.get(k):
                r[k] = str(v)

    def ledger(path):
        for r in csv.DictReader(open(path)):
            put(r["run_id"], instance=r.get("type"), market=r.get("market"), az=r.get("az"), start_utc=r.get("start_utc"),
                seconds=r.get("seconds"), price_per_hour=r.get("price_per_hour"),
                compute_cost_usd=r.get("compute_cost_usd"), exit_code=r.get("exit_code"))

    for p in sorted(glob.glob("evaluation/runs/**/ledger_row.csv", recursive=True)):
        try:
            rid = open(p).read().split(",")[0]
            ledger_hdr = "run_id,type,market,az,start_utc,seconds,price_per_hour,compute_cost_usd,exit_code\n"
            r = next(csv.DictReader([ledger_hdr, open(p).read().strip() + "\n"]))
            put(r["run_id"], instance=r["type"], market=r["market"], az=r["az"], start_utc=r["start_utc"], seconds=r["seconds"],
                price_per_hour=r["price_per_hour"], compute_cost_usd=r["compute_cost_usd"], exit_code=r["exit_code"])
        except (StopIteration, KeyError, IndexError):
            pass
    if os.path.exists(f"{a.build}/runs/ledger.csv"):
        ledger(f"{a.build}/runs/ledger.csv")
    for log in sorted(glob.glob(f"{a.build}/slotq/done/*.log") + glob.glob(f"{a.build}/runs/*/*.log")):
        job = re.sub(r"^\d+x?-[A-Za-z0-9]+-", "", os.path.basename(log)[:-4]) if "/slotq/" in log else ""
        cur = None
        for line in open(log, errors="ignore"):
            m = LAUNCH.search(line)
            if m and ID.fullmatch(m.group(1)):
                cur = m.group(1)
                put(cur, instance=m.group(2), market=m.group(3), az=m.group(4), price_per_hour=m.group(5), job=job,
                    start_utc=f"{cur[:4]}-{cur[4:6]}-{cur[6:8]}T{cur[9:11]}:{cur[11:13]}:{cur[13:15]}Z")
            m = DONE.search(line)
            if m and cur:
                put(cur, exit_code=m.group(1), seconds=m.group(2), compute_cost_usd=m.group(3))
    # Local evidence: the committed evaluation/runs directory whose path names the run (full id or its 6-hex suffix).
    dirs = [d for d in glob.glob("evaluation/runs/**/", recursive=True)]
    for rid, r in rows.items():
        short = rid.rsplit("-", 1)[1]
        hit = [d for d in dirs if rid in d] or [d for d in dirs if re.search(rf"(^|[_/-]){short}(/|$)", d)]
        if not hit:  # a README that cites the id
            for p in glob.glob("evaluation/runs/**/README.md", recursive=True):
                if rid in open(p, errors="ignore").read():
                    hit = [os.path.dirname(p) + "/"]
                    break
        r["evidence"] = min(hit, key=len).rstrip("/") if hit else ""
        r["s3"] = f"s3://{a.bucket}/runs/{rid}/"
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, COLS)
        w.writeheader()
        for rid in sorted(rows):
            w.writerow(rows[rid])
    cost = sum(float(r["compute_cost_usd"] or 0) for r in rows.values())
    print(f"{a.out}: {len(rows)} runs, {sum(1 for r in rows.values() if r['evidence'])} with committed evidence, "
          f"compute ${cost:.2f} (spot, excluding EBS/IPv4/S3)")


if __name__ == "__main__":
    main()
