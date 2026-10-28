# Narration on a Kaggle T4 (fork F6; AWS GPU spot had no capacity, 2026-10-29): the same voice, model and checks as
# infra/eval/narration_aws.sh. infra/scripts/kaggle.sh narration prepends NARRATION_MING_B64 (narration_ming.py) and
# attaches the cues as a private dataset. Writes remote.json + narration.tar.gz to /kaggle/working.
import base64
import glob
import json
import os
import subprocess
import tarfile

W = "/kaggle/working"


def sh(cmd: str) -> None:
    print("$", cmd, flush=True)
    subprocess.run(cmd, shell=True, check=True)


def out(cmd: str) -> str:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


cues = sorted(glob.glob("/kaggle/input/**/cues.jsonl", recursive=True))
assert cues, "no cues.jsonl in the attached dataset"
remote = {"host": "kaggle", "gpu": out("nvidia-smi --query-gpu=name --format=csv,noheader | head -1"),
          "nproc": out("nproc"), "mem": out("free -g | sed -n 2p"), "cues": cues[0]}
open("/tmp/narration_ming.py", "wb").write(base64.b64decode(NARRATION_MING_B64))  # noqa: F821 (prepended)
sh("git clone -q --depth 1 https://github.com/inclusionAI/Ming-3.0-tts.git /tmp/ming")
remote["ming_commit"] = out("git -C /tmp/ming rev-parse HEAD")
# Kaggle's Python is 3.13 without ensurepip; the AWS run used pytorch/pytorch:2.6.0-cuda12.4 (Python 3.11), so build that.
UV = "uv pip install -q -p /tmp/v/bin/python"
sh("pip install -q uv && uv venv -q -p 3.11 /tmp/v")
sh(f"{UV} torch==2.6.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu124")
sh(f"{UV} soundfile -r /tmp/ming/requirements.txt huggingface_hub")
sh(f"{UV} pynini==2.1.6 || echo 'pynini unavailable: text normalisation off'")
sh("/tmp/v/bin/hf download inclusionAI/Ming-3.0-tts-0.5b --local-dir /tmp/ming-0.5b > /dev/null")
remote["versions"] = out("uv pip freeze -p /tmp/v/bin/python | grep -iE '^(torch|torchaudio|transformers|pynini)=='")
try:
    sh(f"/tmp/v/bin/python /tmp/narration_ming.py --cues {cues[0]} --repo /tmp/ming --model /tmp/ming-0.5b "
       f"--out {W}/narration --extra '{json.dumps({'instance': 'kaggle', 'gpu': remote['gpu']})}'")
    remote["rc"] = 0
except subprocess.CalledProcessError as e:
    remote["rc"] = e.returncode
json.dump(remote, open(f"{W}/remote.json", "w"), indent=1)
if os.path.isdir(f"{W}/narration"):
    with tarfile.open(f"{W}/narration.tar.gz", "w:gz") as t:
        t.add(f"{W}/narration", arcname="narration")
assert remote["rc"] == 0, f"narration_ming.py rc={remote['rc']}"
