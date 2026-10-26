#!/usr/bin/env python3
"""Record and restore runtime identity across live/replay container boundaries."""
import argparse
import hashlib
import json
import re
from pathlib import Path


def digest(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("record", "replay"))
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--commit")
    parser.add_argument("--dirty", default="false")
    args = parser.parse_args()
    path = args.run / "runtime_identity.json"
    if args.mode == "record":
        if not args.commit:
            parser.error("record requires --commit")
        data = {"git_commit": args.commit, "git_dirty": args.dirty,
                "image_digest": args.image, "binary_sha256": digest(args.binary)}
        path.write_text(json.dumps(data, indent=2) + "\n")
        return
    if path.exists():
        data = json.loads(path.read_text())
    else:
        # Older recordings predate runtime_identity.json. Their runtime manifest
        # records the environment value actually used by the collaboration log.
        manifests = list(args.run.glob("*/manifest.yaml"))
        if len(manifests) != 1:
            raise SystemExit("replay identity: expected one recorded runtime manifest")
        data = {}
        for line in manifests[0].read_text().splitlines():
            match = re.fullmatch(r'(git_commit|git_dirty|image_digest):\s*(.*)', line)
            if match:
                value = match[2]
                data[match[1]] = json.loads(value) if value.startswith('"') else value
    if data.get("image_digest") != args.image:
        raise SystemExit("replay identity: container image differs from the recorded image")
    if data.get("binary_sha256") and data["binary_sha256"] != digest(args.binary):
        raise SystemExit("replay identity: runtime binary differs from the recorded binary")
    if not data.get("git_commit") or data["git_commit"] == "UNRECORDED":
        raise SystemExit("replay identity: recorded source identity is missing")
    env = {"ARIADNE_GIT_COMMIT": data["git_commit"],
           "ARIADNE_GIT_DIRTY": data.get("git_dirty", "false"),
           "ARIADNE_IMAGE_DIGEST": args.image}
    for value in env.values():
        if "\n" in str(value) or "\r" in str(value):
            raise SystemExit("replay identity: invalid multiline metadata")
    (args.run / "replay_identity.env").write_text(
        "".join(f"{key}={str(value).lower() if isinstance(value, bool) else value}\n"
                for key, value in env.items()))


if __name__ == "__main__":
    main()
