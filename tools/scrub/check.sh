#!/usr/bin/env bash
# tools/scrub/check.sh: the pre-push gate (CLAUDE.md §2 invariants 2 and 7, D330). Exit 0 = clean, 1 = findings.
#   tools/scrub/check.sh             tracked + untracked (not ignored) files: denylist, deleted directories,
#                                    dangling references to them, secrets
#   tools/scrub/check.sh --release   also fails on TODO(FM) markers in README.md and docs/submission/
#   tools/scrub/check.sh --history   also scans every commit on every ref for secrets (run before the repo goes public)
# Secret findings print file:line only, never the matched text.
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
release=0; history=0
for a in "$@"; do case "$a" in --release) release=1 ;; --history) history=1 ;; *) echo "usage: $0 [--release] [--history]" >&2; exit 2 ;; esac; done
fail=0
section() { printf '%-34s' "$1"; }
report() {  # report <findings>: one finding per line; empty = pass
  if [ -z "$1" ]; then echo "ok"; else echo "FAIL"; sed 's/^/    /' <<<"$1"; fail=1; fi
}
GREP=(git grep --untracked -n -I)

# 1. Old-competition identity (D330). Only the award phrasing is denied: bare "COOL" (the OpenCV-on-Arm benchmark),
#    AWS and Bedrock (honest compute and legacy provider) stay. Case-insensitive, plus "Agentic Vision" as the award
#    name in title case. Applies to the whole tree, run evidence included; only this directory is exempt.
DENY='opencv26|opencv[ _-]ai[ _-]competition|powered by aws|best use of cool|cool award|agentic vision award|judges-live|ariadne-vms1xc|yrbcvqknfka'
section "denylist (text)"
report "$( { "${GREP[@]}" -i -E -e "$DENY" -- . ':!tools/scrub'; "${GREP[@]}" -E -e 'Agentic Vision' -- . ':!tools/scrub'; } | cut -c1-200 | sort -u)"
section "denylist (binary, raw bytes)"
report "$(git grep --untracked -l -a -i -E -e "$DENY" -- . ':!tools/scrub' | while IFS= read -r f; do grep -Iq . "$f" 2>/dev/null || echo "$f"; done)"
if command -v pdftotext >/dev/null; then
  section "denylist (PDF text)"
  report "$(git ls-files -- '*.pdf' | while IFS= read -r f; do pdftotext -q "$f" - 2>/dev/null | grep -q -i -E -e "$DENY" -e 'Agentic Vision' && echo "$f"; done)"
fi

# 2. Directories the fork deleted (F3.2) must stay deleted; apply.sh removes them after a merge.
section "deleted directories absent"
report "$( { git ls-files -- submission docs/report infra/cool python/video/scenes; git ls-files --others --exclude-standard -- submission docs/report infra/cool python/video/scenes; } | head -20)"

# 3. Living files must not point into those directories. Historical records are exempt: run evidence, the decision
#    log and the process docs cite paths as they were when written.
section "references to deleted paths"
report "$("${GREP[@]}" -E -e '(^|[^/A-Za-z0-9_.-])submission/|docs/report/|infra/cool/|video/scenes/' -- . \
  ':!evaluation/runs' ':!docs/decisions.md' ':!docs/process' ':!tools/scrub' | cut -c1-200)"

# 3b. Relative Markdown links (`](path)`, `src="path"`, `href="path"`) must resolve, which also catches `../submission/` and
#     `report/` forms the pattern above cannot see. Same exemptions.
section "relative Markdown links resolve"
report "$( { git ls-files -z -- '*.md' ':!evaluation/runs' ':!docs/decisions.md' ':!docs/process' ':!tools/scrub'
             git ls-files -z --others --exclude-standard -- '*.md' ':!evaluation/runs' ':!docs/process' ':!tools/scrub'; } |
  python3 -c '
import os, re, sys
for f in filter(None, sys.stdin.read().split("\0")):
    for n, line in enumerate(open(f, encoding="utf-8", errors="replace"), 1):
        for a, b in re.findall(r"\]\(([^)\s]+)\)|(?:src|href)=\"([^\"]+)\"", line):
            t = (a or b).split("#")[0]
            if t and not re.match(r"^(https?:|mailto:|data:)", t) and not os.path.exists(os.path.join(os.path.dirname(f), t)):
                print(f"{f}:{n}: {a or b}")')"

# 4. Secrets (invariant 2). Token shapes, then the literal values of the local credential files, if present.
# Token shapes match case-sensitively (a case-folded AKIA... matches base64 in embedded PNGs); key assignments
# (aws_secret_access_key = ..., NEBIUS_API_KEY: ...) match case-insensitively.
TOKENS='\b(AKIA|ASIA)[0-9A-Z]{16}\b|-----BEGIN ([A-Z]+ )?PRIVATE KEY-----|\bgh[pousr]_[A-Za-z0-9]{36}\b|\bgithub_pat_[A-Za-z0-9_]{22,}'
TOKENS+='|\bsk-(proj-)?[A-Za-z0-9_-]{20,}|\bxox[abprs]-[A-Za-z0-9-]{10,}|\bhf_[A-Za-z0-9]{30,}|\bAIza[0-9A-Za-z_-]{35}\b|Bearer [A-Za-z0-9._~+/-]{20,}'
ASSIGN='aws_secret_access_key[[:space:]]*[=:][[:space:]]*[A-Za-z0-9/+]{40}'
ASSIGN+='|(nebius|token_factory|openai|api)_?(api_)?key[[:space:]]*[=:][[:space:]]*["'"'"']?[A-Za-z0-9_.-]{16,}'
EXAMPLES='AKIAIOSFODNN7EXAMPLE|wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY'  # AWS documentation's example pair
section "secrets (token shapes)"
report "$( { "${GREP[@]}" -E -e "$TOKENS" -- . ':!tools/scrub'; "${GREP[@]}" -i -E -e "$ASSIGN" -- . ':!tools/scrub'; } |
  grep -v -E "$EXAMPLES" | cut -d: -f1,2 | sort -u)"

# Literal values: every VALUE of length >= 16 in the local credential files, matched as fixed strings.
lit=$(mktemp); chmod 600 "$lit"; trap 'rm -f "$lit"' EXIT
for cred in "$HOME/.config/ariadne/nebius.env" "$HOME/.aws/credentials"; do
  [ -r "$cred" ] && sed -n -E 's/^[[:space:]]*(export[[:space:]]+)?[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=[[:space:]]*["'"'"']?([^"'"'"'[:space:]]{16,})["'"'"']?[[:space:]]*$/\2/p' "$cred" >> "$lit"
done
if [ -s "$lit" ]; then
  section "secrets (local key values, $(wc -l <"$lit"))"
  report "$(git grep --untracked -n -a -F -f "$lit" -- . | cut -d: -f1,2)"
else
  section "secrets (local key values)"; echo "skipped (no local credential file)"
fi

if [ "$history" = 1 ]; then
  # Every commit on every ref as patches; each added line is prefixed "<commit> <file><TAB>" so plain grep can scan it.
  # A finding prints the commit and file only. Any failure in the pipeline is a FAIL, never a silent pass.
  section "secrets in history (all refs)"
  hist=$(mktemp); chmod 600 "$hist"; trap 'rm -f "$lit" "$hist"' EXIT
  if git log --all -p -U0 --no-color --no-ext-diff --format='@@commit %h' |
     awk '/^@@commit / { c = $2; next } /^\+\+\+ / { f = substr($0, 7); next } /^\+/ { print c " " f "\t" $0 }' > "$hist"
  then
    report "$( { grep -a -E -e "$TOKENS" "$hist"; grep -a -i -E -e "$ASSIGN" "$hist"; [ -s "$lit" ] && grep -a -F -f "$lit" "$hist"; } |
      grep -a -v -E "$EXAMPLES" | cut -f1 | sort -u)"
    echo "    ($(wc -l <"$hist") added lines scanned)"
  else
    report "history export failed"
  fi
fi

if [ "$release" = 1 ]; then
  section "release: no TODO(FM) markers"
  report "$("${GREP[@]}" -F 'TODO(FM)' -- README.md docs/submission 2>/dev/null)"
fi

[ "$fail" = 0 ] && echo "check.sh: clean" || echo "check.sh: FINDINGS (see FAIL above)"
exit "$fail"
