#!/usr/bin/env bash
# tools/scrub/apply.sh: re-applies the scrub to the current tree (D300, D330).
# Run after every upstream merge, then tools/scrub/check.sh. Idempotent: a second run changes nothing.
#   tools/scrub/apply.sh      prints each path it removed or rewrote; stages nothing except the removals
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
RULES=tools/scrub/rules.sed

# 1. Directories the fork deleted (F3.2). A merge from upstream can bring them back; take them out again.
#    The last commit that still has them is 7496988 (the film and its beat sources are recoverable from it).
for d in submission docs/report infra/cool python/video/scenes; do
  if [ -n "$(git ls-files -- "$d")" ]; then git rm -r -q -- "$d"; echo "removed  $d/"; fi
done

# 2. Line rewrites over tracked text files (git grep -I skips binaries). Run evidence under evaluation/runs/ is kept
#    byte-for-byte, and the scrub's own files hold the patterns. A file is written only if sed changed it, so
#    untouched sources keep their mtime (no spurious rebuilds), and it is written through `cat >` to keep its mode.
tmp=$(mktemp); trap 'rm -f "$tmp"' EXIT
git grep -I -l -z -e '' -- . ':!evaluation/runs' ':!tools/scrub' |
while IFS= read -r -d '' f; do
  [ -L "$f" ] && continue
  sed -E -f "$RULES" "$f" > "$tmp"
  if ! cmp -s "$f" "$tmp"; then cat "$tmp" > "$f"; echo "rewrote  $f"; fi
done
