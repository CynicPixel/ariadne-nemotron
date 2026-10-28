#!/usr/bin/env bash
# Wait for the single g4dn.xlarge Spot quota to be released by the final-stack run,
# then start the isolated direct-demo profile with its already staged RTF-0 payload.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
INSTANCE=i-09492834f6cf1b8a7
while :; do
  state="$(infra/scripts/aws.sh aws ec2 describe-instances --instance-ids "$INSTANCE" \
    --region eu-north-1 --query 'Reservations[0].Instances[0].State.Name' --output text)"
  printf '%s g4dn %s\n' "$(date -u +%FT%TZ)" "$state" | tee -a evaluation/runs/direct_demo_profile_1500/waiter.log
  [[ "$state" == terminated ]] && break
  sleep 60
done
export ARIADNE_LIVE_PAYLOAD_URI=s3://ariadne-836505524290-eun1/runs/live-payload-20261026T004920Z/payload.tar.gz
exec infra/eval/run_direct_demo_profile_1500.sh
