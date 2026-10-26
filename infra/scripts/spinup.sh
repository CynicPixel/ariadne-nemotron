#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Ephemeral AWS test runs: launch -> run ONE command -> upload results ->
# terminate. Nothing stays up between tests (DECISIONS D27).
#
#   infra/scripts/spinup.sh run --profile cpu -- 'uname -m; lscpu'
#   infra/scripts/spinup.sh run --profile gpu --max-minutes 45 -- 'nvidia-smi'
#   infra/scripts/spinup.sh run --profile gpu --az eu-north-1c -- '...'   # pin the AZ (spot capacity ran out in 1a)
#   infra/scripts/spinup.sh sweep            # terminate strays, list orphan volumes
#
# PROFILES (cheapest config that runs each workload; Stockholm prices in D28):
#   cpu  c8g.large   Graviton4, 2 vCPU/4 GiB. Smallest size COOL is priced for.
#   cpux86 c7i.4xlarge Ubuntu 24.04 amd64, 16 vCPU/32 GiB: x86 CPU runs in the amd64 images.
#   cpuarm c8g.4xlarge Ubuntu 24.04 arm64, 16 vCPU/32 GiB: native arm64 image builds (plan 0.2, infra/eval/arm64_aws.sh).
#   cool c8g.4xlarge the COOL Marketplace AMI (Ubuntu 24.04); needs the subscription (infra/cool/day1.sh).
#   gpu  g4dn.xlarge NVIDIA T4 16 GB, 4 vCPU. Smallest full-GPU NVIDIA type;
#                    needs the G/VT quota raised from 0.
# Spot is the default market. On-demand is never a silent fallback: it costs
# ~2.6x on g4dn, so it has to be asked for with --market on-demand.
#
# THREE INDEPENDENT BRAKES, because a forgotten instance is the only way this
# project can burn its credits:
#   1. On the instance: `shutdown -h +MAX` is the first line of user-data, and
#      shutdown means TERMINATE (instance-initiated-shutdown-behavior).
#   2. On this host: a trap terminates the instance on exit, error or Ctrl-C.
#   3. `sweep`: terminates any Project=ariadne instance past its own deadline,
#      for when this host crashed and brake 2 never ran.
#
# The test command runs as root on the instance, from the repo root of a copy
# of this working tree (tracked + untracked-but-not-ignored files). Anything it
# writes to $ARIADNE_RESULTS comes back to build/runs/<run-id>/output/, along
# with the full log and the exit code -- uploaded on failure too.
# ---------------------------------------------------------------------------
set -Eeuo pipefail
# No silent deaths: `set -e` exits without a word, which hid two bugs during
# bring-up. -E makes functions inherit this trap.
trap 'log "FAILED (exit $?) at line $LINENO: $BASH_COMMAND"' ERR

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AWS="$REPO_ROOT/infra/scripts/aws.sh"
export AWS_REGION="${AWS_REGION:-eu-north-1}"
LEDGER="$REPO_ROOT/build/runs/ledger.csv"

log() { printf '[spinup %s] %s\n' "$(date -u +%H:%M:%S)" "$*" >&2; }
die() { log "ERROR: $*"; exit 1; }

# Run the AWS CLI and return its stdout without Windows line endings. Captured
# into a variable first so the CLI's exit status is the one that counts: a
# `cmd | tr` pipeline would report tr's status and hide the failure.
awsq() {
  local out
  out="$("$AWS" aws "$@")" || return $?
  printf '%s' "${out//$'\r'/}"
}

profile_defaults() {
  case "$1" in
    cpu)
      TYPE="${TYPE:-c8g.large}"
      AMI_PARAM="/aws/service/ami-amazon-linux-latest/al2023-ami-kernel-default-arm64"
      ROOT_GB=16 ;;
    gpu)
      TYPE="${TYPE:-g4dn.xlarge}"
      AMI_PARAM="/aws/service/deeplearning/ami/x86_64/base-oss-nvidia-driver-gpu-ubuntu-24.04/latest/ami-id"
      ROOT_GB=0 ;;   # 0 = the AMI's own snapshot size (75 GB for this DLAMI)
    cpux86)
      # x86 CPU work that needs the amd64 ariadne images (no arm64 image until plan 0.2): Ubuntu 24.04 amd64.
      # c7i.4xlarge = 16 vCPU (8 cores x 2), 32 GiB.
      TYPE="${TYPE:-c7i.4xlarge}"
      AMI_PARAM="/aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id"
      ROOT_GB=80 ;;
    cpuarm)
      # Native arm64 Docker builds (plan 0.2: OpenCV 5 + ariadne-core for linux/arm64): Ubuntu 24.04 arm64, like
      # cpux86 but Graviton4. c8g.4xlarge = 16 vCPU (16 cores, no SMT), 32 GiB. Root sized for the OpenCV build.
      TYPE="${TYPE:-c8g.4xlarge}"
      AMI_PARAM="/aws/service/canonical/ubuntu/server/24.04/stable/current/arm64/hvm/ebs-gp3/ami-id"
      ROOT_GB=60 ;;
    cool)
      # The COOL Marketplace AMI (Ubuntu 24.04, Graviton4; D28: product aajkmdd4qo3r7yhqg61a7aah9). Needs an active
      # Marketplace subscription, else launch fails with an opt-in error. Root sized for a native OpenCV build.
      TYPE="${TYPE:-c8g.4xlarge}"
      AMI_ID="ami-0a302ff85af8d8764"
      ROOT_GB=40 ;;
    *) die "unknown profile '$1' (cpu | cpux86 | cpuarm | gpu | cool)" ;;
  esac
}

runner_outputs() {
  local json
  # ARIADNE_RUNNER_JSON: the `tf main output -json runner` of the checkout that holds the Terraform state, saved to a
  # file, for a git worktree that has none (the state is never copied).
  if [ -n "${ARIADNE_RUNNER_JSON:-}" ]; then
    json="$(cat "$ARIADNE_RUNNER_JSON")" || die "cannot read $ARIADNE_RUNNER_JSON"
  else
    json="$("$AWS" tf main output -json runner 2>/dev/null)" \
      || die "cannot read Terraform outputs; is the main stack applied?"
  fi
  eval "$(printf '%s' "$json" | python3 -c '
import json, sys, shlex
d = json.load(sys.stdin)
for k, v in (("PROFILE_NAME","instance_profile"),("SG_ID","security_group_id"),
             ("BUCKET","artifact_bucket")):
    print(f"{k}={shlex.quote(d[v])}")
print("SUBNETS_BY_AZ=" + shlex.quote(" ".join(f"{az}={sn}" for az, sn in sorted(d["subnets_by_az"].items()))))')"
}

# Pick the subnet to launch into. Spot: the AZ with the lowest current spot
# price for this type (prices differ per AZ, sometimes by 2x). On-demand: the
# first AZ, since on-demand prices are uniform across a region. Sets SUBNET_ID,
# AZ and PRICE.
choose_placement() {
  local market="$1" want_az="${2:-}"
  if [ "$market" = spot ] && [ -n "$want_az" ]; then
    # --az: the cheapest AZ can be out of capacity (InsufficientInstanceCapacity, 2026-10-22), so the caller pins
    # one. Its current spot price is still recorded for the ledger.
    case " $SUBNETS_BY_AZ " in *" $want_az="*) ;; *) die "--az $want_az has no default subnet ($SUBNETS_BY_AZ)" ;; esac
    AZ="$want_az"
    PRICE="$(awsq ec2 describe-spot-price-history --instance-types "$TYPE" --availability-zone "$AZ" \
      --product-descriptions 'Linux/UNIX' --start-time "$(date -u +%FT%TZ)" \
      --query 'SpotPriceHistory[0].SpotPrice' --output text)"
  elif [ "$market" = spot ]; then
    # Captured first, then scanned: an early `break` inside `sort | while`
    # would SIGPIPE the sort, and under pipefail that kills the script.
    local prices best="" az p
    prices="$(awsq ec2 describe-spot-price-history --instance-types "$TYPE" \
      --product-descriptions 'Linux/UNIX' --start-time "$(date -u +%FT%TZ)" \
      --query 'SpotPriceHistory[].[AvailabilityZone,SpotPrice]' --output text)"
    while read -r az p; do
      # SPINUP_AVOID_AZ: space-separated AZs to skip, e.g. one that just answered InsufficientInstanceCapacity
      case " ${SPINUP_AVOID_AZ:-} " in *" $az "*) continue ;; esac
      case " $SUBNETS_BY_AZ " in *" $az="*) best="$az $p"; break ;; esac
    done < <(sort -k2 -g <<< "$prices")
    [ -n "$best" ] || die "no current spot price for $TYPE in any default-subnet AZ"
    read -r AZ PRICE <<< "$best"
  else
    AZ="${SUBNETS_BY_AZ%%=*}"
    PRICE="${ONDEMAND_PRICE[$TYPE]:-unknown}"
  fi
  local pair
  for pair in $SUBNETS_BY_AZ; do
    [ "${pair%%=*}" = "$AZ" ] && SUBNET_ID="${pair#*=}"
  done
  [ -n "${SUBNET_ID:-}" ] || die "no default subnet in $AZ"
}

cmd_run() {
  local profile="" market="spot" max_minutes=30 az=""
  TYPE=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --profile)     profile="$2"; shift 2 ;;
      --type)        TYPE="$2"; shift 2 ;;
      --market)      market="$2"; shift 2 ;;
      --max-minutes) max_minutes="$2"; shift 2 ;;
      --az)          az="$2"; shift 2 ;;
      --) shift; break ;;
      *) die "unknown option $1" ;;
    esac
  done
  [ -n "$profile" ] || die "--profile is required"
  [ $# -gt 0 ] || die "no test command given (after --)"
  case "$market" in spot|on-demand) ;; *) die "--market must be spot or on-demand" ;; esac
  [ "$max_minutes" -ge 5 ] && [ "$max_minutes" -le 240 ] || die "--max-minutes must be 5..240"
  local test_cmd="$*"
  profile_defaults "$profile"
  if [ -n "${ARIADNE_ROOT_GB:-}" ]; then
    [[ "$ARIADNE_ROOT_GB" =~ ^[0-9]+$ ]] && [ "$ARIADNE_ROOT_GB" -gt 0 ] || die 'ARIADNE_ROOT_GB must be positive GiB'
    ROOT_GB="$ARIADNE_ROOT_GB"
  fi
  runner_outputs
  choose_placement "$market" "$az"

  local run_id; run_id="$(date -u +%Y%m%dT%H%M%SZ)-$profile-$(head -c3 /dev/urandom | od -An -tx1 | tr -d ' \n')"
  local run_dir="$REPO_ROOT/build/runs/$run_id" s3_run="s3://$BUCKET/runs/$run_id"
  mkdir -p "$run_dir"
  log "run $run_id: $TYPE ($market) in $AZ at \$$PRICE/h, max ${max_minutes} min"

  # 1. Ship the working tree. Tracked + untracked-but-not-ignored, so work that
  #    is not committed yet is tested too; build/ and secrets stay out via
  #    .gitignore.
  (cd "$REPO_ROOT" && git ls-files -z --cached --others --exclude-standard \
     | tar --exclude="${ARIADNE_SNAPSHOT_EXCLUDE:-__no_snapshot_exclusions__}" --null -T - -czf "$run_dir/repo.tar.gz")
  awsq s3 cp "build/runs/$run_id/repo.tar.gz" "$s3_run/input/repo.tar.gz" --only-show-errors >/dev/null
  log "uploaded repo snapshot ($(du -h "$run_dir/repo.tar.gz" | cut -f1))"

  # 2. User data. Brake 1 comes first, before anything that could hang.
  local cmd_b64; cmd_b64="$(printf '%s' "$test_cmd" | base64 -w0)"
  cat > "$run_dir/user-data.sh" <<EOF
#!/bin/bash
shutdown -h +$max_minutes
exec > /var/log/ariadne-run.log 2>&1
export AWS_DEFAULT_REGION=$AWS_REGION AWS_REGION=$AWS_REGION
RUN=$s3_run
finish() {
  rc=\$?
  echo "exit_code=\$rc" > /tmp/exit_code
  aws s3 cp /tmp/exit_code \$RUN/output/exit_code --only-show-errors
  [ -d /opt/ariadne/results ] && aws s3 cp --recursive /opt/ariadne/results \$RUN/output/results/ --only-show-errors
  aws s3 cp /var/log/ariadne-run.log \$RUN/output/run.log --only-show-errors
  shutdown -h now
}
trap finish EXIT
set -x
if ! command -v aws >/dev/null 2>&1; then
  # Results can only leave through the CLI, so install it if the AMI lacks it.
  curl -fsSL "https://awscli.amazonaws.com/awscli-exe-linux-\$(uname -m).zip" -o /tmp/awscli.zip
  python3 -m zipfile -e /tmp/awscli.zip /tmp && chmod -R +x /tmp/aws && /tmp/aws/install
fi
mkdir -p /opt/ariadne/src /opt/ariadne/results
cd /opt/ariadne/src
aws s3 cp \$RUN/input/repo.tar.gz - | tar xz
export ARIADNE_RESULTS=/opt/ariadne/results ARIADNE_RESULTS_URI=\$RUN/output/results   # jobs may upload as they go
echo '$cmd_b64' | base64 -d > /opt/ariadne/test.sh
bash /opt/ariadne/test.sh
EOF

  # 3. Launch. Root volume is forced to DeleteOnTermination: a stray 75 GB
  #    gp3 volume from the GPU AMI would bill silently for months.
  local ami root_dev snap_gb
  if [ -n "${AMI_ID:-}" ]; then ami="$AMI_ID"; else
    ami="$(awsq ssm get-parameter --name "$AMI_PARAM" --query Parameter.Value --output text)"; fi
  # Here-string, not `< <(awsq ...)`: awsq prints no trailing newline, and
  # `read` returns 1 at EOF-without-newline EVEN THOUGH it assigned the
  # variables. Under `set -e` that killed the script silently, mid-launch.
  read -r root_dev snap_gb <<< "$(awsq ec2 describe-images --image-ids "$ami" \
    --query 'Images[0].[RootDeviceName,BlockDeviceMappings[0].Ebs.VolumeSize]' --output text)"
  [ -n "${root_dev:-}" ] && [ -n "${snap_gb:-}" ] || die "could not read root device of $ami"
  local root_gb=$ROOT_GB; [ "$root_gb" -eq 0 ] && root_gb="$snap_gb"
  local market_args=()
  [ "$market" = spot ] && market_args=(--instance-market-options
    'MarketType=spot,SpotOptions={SpotInstanceType=one-time,InstanceInterruptionBehavior=terminate}')

  local iid
  iid="$(awsq ec2 run-instances --image-id "$ami" --instance-type "$TYPE" --count 1 \
    "${market_args[@]}" \
    --iam-instance-profile "Name=$PROFILE_NAME" --subnet-id "$SUBNET_ID" --security-group-ids "$SG_ID" \
    --instance-initiated-shutdown-behavior terminate \
    --metadata-options 'HttpTokens=required,HttpPutResponseHopLimit=2,HttpEndpoint=enabled' \
    --block-device-mappings "DeviceName=$root_dev,Ebs={VolumeSize=$root_gb,VolumeType=gp3,DeleteOnTermination=true}" \
    --user-data "file://build/runs/$run_id/user-data.sh" \
    --tag-specifications "ResourceType=instance,Tags=[{Key=Name,Value=ariadne-$run_id},{Key=Project,Value=ariadne},{Key=ariadne:run-id,Value=$run_id},{Key=ariadne:max-minutes,Value=$max_minutes}]" \
    --query 'Instances[0].InstanceId' --output text)" || die "launch failed (see above)"
  log "launched $iid"

  # Brake 2. RUN_IID / RUN_FINISHED are GLOBALS on purpose: the EXIT trap fires
  # after cmd_run has returned, when its locals no longer exist, so a trap
  # reading a local would terminate an empty instance id and silently do nothing.
  RUN_IID="$iid"; RUN_FINISHED=0
  trap terminate_on_exit EXIT INT TERM

  local start_epoch; start_epoch="$(date +%s)"
  local az="$AZ" price="$PRICE"

  # 4. Wait for the instance to terminate itself. Deadline: its own brake plus
  #    a margin, so this loop cannot outlive brake 1 by much.
  #    A failed poll is NOT a reason to kill a healthy run (a transient AuthFailure terminated
  #    20261023T234259Z-cpux86-4fc41e 48 min in): retry; give up only after ~10 min of consecutive failures.
  #    The instance's own brake 1 still bounds it.
  local deadline=$(( start_epoch + (max_minutes + 8) * 60 )) state="" poll_fail=0
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if state="$(awsq ec2 describe-instances --instance-ids "$iid" --query 'Reservations[0].Instances[0].State.Name' --output text)"; then
      poll_fail=0
    else
      poll_fail=$((poll_fail + 1))
      log "state poll failed ($poll_fail consecutive); retrying"
      if [ "$poll_fail" -ge 30 ]; then
        # The host cannot see AWS (expired credentials, 2026-10-24 09:22 IST): it must NOT kill a run it cannot see.
        # The instance's own brake still ends it and its exit trap uploads the results under its instance role.
        RUN_FINISHED=1
        log "state poll failing for ~10 min: leaving $iid to its own brake (max ${max_minutes} min);" \
            "fetch later: aws s3 cp --recursive $s3_run/output build/runs/$run_id/output"
        return 75
      fi
    fi
    [ "$state" = terminated ] && break
    sleep 20
  done
  [ "$state" = terminated ] || { log "deadline passed in state '$state'"; terminate_on_exit; }
  RUN_FINISHED=1
  local end_epoch; end_epoch="$(date +%s)"

  # 5. Results. SPINUP_FETCH_EXCLUDE (space-separated globs, e.g. '*.tar.gz *.mcap') leaves bulky outputs in S3: the
  #    download runs over the host's link while the slot queue still counts the job as running (2026-10-24: two
  #    live runs held their slots ~70 min after their instances ended, fetching multi-GB run tarballs). Fetch those
  #    later, or analyse them in-region.
  local excl=() globs=()
  read -r -a globs <<< "${SPINUP_FETCH_EXCLUDE:-}"   # read -a: the globs are never expanded against the cwd
  for g in "${globs[@]}"; do excl+=(--exclude "$g"); done
  [ ${#excl[@]} -gt 0 ] && log "fetch skips: ${SPINUP_FETCH_EXCLUDE} (left in $s3_run/output)"
  "$AWS" aws s3 cp --recursive "$s3_run/output" "build/runs/$run_id/output" "${excl[@]}" --only-show-errors >/dev/null 2>&1 || true
  local exit_code="missing"
  [ -f "$run_dir/output/exit_code" ] && exit_code="$(cut -d= -f2 < "$run_dir/output/exit_code")"
  local secs=$(( end_epoch - start_epoch ))
  local cost; cost="$(python3 -c "p='$price'; print(f'{float(p)*max($secs,60)/3600:.4f}' if p.replace('.','',1).isdigit() else 'unknown')")"
  [ -f "$LEDGER" ] || echo "run_id,type,market,az,start_utc,seconds,price_per_hour,compute_cost_usd,exit_code" > "$LEDGER"
  echo "$run_id,$TYPE,$market,$az,$(date -u -d "@$start_epoch" +%FT%TZ),$secs,$price,$cost,$exit_code" >> "$LEDGER"

  log "done: exit_code=$exit_code, ${secs}s, compute ~\$$cost (EBS/IPv4 extra, cents)"
  log "results: build/runs/$run_id/output/"
  [ -f "$run_dir/output/run.log" ] && tail -n 25 "$run_dir/output/run.log" >&2
  [ "$exit_code" = 0 ]
}

RUN_IID=""; RUN_FINISHED=1
terminate_on_exit() {
  [ "$RUN_FINISHED" = 1 ] && return 0
  [ -n "$RUN_IID" ] || return 0
  log "host exiting early: terminating $RUN_IID"
  "$AWS" aws ec2 terminate-instances --instance-ids "$RUN_IID" >/dev/null 2>&1 || true
  RUN_FINISHED=1
}

cmd_sweep() {
  local now; now="$(date +%s)"
  local rows
  rows="$(awsq ec2 describe-instances \
    --filters Name=tag:Project,Values=ariadne Name=instance-state-name,Values=pending,running,stopping,stopped \
    --query 'Reservations[].Instances[].[InstanceId,LaunchTime,Tags[?Key==`ariadne:max-minutes`]|[0].Value,InstanceType]' \
    --output text)"
  local killed=0
  while read -r iid launched maxm itype; do
    [ -n "${iid:-}" ] || continue
    local age=$(( (now - $(date -d "$launched" +%s)) / 60 ))
    [ "$maxm" = None ] && maxm=30
    if [ "$age" -gt $(( maxm + 5 )) ]; then
      log "STRAY $iid ($itype) age ${age} min > limit ${maxm}: terminating"
      awsq ec2 terminate-instances --instance-ids "$iid" >/dev/null; killed=$((killed + 1))
    else
      log "running $iid ($itype) age ${age} min, within limit ${maxm}"
    fi
  done <<< "$rows"
  local vols
  vols="$(awsq ec2 describe-volumes --filters Name=status,Values=available --query 'Volumes[].[VolumeId,Size,CreateTime]' --output text)"
  if [ -n "$vols" ]; then log "UNATTACHED volumes (billing, review and delete):"; echo "$vols" >&2; fi
  log "sweep done: $killed terminated"
}

# On-demand $/h, eu-north-1 (Stockholm), AWS published price data 2026-09-29 (D28).
# Only used for --market on-demand; spot runs record the live spot price.
declare -A ONDEMAND_PRICE=(
  [c8g.large]=0.0851 [m8g.4xlarge]=0.76304 [g4dn.xlarge]=0.558 [g6f.large]=0.21424
)

case "${1:-}" in
  run)   shift; cmd_run "$@" ;;
  sweep) shift; cmd_sweep "$@" ;;
  *) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
