#!/usr/bin/env bash
# Launch the Operon RTX demo cluster with MANDATORY idle auto-teardown.
#
# Use this, never a bare `sky launch`. The -i/--down pair is what guarantees
# the GPU cannot be left running by accident, and it is enforced here rather
# than left to whoever types the command.
#
#   IDLE_MIN=15 infra/launch.sh
set -euo pipefail
cd "$(dirname "$0")/.."

IDLE_MIN="${IDLE_MIN:-15}"
CLUSTER="${CLUSTER:-operon-rtx}"

if [ -z "${NGC_API_KEY:-}" ]; then
  echo "NGC_API_KEY is not set. The Isaac Sim image lives on nvcr.io and" >&2
  echo "cannot be pulled without it. Free key: https://ngc.nvidia.com" >&2
  echo "  export NGC_API_KEY=..." >&2
  exit 1
fi

cat <<BANNER
Launching $CLUSTER
  GPU            : 1x NVIDIA L4 (spot)  ~\$0.28-0.33/hr
  Region         : us-central1
  Idle teardown  : ${IDLE_MIN} min  -> cluster is DOWNed, not just stopped (\$0)
  Session cap    : 4 h hard timeout inside the job
BANNER

# -i N  : autostop after N idle minutes
# --down: TEAR DOWN rather than stop, so no disk is billed either
sky launch -c "$CLUSTER" infra/operon-rtx.sky.yaml \
  -i "$IDLE_MIN" --down --yes \
  --secret NGC_API_KEY

echo
echo "Cluster up. Verify teardown is actually armed:"
echo "  infra/status.sh"
echo "Kill it right now:"
echo "  infra/down.sh"
