#!/usr/bin/env bash
# Show the cluster AND prove the auto-teardown timer is armed.
set -euo pipefail
CLUSTER="${CLUSTER:-operon-rtx}"
sky status "$CLUSTER" 2>/dev/null || true
echo
echo "=== autostop / teardown state (AUTOSTOP column must NOT be '-') ==="
sky status "$CLUSTER" --refresh 2>/dev/null | sed -n '1,8p'
echo
echo "=== GCP ground truth: any instance actually billing? ==="
gcloud compute instances list \
  --project=gen-lang-client-0221638641 \
  --format="table(name,status,machineType.basename())" 2>/dev/null || true
