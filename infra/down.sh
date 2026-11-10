#!/usr/bin/env bash
# Immediate teardown. Safe to run at any time, including if SkyPilot has lost
# track of the cluster — the gcloud sweep is the backstop.
set -euo pipefail
CLUSTER="${CLUSTER:-operon-rtx}"
sky down "$CLUSTER" --yes 2>/dev/null || echo "(sky had no such cluster)"
echo "=== GCP ground truth after teardown ==="
gcloud compute instances list --project=gen-lang-client-0221638641 2>/dev/null \
  || echo "(no instances / no access)"
