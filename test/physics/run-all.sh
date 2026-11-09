#!/usr/bin/env bash
# Operon MuJoCo physics regression suite.
#
#   ./test/physics/run-all.sh            # Node suites only (no browser needed)
#   ./test/physics/run-all.sh --browser  # also the headless-Chrome suites
#
# The browser suites need a server. Start one first:
#   PORT=8841 node server.mjs &
# or point them elsewhere with OPERON_URL=http://host:port
#
# Measure on an IDLE machine. A full /tmp or a server that dies mid-loop
# produces failures that look exactly like physics regressions -- that cost
# real debugging time once already.
set -u
cd "$(dirname "$0")/../.."
pass=0; fail=0
run() {
  printf "  %-26s " "$1"
  out="$(node "$2" 2>&1 | tail -1)"
  case "$out" in
    *PASSED*|*"8/8"*|*"reduced on EVERY"*) echo "ok    $out"; pass=$((pass+1));;
    *) echo "FAIL  $out"; fail=$((fail+1));;
  esac
}
echo "Node suites (deterministic):"
run "MJCF vs fk6"        test/physics/validate_mjcf.mjs
run "module API"         test/physics/module_api.mjs
run "grasp, random yaws" test/physics/grasp_random_yaw.mjs
run "dock retention"     test/physics/dock_retention.mjs
run "dock insertion"     test/physics/dock_insertion.mjs
run "valve turn"         test/physics/valve_turn.mjs
run "trajectory smoothness" test/physics/smoothness.mjs

if [ "${1:-}" = "--browser" ]; then
  APP="${OPERON_URL:-http://127.0.0.1:8841}"
  if ! curl -fsS -o /dev/null "$APP/teleop-studio.html" 2>/dev/null; then
    echo "  !! no server at $APP -- start one:  PORT=8841 node server.mjs &"
    exit 1
  fi
  echo "Browser suites (headless Chrome against $APP):"
  run "station 0 grasp cycle"  test/physics/browser_station0.mjs
  run "stations 1+2"           test/physics/browser_stations12.mjs
  run "render fidelity"        test/physics/browser_render_fidelity.mjs
  run "kinematic fallback"     test/physics/browser_fallback.mjs
  rm -rf /tmp/cr-prof-* 2>/dev/null || true
fi
echo
echo "  $pass passed, $fail failed"
[ "$fail" -eq 0 ]
