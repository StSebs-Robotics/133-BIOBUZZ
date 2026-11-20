#!/usr/bin/env bash
# Desktop simulator for the launcher MPC. Compiles the pure-Java controller classes straight out
# of TeamCode (no Android/FTC needed) and runs them against a simulated flywheel.
#
#   ./tools/launcher/sim.sh run   [launcher.properties] [--csv out.csv] [--mismatch 0.15]
#   ./tools/launcher/sim.sh sweep [launcher.properties]
#   ./tools/launcher/sim.sh sysid [launcher.properties]
#
# Get the robot's current params with:  adb pull /sdcard/FIRST/launcher/launcher.properties
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
src="$root/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher"
out="$root/build/launcher-sim"
mkdir -p "$out"
javac -d "$out" \
  "$src/LauncherParams.java" "$src/MpcCore.java" "$src/SysIdFit.java" "$src/StepAnalyzer.java" "$src/TuneScore.java" "$src/AutoTuner.java" "$src/AutoTuneTrial.java" \
  "$here/LauncherSim.java"
exec java -cp "$out" LauncherSim "$@"
