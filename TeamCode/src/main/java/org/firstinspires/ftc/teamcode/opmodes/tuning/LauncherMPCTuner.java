package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherMPC;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;
import org.firstinspires.ftc.teamcode.subsystems.launcher.StepAnalyzer;
import org.firstinspires.ftc.teamcode.subsystems.launcher.TuneScore;

import java.io.File;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * STEP 2 of launcher tuning: live-tune the MPC with numbers, not by ear. Run Launcher SysId first.
 *
 * Every change applies instantly on the control thread. Every step response and every shot dip is
 * measured automatically (rise time, overshoot, settle time, recovery) and the AUTO TEST gives one
 * score to compare settings — the same score the desktop simulator prints (tools/launcher/sim.sh).
 *
 * Gamepad 1
 *   dpad up/down     select parameter          dpad right/left   increase / decrease it
 *   A                step test: toggle LOW ↔ HIGH target
 *   B                spin up to HIGH / off
 *   X                AUTO TEST: off → HIGH → LOW → HIGH → off, then score (feed balls during
 *                    the last HIGH hold to include shot recovery)
 *   Y                SAVE params to the Control Hub file
 *   back / share     write CSV log of the last 60 s (plot it with tools/launcher/plot.html)
 *   left bumper      hold for ×10 bigger adjustments
 *
 * What to do: see subsystems/launcher/README.md ("Tuning recipe").
 */
@TeleOp(name = "Launcher MPC Tuner", group = "tuning")
public class LauncherMPCTuner extends LinearOpMode {

    /** One tunable thing: a LauncherParams field or a tuner-local value. */
    private static final class Item {
        final String name, help;
        final boolean multiplicative;
        final double step, min, max;
        final Field field; // null → tuner-local

        Item(String name, boolean multiplicative, double step, double min, double max, String help) {
            this.name = name;
            this.multiplicative = multiplicative;
            this.step = step;
            this.min = min;
            this.max = max;
            this.help = help;
            Field f;
            try {
                f = LauncherParams.class.getField(name);
            } catch (NoSuchFieldException e) {
                f = null;
            }
            this.field = f;
        }
    }

    private final Item[] items = {
            new Item("targetHigh", false, 25, 0, 6000, "HIGH test target, ticks/s"),
            new Item("targetLow", false, 25, 0, 6000, "LOW test target, ticks/s"),
            new Item("rDu", true, 1.25, 1e-4, 10, "↑ smoother/slower, ↓ snappier/buzzier"),
            new Item("loadNoise", true, 1.25, 0.1, 500, "↑ faster shot recovery & offset fix, ↓ calmer"),
            new Item("measNoise", true, 1.25, 1, 500, "↑ trust encoder less (calmer), ↓ react faster"),
            new Item("shotDetectSigma", false, 0.5, 0, 20, "↓ catches smaller shots, ↑ fewer false alarms, 0 off"),
            new Item("horizonSec", false, 0.05, 0.05, 3, "look-ahead; ~tau is a good start"),
            new Item("qTerminal", false, 1, 0, 200, "↑ less overshoot at end of horizon"),
            new Item("horizonSteps", false, 1, 2, 60, "QP size; more = finer plan, more CPU"),
            new Item("uMin", false, 0.05, -1, 0, "<0 lets it brake when slowing down"),
            new Item("maxDuPerSec", false, 0.5, 0, 50, "power slew limit, 0 = off"),
            new Item("velNoise", true, 1.25, 0.1, 500, "observer: unmodelled speed change"),
            new Item("readyTolFrac", false, 0.005, 0.001, 0.2, "ready band, fraction of target"),
            new Item("readySettleMs", false, 10, 0, 1000, "time in band before ready"),
            new Item("periodMs", false, 1, 2, 50, "control period"),
            new Item("tau", true, 1.03, 0.01, 10, "model (from SysId)"),
            new Item("kV", true, 1.01, 1, 100000, "model (from SysId)"),
            new Item("kS", false, 5, -5000, 5000, "model (from SysId)"),
            new Item("delaySec", false, 0.005, 0, 0.2, "model (from SysId)"),
    };

    private double targetHigh, targetLow;

    private enum Auto { NONE, OFF1, HIGH1, LOW, HIGH2, OFF2 }

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        LauncherMPC launcher = new LauncherMPC(this);
        LauncherParams p = launcher.getParams();
        double vMax = p.kV - p.kS;
        targetHigh = Math.round(0.75 * vMax / 10) * 10.0;
        targetLow = Math.round(0.55 * vMax / 10) * 10.0;
        launcher.startLogging(6000);

        telemetry.addLine("LAUNCHER MPC TUNER — run Launcher SysId first.");
        telemetry.addData("Params", launcher.getParamsFile().exists() ? launcher.getParamsFile().getPath() : "no file yet (defaults)");
        telemetry.addLine("Press START. A = step test, X = auto test, Y = save.");
        telemetry.update();
        waitForStart();

        int sel = 2;
        boolean high = false;
        TuneScore live = new TuneScore(p.readyTolFrac, p.readyTolMin);
        TuneScore auto = null;
        Auto phase = Auto.NONE;
        double phaseStart = 0;
        double lastAutoScore = Double.NaN, lastAutoChatter = Double.NaN;
        List<String> recent = new ArrayList<>();
        double lastLogT = -1;
        String status = "";

        while (opModeIsActive()) {
            double now = getRuntime();
            double mult = gamepad1.left_bumper ? 10 : 1;

            // ---------------- parameter editing
            if (gamepad1.dpadUpWasPressed()) sel = (sel - 1 + items.length) % items.length;
            if (gamepad1.dpadDownWasPressed()) sel = (sel + 1) % items.length;
            int dir = gamepad1.dpadRightWasPressed() ? 1 : gamepad1.dpadLeftWasPressed() ? -1 : 0;
            if (dir != 0) {
                p = launcher.getParams();
                Item it = items[sel];
                double v = get(it, p);
                if (it.multiplicative) {
                    v *= Math.pow(it.step, dir * (mult > 1 ? 4 : 1));
                } else {
                    v += dir * it.step * mult;
                }
                v = Math.max(it.min, Math.min(it.max, v));
                set(it, p, v);
                if (it.field == null) {
                    // retarget live if we are spinning at the value being edited
                    if (launcher.getTarget() > 0 && phase == Auto.NONE) {
                        launcher.setTargetVelocity(high ? targetHigh : targetLow);
                    }
                } else {
                    launcher.setParams(p);
                    if (it.name.equals("tau") || it.name.equals("kV") || it.name.equals("kS")) launcher.resetObserver();
                }
                live = new TuneScore(p.readyTolFrac, p.readyTolMin); // new settings → new stats
                status = "";
            }

            // ---------------- tests
            if (gamepad1.aWasPressed()) {
                phase = Auto.NONE;
                high = !high;
                launcher.setTargetVelocity(high ? targetHigh : targetLow);
            }
            if (gamepad1.bWasPressed()) {
                phase = Auto.NONE;
                if (launcher.getTarget() > 0) {
                    launcher.off();
                } else {
                    high = true;
                    launcher.setTargetVelocity(targetHigh);
                }
            }
            if (gamepad1.xWasPressed()) {
                LauncherParams cur = launcher.getParams();
                auto = new TuneScore(cur.readyTolFrac, cur.readyTolMin);
                phase = Auto.OFF1;
                phaseStart = now;
                launcher.off();
            }
            if (phase != Auto.NONE) {
                double el = now - phaseStart;
                Auto next = phase;
                switch (phase) {
                    case OFF1: // wait until it has coasted down
                        if (el > 1.0 && launcher.getVelocity() < 0.05 * targetHigh) next = Auto.HIGH1;
                        if (el > 8) next = Auto.HIGH1;
                        break;
                    case HIGH1: if (el > 3.0) next = Auto.LOW; break;
                    case LOW: if (el > 2.5) next = Auto.HIGH2; break;
                    case HIGH2: if (el > 4.0) next = Auto.OFF2; break;
                    case OFF2:
                        if (el > 0.3) {
                            lastAutoScore = auto.score();
                            lastAutoChatter = auto.chatter();
                            next = Auto.NONE;
                        }
                        break;
                    default:
                        break;
                }
                if (next != phase) {
                    phase = next;
                    phaseStart = now;
                    switch (phase) {
                        case HIGH1: case HIGH2: launcher.setTargetVelocity(targetHigh); break;
                        case LOW: launcher.setTargetVelocity(targetLow); break;
                        default: launcher.off(); break;
                    }
                }
            }

            // ---------------- save / log
            if (gamepad1.yWasPressed()) {
                try {
                    launcher.saveParams("LauncherMPCTuner");
                    status = "Saved " + launcher.getParamsFile().getName();
                } catch (Exception e) {
                    status = "SAVE FAILED: " + e.getMessage();
                }
            }
            if (gamepad1.backWasPressed()) {
                try {
                    File f = launcher.dumpLog("tune");
                    status = "Wrote " + f.getName();
                } catch (Exception e) {
                    status = "CSV failed: " + e.getMessage();
                }
            }

            // ---------------- metrics from every control-loop sample (not just this slow loop)
            for (double[] r : launcher.getLogSince(lastLogT)) {
                lastLogT = r[0];
                // t, target, velocity, estimate, load, power, battery, loopMs, solveMs
                StepAnalyzer.Result res = live.add(r[0], r[1], r[2], r[5]);
                if (auto != null && phase != Auto.NONE) auto.add(r[0], r[1], r[2], r[5]);
                if (res != null) {
                    recent.add(0, res.summary());
                    while (recent.size() > 4) recent.remove(recent.size() - 1);
                }
            }

            // ---------------- telemetry
            LauncherMPC.State s = launcher.getState();
            p = launcher.getParams();
            if (s != null) {
                telemetry.addLine(String.format("%s  target %.0f  vel %.0f  est %.0f  %s",
                        s.mode, s.target, s.velocity, s.estimate, s.ready ? "READY" : ""));
                telemetry.addLine(String.format("power %.3f  load %.0f  batt %.2fV  loop %.1fms  solve %.2fms/%d it",
                        s.power, s.load, s.battery, s.loopMs, s.solveMs, s.iterations));
                telemetry.addLine(String.format("shots detected %d %s", s.shotCount, s.shotActive ? "← SHOT" : ""));
            }
            if (launcher.getError() != null) telemetry.addLine("CONTROL THREAD DIED: " + launcher.getError());
            if (phase != Auto.NONE) telemetry.addLine("AUTO TEST: " + phase + (phase == Auto.HIGH2 ? "  ← feed balls now" : ""));
            if (!Double.isNaN(lastAutoScore)) {
                telemetry.addLine(String.format("AUTO SCORE %.3f (lower better)  chatter %.4f", lastAutoScore, lastAutoChatter));
            }
            telemetry.addLine(String.format("live chatter %.4f/loop", live.chatter()));
            for (String line : recent) telemetry.addLine(line);
            telemetry.addLine();

            // parameter list: window of 7 around the selection
            int from = Math.max(0, Math.min(sel - 3, items.length - 7));
            for (int i = from; i < Math.min(items.length, from + 7); i++) {
                Item it = items[i];
                telemetry.addLine(String.format("%s %-13s %s", i == sel ? "▶" : "  ", it.name, fmt(get(it, p))));
            }
            telemetry.addLine("   " + items[sel].help);
            if (!status.isEmpty()) telemetry.addLine(status);
            telemetry.update();
            sleep(15);
        }
        launcher.close();
    }

    private double get(Item it, LauncherParams p) {
        if (it.field == null) return it.name.equals("targetHigh") ? targetHigh : targetLow;
        try {
            return it.field.getType() == int.class ? it.field.getInt(p) : it.field.getDouble(p);
        } catch (IllegalAccessException e) {
            return Double.NaN;
        }
    }

    private void set(Item it, LauncherParams p, double v) {
        if (it.field == null) {
            if (it.name.equals("targetHigh")) targetHigh = v;
            else targetLow = v;
            return;
        }
        try {
            if (it.field.getType() == int.class) it.field.setInt(p, (int) Math.round(v));
            else it.field.setDouble(p, v);
        } catch (IllegalAccessException ignored) {
        }
    }

    private static String fmt(double v) {
        double a = Math.abs(v);
        if (a == 0) return "0";
        if (a >= 100) return String.format("%.0f", v);
        if (a >= 1) return String.format("%.3f", v);
        if (a >= 0.01) return String.format("%.4f", v);
        return String.format("%.2e", v);
    }
}
