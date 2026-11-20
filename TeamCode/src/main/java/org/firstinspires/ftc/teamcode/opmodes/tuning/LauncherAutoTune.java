package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.launcher.AutoTuneTrial;
import org.firstinspires.ftc.teamcode.subsystems.launcher.AutoTuner;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherMPC;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;
import java.util.Locale;

/**
 * STEP 2 (automatic) of launcher tuning: finds the MPC weights by testing, then saves them.
 * Run Launcher SysId first.
 *
 * Press start and leave it: it repeats a test (steps between LOW and HIGH plus virtual shots,
 * ~15 s each) for every candidate, searching rDu, loadNoise and measNoise (see {@link AutoTuner}).
 * At the end it re-tests the winner against your current values back to back, and only saves if
 * the winner is really better. The old file is kept as launcher.properties.bak.
 *
 * No balls needed: "virtual shots" briefly cut the motor without telling the controller, which
 * looks to it exactly like a ball going through.
 *
 * Before start (init):  dpad up/down = HIGH speed   dpad left/right = trial budget
 * While running:        B = abort (nothing saved)
 * After:                X = undo (restore the previous file)
 */
@TeleOp(name = "Launcher Auto-Tune", group = "tuning")
public class LauncherAutoTune extends LinearOpMode {

    /** Game-like step: LOW = this fraction of HIGH. */
    static final double LOW_FRACTION = 0.85;
    static final int CYCLES = 2;
    static final double SETTLE_SEC = 1.0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        LauncherMPC launcher = new LauncherMPC(this);
        LauncherParams start = launcher.getParams();
        File file = launcher.getParamsFile();
        boolean haveFile = file.exists();

        double vMax = start.kV - start.kS;
        double high = Math.round(0.75 * vMax / 10) * 10.0;
        int budget = 30;
        double shotSec = AutoTuneTrial.defaultShotSec(start);

        // ------------------------------------------------------------------ init: settings
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpadUpWasPressed()) high += 25;
            if (gamepad1.dpadDownWasPressed()) high = Math.max(100, high - 25);
            if (gamepad1.dpadRightWasPressed()) budget = Math.min(80, budget + 2);
            if (gamepad1.dpadLeftWasPressed()) budget = Math.max(6, budget - 2);
            double trialSec = new AutoTuneTrial(LOW_FRACTION * high, high, SETTLE_SEC, CYCLES, shotSec,
                    start.readyTolFrac, start.readyTolMin).durationSec();

            telemetry.addLine("LAUNCHER AUTO-TUNE");
            if (!haveFile) telemetry.addLine("⚠ No params file: run Launcher SysId first (model is defaults).");
            telemetry.addLine("Spins the launcher for several minutes. No balls needed.");
            telemetry.addData("HIGH / LOW", "%.0f / %.0f ticks/s   (dpad ↑↓)", high, LOW_FRACTION * high);
            if (high > 0.85 * vMax) telemetry.addLine("⚠ HIGH is above 85% of top speed: results will be poor.");
            telemetry.addData("Trial budget", "%d   (dpad ←→)", budget);
            telemetry.addData("Takes up to", "%.1f min", (budget + 6) * trialSec / 60);
            telemetry.addData("Starting values", "rDu %s  loadNoise %s  measNoise %s",
                    AutoTuner.fmt(start.rDu), AutoTuner.fmt(start.loadNoise), AutoTuner.fmt(start.measNoise));
            telemetry.addLine("Battery ≥ 12.5 V recommended. Press START.");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            launcher.close();
            return;
        }

        double low = LOW_FRACTION * high;
        launcher.startLogging(6000);
        AutoTuner tuner = new AutoTuner(start, budget);
        String abortReason = null;
        long startMs = System.currentTimeMillis();

        // spin up to LOW and let it settle before the first (warm-up) trial
        launcher.setTargetVelocity(low);
        double spinStart = getRuntime();
        while (opModeIsActive() && getRuntime() - spinStart < 8 && !launcher.isReady()) {
            telemetry.addLine("Spinning up...");
            telemetry.update();
            sleep(20);
        }

        // ------------------------------------------------------------------ search
        LauncherParams cand;
        double lastLogT = -1;
        while (abortReason == null && (cand = tuner.next()) != null) {
            launcher.setParams(cand);
            AutoTuneTrial trial = new AutoTuneTrial(low, high, SETTLE_SEC, CYCLES, shotSec,
                    cand.readyTolFrac, cand.readyTolMin);
            LauncherMPC.State s0 = launcher.getState();
            double t0 = s0 == null ? 0 : s0.time;
            double nextShot = trial.nextShotAt(0);
            double fedUpTo = 0;

            while (true) {
                if (!opModeIsActive()) { abortReason = "stopped"; break; }
                if (gamepad1.bWasPressed()) { abortReason = "aborted (B)"; break; }
                if (launcher.getError() != null) { abortReason = "control thread died: " + launcher.getError(); break; }

                LauncherMPC.State s = launcher.getState();
                double tt = (s == null ? t0 : s.time) - t0;
                launcher.setTargetVelocity(trial.targetAt(tt));
                if (!Double.isNaN(nextShot) && tt >= nextShot) {
                    launcher.virtualShot(trial.shotSec);
                    nextShot = trial.nextShotAt(tt + trial.shotSec + 0.01);
                }
                // score from every control-loop sample: t, target, velocity, estimate, load, power, ...
                for (double[] r : launcher.getLogSince(lastLogT)) {
                    lastLogT = r[0];
                    double rt = r[0] - t0;
                    if (rt >= 0) {
                        trial.add(rt, r[1], r[2], r[5]);
                        fedUpTo = rt;
                    }
                }
                if (trial.isDone(fedUpTo)) break;

                showProgress(tuner, trial, tt, s, startMs);
                sleep(10);
            }
            if (abortReason != null) break;
            tuner.report(trial.score());
        }
        launcher.setTargetVelocity(low);

        // ------------------------------------------------------------------ result
        String saved;
        File backup = new File(file.getPath() + ".bak");
        LauncherParams result = tuner.best();
        if (abortReason != null) {
            launcher.setParams(start);
            saved = "Not saved: " + abortReason + ". Your values are unchanged.";
        } else if (tuner.improved()) {
            launcher.setParams(result);
            try {
                if (file.exists()) copy(file, backup);
                result.save(file, "LauncherAutoTune");
                saved = "SAVED to " + file.getName() + " (old values in " + backup.getName() + ")";
            } catch (Exception e) {
                saved = "SAVE FAILED: " + e.getMessage();
            }
        } else {
            launcher.setParams(start);
            saved = "Your current values were already the best. Nothing changed.";
        }
        launcher.off();

        boolean undone = false;
        while (opModeIsActive()) {
            telemetry.addLine("AUTO-TUNE " + (abortReason != null ? "STOPPED" : "DONE")
                    + String.format(Locale.US, " after %.1f min", (System.currentTimeMillis() - startMs) / 60000.0));
            telemetry.addLine(saved);
            if (abortReason == null) {
                telemetry.addLine(String.format(Locale.US, "score  before %.3f → after %.3f  (%.0f%% better)",
                        tuner.startScore(), tuner.bestScore(),
                        100 * (1 - tuner.bestScore() / tuner.startScore())));
            }
            telemetry.addLine(String.format(Locale.US, "rDu        %s → %s", AutoTuner.fmt(start.rDu), AutoTuner.fmt(result.rDu)));
            telemetry.addLine(String.format(Locale.US, "loadNoise  %s → %s", AutoTuner.fmt(start.loadNoise), AutoTuner.fmt(result.loadNoise)));
            telemetry.addLine(String.format(Locale.US, "measNoise  %s → %s", AutoTuner.fmt(start.measNoise), AutoTuner.fmt(result.measNoise)));
            if (tuner.improved() && abortReason == null && !undone) telemetry.addLine("X = undo (restore old file)");
            if (undone) telemetry.addLine("Undone: old values restored.");
            telemetry.addLine();
            List<String> h = tuner.history;
            for (int i = Math.max(0, h.size() - 8); i < h.size(); i++) telemetry.addLine(h.get(i));
            telemetry.update();

            if (!undone && tuner.improved() && abortReason == null && gamepad1.xWasPressed()) {
                try {
                    if (backup.exists()) {
                        copy(backup, file);
                    } else {
                        start.save(file, "LauncherAutoTune undo");
                    }
                    launcher.setParams(start);
                    undone = true;
                } catch (Exception e) {
                    saved = "UNDO FAILED: " + e.getMessage();
                }
            }
            sleep(20);
        }
        launcher.close();
    }

    /** Plain stream copy: java.nio.file needs Android 8, the Control Hub runs Android 7. */
    private static void copy(File from, File to) throws IOException {
        try (InputStream in = new FileInputStream(from); OutputStream out = new FileOutputStream(to)) {
            byte[] buf = new byte[4096];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        }
    }

    private void showProgress(AutoTuner tuner, AutoTuneTrial trial, double tt, LauncherMPC.State s, long startMs) {
        int done = tuner.history.size();
        int left = tuner.trialsLeftAtMost();
        double elapsedMin = (System.currentTimeMillis() - startMs) / 60000.0;
        telemetry.addLine(String.format(Locale.US, "AUTO-TUNE  trial %d (≤%d left)  pass %d  %.1f min",
                done + 1, left, tuner.pass(), elapsedMin));
        telemetry.addLine("Testing: " + tuner.currentDescription());
        telemetry.addLine(String.format(Locale.US, "%.1f / %.1f s   %s", tt, trial.durationSec(),
                trial.shotActive(tt) ? "VIRTUAL SHOT" : ""));
        if (s != null) {
            telemetry.addLine(String.format(Locale.US, "target %.0f  vel %.0f  power %.2f  batt %.2fV",
                    s.target, s.velocity, s.power, s.battery));
        }
        telemetry.addLine("B = abort (nothing saved)");
        telemetry.addLine();
        List<String> h = tuner.history;
        for (int i = Math.max(0, h.size() - 6); i < h.size(); i++) telemetry.addLine(h.get(i));
        telemetry.update();
    }
}
