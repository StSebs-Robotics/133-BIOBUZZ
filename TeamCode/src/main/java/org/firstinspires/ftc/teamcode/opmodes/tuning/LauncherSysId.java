package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherMPC;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;
import org.firstinspires.ftc.teamcode.subsystems.launcher.SysIdFit;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * STEP 1 of launcher tuning: measure the flywheel and fit the model (tau, kV, kS, delay).
 *
 * Runs a fixed open-loop power sequence (~30 s, up to FULL power), records every control-thread
 * sample, fits the model and shows the result. Press A to save it into the launcher's params file
 * (MPC weights in the file are kept). Run it again whenever the launcher hardware changes:
 * new wheel, new gearing, new motor, added mass, belt tension...
 *
 * Before you start: no balls in the launcher, robot on a stand or held, fresh-ish battery
 * (12.5 V+). Hood/other mechanisms where they are in a match (they can add drag).
 *
 * Gamepad 1 after the run:  A = save   B = discard   X = write CSV log
 */
@TeleOp(name = "Launcher SysId", group = "tuning")
public class LauncherSysId extends LinearOpMode {

    static final double[] POWERS = SysIdFit.TEST_POWERS;
    static final double MIN_HOLD_S = 1.0, MAX_HOLD_S = 5.0, SETTLE_WINDOW_S = 0.4;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        LauncherMPC launcher = new LauncherMPC(this);
        LauncherParams before = launcher.getParams();

        telemetry.addLine("LAUNCHER SYSID");
        telemetry.addLine("Spins the flywheel up to FULL POWER for ~30 s.");
        telemetry.addLine("Remove balls, keep hands clear, battery ≥ 12.5 V.");
        telemetry.addData("Params file", launcher.getParamsFile());
        telemetry.addData("Direct bulk read", launcher.hasDirectBulkRead() ? "yes" : "NO (fallback)");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            launcher.close();
            return;
        }

        launcher.startLogging(30000);
        List<double[]> settled = new ArrayList<>(); // {w, v}
        double maxSeen = 0;

        for (int i = 0; i < POWERS.length && opModeIsActive(); i++) {
            double power = POWERS[i];
            launcher.setOpenLoopPower(power);
            double start = getRuntime();
            List<double[]> window = new ArrayList<>(); // {t, v}
            while (opModeIsActive()) {
                LauncherMPC.State s = launcher.getState();
                double t = getRuntime() - start;
                if (s != null) {
                    window.add(new double[]{t, s.velocity, s.battery});
                    maxSeen = Math.max(maxSeen, s.velocity);
                }
                // Settled: the mean of the newer half of the last SETTLE_WINDOW_S differs from the
                // older half by < 1% (or 10 t/s). Means, not min/max, because the encoder is noisy.
                boolean isSettled = false;
                double avgV = 0, avgB = 0;
                if (t > MIN_HOLD_S && window.size() > 10) {
                    double sumOld = 0, sumNew = 0;
                    int nOld = 0, nNew = 0;
                    for (int k = window.size() - 1; k >= 0 && window.get(k)[0] > t - SETTLE_WINDOW_S; k--) {
                        double[] w = window.get(k);
                        if (w[0] > t - SETTLE_WINDOW_S / 2) {
                            sumNew += w[1];
                            nNew++;
                        } else {
                            sumOld += w[1];
                            nOld++;
                        }
                        avgV += w[1];
                        avgB += w[2];
                    }
                    int m = nOld + nNew;
                    avgV /= Math.max(m, 1);
                    avgB /= Math.max(m, 1);
                    double drift = nOld > 0 && nNew > 0 ? Math.abs(sumNew / nNew - sumOld / nOld) : Double.MAX_VALUE;
                    isSettled = drift < Math.max(0.01 * Math.abs(avgV), 10) || t > MAX_HOLD_S;
                }
                telemetry.addLine(String.format("Step %d/%d   power %.2f   %.1f s", i + 1, POWERS.length, power, t));
                if (s != null) {
                    telemetry.addData("Velocity", "%.0f ticks/s", s.velocity);
                    telemetry.addData("Battery", "%.2f V", s.battery);
                    telemetry.addData("Loop", "%.1f ms", s.loopMs);
                }
                telemetry.update();
                if (isSettled) {
                    if (power > 0 && avgV > 0) settled.add(new double[]{power * avgB / before.nominalVoltage, avgV});
                    break;
                }
                sleep(20);
            }
        }
        // coast for a bit (coast-down data is useful for kS / tau)
        launcher.off();
        double coastStart = getRuntime();
        while (opModeIsActive() && getRuntime() - coastStart < 2.0) {
            telemetry.addLine("Coasting...");
            telemetry.update();
            sleep(50);
        }
        launcher.stopLogging();
        if (!opModeIsActive()) {
            launcher.close();
            return;
        }

        // ------------------------------------------------------------------ fit
        telemetry.addLine("Fitting...");
        telemetry.update();
        double[][] log = launcher.getLog(); // t,target,velocity,estimate,load,power,battery,...
        int n = log.length;
        double[] t = new double[n], v = new double[n], u = new double[n], b = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = log[i][0];
            v[i] = log[i][2];
            u[i] = log[i][5];
            b[i] = log[i][6];
        }
        SysIdFit.Result fit = SysIdFit.fit(t, v, u, b, before.nominalVoltage, 0.1);
        double[] ws = new double[settled.size()], vs = new double[settled.size()];
        for (int i = 0; i < settled.size(); i++) {
            ws[i] = settled.get(i)[0];
            vs[i] = settled.get(i)[1];
        }
        double[] ss = SysIdFit.fitSteadyState(ws, vs);

        File csv = null;
        try {
            csv = launcher.dumpLog("sysid");
        } catch (Exception ignored) {
        }

        String status = "";
        while (opModeIsActive()) {
            telemetry.addLine("SYSID RESULT");
            if (fit.ok) {
                telemetry.addData("tau", "%.4f s   (was %.4f)", fit.tau, before.tau);
                telemetry.addData("kV", "%.1f ticks/s per power   (was %.1f)", fit.kV, before.kV);
                telemetry.addData("kS", "%.1f ticks/s   (was %.1f)", fit.kS, before.kS);
                telemetry.addData("delay", "%.0f ms   (was %.0f)", fit.delaySec * 1000, before.delaySec * 1000);
                telemetry.addData("fit error", "%.1f ticks/s RMS   r² %.4f", fit.rms, fit.r2);
                if (ss != null) {
                    telemetry.addData("check (steady-state)", "kV %.0f  kS %.0f  r² %.4f", ss[0], ss[1], ss[2]);
                }
                double vMax = fit.kV - fit.kS;
                telemetry.addData("Top speed @12V", "%.0f ticks/s   (%.0f RPM)", vMax, vMax * 60 / before.ticksPerRev);
                telemetry.addLine(String.format("Keep shooting targets ≤ %.0f ticks/s (85%%) for recovery headroom.", 0.85 * vMax));
                if (fit.r2 < 0.98) telemetry.addLine("⚠ Poor fit. Something rubbing/slipping? Encoder on the right motor?");
                if (ss != null && Math.abs(ss[0] - fit.kV) > 0.1 * fit.kV) {
                    telemetry.addLine("⚠ Dynamic and steady-state kV disagree >10%: check for belt slip / battery sag.");
                }
                telemetry.addLine();
                telemetry.addLine("A = SAVE   B = discard   X = write CSV");
            } else {
                telemetry.addLine("FIT FAILED: " + fit.message);
                telemetry.addLine("Positive power must spin the flywheel the shooting way AND read positive velocity.");
                telemetry.addLine("Fix motor directions in LauncherMPC.defaultConfig().");
            }
            if (csv != null) telemetry.addData("Log", csv.getPath());
            telemetry.addLine(status);
            telemetry.update();

            if (fit.ok && gamepad1.aWasPressed()) {
                LauncherParams p = launcher.getParams();
                p.tau = fit.tau;
                p.kV = fit.kV;
                p.kS = fit.kS;
                p.delaySec = fit.delaySec;
                // good starting points that follow from the model
                p.horizonSec = Math.max(0.2, Math.min(1.5, fit.tau));
                p.measNoise = Math.max(10, fit.rms * 1.5);
                launcher.setParams(p);
                try {
                    p.save(launcher.getParamsFile(), "LauncherSysId");
                    status = "Saved to " + launcher.getParamsFile() + ". Next: Launcher MPC Tuner.";
                } catch (Exception e) {
                    status = "SAVE FAILED: " + e.getMessage();
                }
            }
            if (gamepad1.bWasPressed()) status = "Discarded (file unchanged).";
            if (gamepad1.xWasPressed()) {
                try {
                    csv = launcher.dumpLog("sysid");
                    status = "Wrote " + csv.getName();
                } catch (Exception e) {
                    status = "CSV failed: " + e.getMessage();
                }
            }
            sleep(20);
        }
        launcher.close();
    }
}
