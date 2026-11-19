import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;
import org.firstinspires.ftc.teamcode.subsystems.launcher.MpcCore;
import org.firstinspires.ftc.teamcode.subsystems.launcher.StepAnalyzer;
import org.firstinspires.ftc.teamcode.subsystems.launcher.SysIdFit;
import org.firstinspires.ftc.teamcode.subsystems.launcher.TuneScore;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Desktop simulator for the launcher MPC. Runs the exact controller code from TeamCode against a
 * simulated flywheel, so you can try weights at your desk before touching the robot.
 *
 *   ./tools/launcher/sim.sh run    [params.properties] [--csv out.csv]
 *   ./tools/launcher/sim.sh sweep  [params.properties]
 *   ./tools/launcher/sim.sh sysid  [params.properties]
 *
 * params.properties: pull the robot's file with
 *   adb pull /sdcard/FIRST/launcher/launcher.properties
 * Without one, the defaults in LauncherParams are used.
 *
 * Options:
 *   --mismatch X   true plant kV, tau and kS are off from the model by ±X (e.g. 0.15 = 15%)
 *   --shots N      ball shots in the scenario (default 3)
 *   --shotDrop X   speed lost per shot as a fraction of speed (default 0.12)
 *   --target T     high target, ticks/s (default 80% of max)
 *   --noise S      encoder noise std-dev, ticks/s (default 15)
 *   --seed N
 */
public class LauncherSim {

    // ------------------------------------------------------------------ simulated hardware
    static final class Plant {
        double tau, kV, kS, delaySec, noise, vNom = 12.0;
        double v = 0;
        /** Fraction of speed still to be removed by the ball currently going through, and over how long. */
        double shotLeft = 0, shotRate = 0;
        final Random rnd;
        final ArrayList<double[]> queue = new ArrayList<>(); // {timeEffective, power}
        double powerNow = 0;

        Plant(LauncherParams model, double mismatch, double noise, long seed) {
            rnd = new Random(seed);
            this.tau = model.tau * (1 + mismatch);
            this.kV = model.kV * (1 - mismatch);
            this.kS = model.kS * (1 + 2 * mismatch);
            this.delaySec = model.delaySec + 0.004; // real latency is never exactly what you fit
            this.noise = noise;
        }

        double battery() {
            return 12.6 - 1.2 * Math.abs(powerNow); // sag under load
        }

        void command(double t, double power) {
            queue.add(new double[]{t + delaySec, power});
        }

        /** Advance physics from t to t+dt in 0.5 ms substeps. */
        void advance(double t, double dt) {
            double h = 0.0005;
            for (double s = 0; s < dt - 1e-9; s += h) {
                double now = t + s;
                while (!queue.isEmpty() && queue.get(0)[0] <= now) powerNow = queue.remove(0)[1];
                double w = powerNow * battery() / vNom;
                double fr = v > 0 ? kS : (w * kV > kS ? kS : w * kV); // static friction holds at 0
                double dv = (-v + kV * w - fr) / tau;
                if (shotLeft > 0) { // ball contact: lose speed over SHOT_SEC
                    double f = Math.min(shotLeft, shotRate * h);
                    dv -= v * f / h;
                    shotLeft -= f;
                }
                v = Math.max(0, v + dv * h);
            }
        }

        double measure() {
            // REV hub reports integer ticks/s
            return Math.round(v + rnd.nextGaussian() * noise);
        }
    }

    static final class Sample {
        double t, target, v, vHat, load, u, batt;
    }

    // ------------------------------------------------------------------ scenario
    static final class Scenario {
        double high, low, shotDrop;
        int shots;
    }

    static double targetAt(Scenario sc, double t) {
        if (t < 0.3) return 0;
        if (t < 3.0) return sc.high;           // spin-up from rest
        if (t < 5.5) return sc.low;            // step down
        if (t < 9.0 + sc.shots * 0.6) return sc.high; // step up, settle, then shots at 8.0, 8.6, ...
        return 0;
    }

    static List<Sample> simulate(LauncherParams params, Scenario sc, Plant plant, long seed, TuneScore score) {
        MpcCore mpc = new MpcCore();
        mpc.setParams(params);
        LauncherParams p = mpc.getParams();
        Random jitter = new Random(seed + 7);
        List<Sample> log = new ArrayList<>();
        double t = 0, period = p.periodMs / 1000.0, end = 9.0 + sc.shots * 0.6 + 1.0;
        int nextShot = 0;
        double dt = period;
        while (t < end) {
            double target = targetAt(sc, t);
            if (nextShot < sc.shots && t >= 8.0 + nextShot * 0.6) {
                plant.shotLeft = sc.shotDrop;
                plant.shotRate = sc.shotDrop / 0.03; // ~30 ms of ball contact
                nextShot++;
            }
            double batt = plant.battery();
            double meas = plant.measure();
            LauncherParams cp = mpc.getParams();
            boolean arm = target > 0 && Math.abs(mpc.getVelocityEstimate() - target)
                    < 3 * Math.max(cp.readyTolFrac * target, cp.readyTolMin);
            mpc.observe(meas, dt, arm);
            double u;
            if (target > 0) {
                u = mpc.solve(target, batt);
            } else {
                u = 0;
                mpc.resetPlan(0);
            }
            mpc.commit(u, batt);
            plant.command(t, u);

            Sample s = new Sample();
            s.t = t; s.target = target; s.v = plant.v; s.vHat = mpc.getVelocityEstimate();
            s.load = mpc.getLoadEstimate(); s.u = u; s.batt = batt;
            log.add(s);
            if (score != null) score.add(t, target, meas, u);

            dt = period + (jitter.nextDouble() - 0.3) * 0.004; // real loops jitter, mostly late
            plant.advance(t, dt);
            t += dt;
        }
        return log;
    }

    // ------------------------------------------------------------------ main
    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.US);
        String mode = args.length > 0 ? args[0] : "run";
        String file = null, csv = null;
        double mismatch = 0.0, shotDrop = 0.12, noise = 15, target = Double.NaN;
        int shots = 3;
        long seed = 1;
        for (int i = 1; i < args.length; i++) {
            switch (args[i]) {
                case "--csv": csv = args[++i]; break;
                case "--mismatch": mismatch = Double.parseDouble(args[++i]); break;
                case "--shots": shots = Integer.parseInt(args[++i]); break;
                case "--shotDrop": shotDrop = Double.parseDouble(args[++i]); break;
                case "--noise": noise = Double.parseDouble(args[++i]); break;
                case "--target": target = Double.parseDouble(args[++i]); break;
                case "--seed": seed = Long.parseLong(args[++i]); break;
                default: file = args[i];
            }
        }
        LauncherParams params = file != null ? LauncherParams.loadOrDefault(new File(file)) : new LauncherParams();
        if (file != null && !new File(file).exists()) System.out.println("(no file " + file + ", using defaults)");
        double vMax = params.kV - params.kS;
        Scenario sc = new Scenario();
        sc.high = Double.isNaN(target) ? 0.7 * vMax : target;
        sc.low = 0.6 * sc.high;
        sc.shots = shots;
        sc.shotDrop = shotDrop;

        switch (mode) {
            case "run": run(params, sc, mismatch, noise, seed, csv); break;
            case "sweep": sweep(params, sc, mismatch, noise, seed); break;
            case "sysid": sysid(params, noise, mismatch, seed); break;
            default:
                System.out.println("usage: run|sweep|sysid [params.properties] [options]");
        }
    }

    static void run(LauncherParams params, Scenario sc, double mismatch, double noise, long seed, String csv)
            throws Exception {
        LauncherParams sp = params.sanitized();
        TuneScore ts = new TuneScore(sp.readyTolFrac, sp.readyTolMin);
        Plant plant = new Plant(params, mismatch, noise, seed);
        long t0 = System.nanoTime();
        List<Sample> log = simulate(params, sc, plant, seed, ts);
        double us = (System.nanoTime() - t0) / 1e3 / log.size();
        System.out.println("params: " + params.sanitized());
        System.out.printf("plant: tau=%.3f kV=%.0f kS=%.0f delay=%.3f (mismatch %.0f%%)%n",
                plant.tau, plant.kV, plant.kS, plant.delaySec, mismatch * 100);
        for (StepAnalyzer.Result r : ts.results) System.out.println("  " + r.summary());
        System.out.printf("power chatter %.4f /loop   score %.3f   (%.1f µs per control step here)%n",
                ts.chatter(), ts.score(), us);
        if (csv != null) {
            try (PrintWriter pw = new PrintWriter(csv)) {
                pw.println("t,target,velocity,estimate,load,power,battery");
                for (Sample s : log) {
                    pw.printf("%.4f,%.1f,%.1f,%.1f,%.1f,%.4f,%.3f%n", s.t, s.target, s.v, s.vHat, s.load, s.u, s.batt);
                }
            }
            System.out.println("wrote " + csv);
        }
    }

    static void sweep(LauncherParams base, Scenario sc, double mismatch, double noise, long seed) {
        double[] rDus = {0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5};
        double[] velN = {base.velNoise * 0.5, base.velNoise, base.velNoise * 3, base.velNoise * 10};
        double[] loads = {1, 2, 4, 8, 16};
        double[] meas = {base.measNoise * 0.5, base.measNoise, base.measNoise * 2};
        class Row { double score, chatter; LauncherParams p; }
        List<Row> rows = new ArrayList<>();
        for (double rDu : rDus) for (double vn : velN) for (double ln : loads) for (double mn : meas) {
            LauncherParams p = base.copy();
            p.rDu = rDu;
            p.velNoise = vn;
            p.loadNoise = ln;
            p.measNoise = mn;
            double ch = 0;
            // average over model mismatch both ways so the winner is robust, not lucky
            double s = 0;
            double[] mm = mismatch == 0 ? new double[]{-0.1, 0, 0.1} : new double[]{-mismatch, 0, mismatch};
            for (double m : mm) {
                TuneScore ts = new TuneScore(base.readyTolFrac, base.readyTolMin);
                simulate(p, sc, new Plant(base, m, noise, seed), seed, ts);
                s += ts.score();
                ch += ts.chatter();
            }
            Row r = new Row();
            r.score = s / mm.length;
            r.chatter = ch / mm.length;
            r.p = p;
            rows.add(r);
        }
        rows.sort((a, b) -> Double.compare(a.score, b.score));
        System.out.println("Best settings (score: lower is better; averaged over ±model error):");
        System.out.println("  score  chatter  rDu     velNoise   loadNoise measNoise");
        for (int i = 0; i < Math.min(10, rows.size()); i++) {
            Row r = rows.get(i);
            System.out.printf("  %.3f  %.4f   %-7.3f %-10.3f %-9.1f %-9.1f%n",
                    r.score, r.chatter, r.p.rDu, r.p.velNoise, r.p.loadNoise, r.p.measNoise);
        }
        TuneScore ts = new TuneScore(base.readyTolFrac, base.readyTolMin);
        simulate(base, sc, new Plant(base, 0, noise, seed), seed, ts);
        System.out.printf("current params (no model error): score %.3f chatter %.4f%n", ts.score(), ts.chatter());
        System.out.println("Copy the winners into the tuner on the robot (or edit launcher.properties), then verify there.");
    }

    /** Simulate the SysId OpMode's open-loop sequence on a known plant and check the fit recovers it. */
    static void sysid(LauncherParams model, double noise, double mismatch, long seed) {
        Plant plant = new Plant(model, mismatch, noise, seed);
        double[] seq = SysIdFit.TEST_POWERS;
        double period = model.periodMs / 1000.0, t = 0;
        Random jitter = new Random(seed);
        List<double[]> rows = new ArrayList<>();
        for (double pw : seq) {
            double holdEnd = t + 2.5;
            while (t < holdEnd) {
                double meas = plant.measure();
                double batt = plant.battery();
                plant.command(t, pw);
                rows.add(new double[]{t, meas, pw, batt});
                double dt = period + (jitter.nextDouble() - 0.3) * 0.004;
                plant.advance(t, dt);
                t += dt;
            }
        }
        int n = rows.size();
        double[] tt = new double[n], vv = new double[n], uu = new double[n], bb = new double[n];
        for (int i = 0; i < n; i++) {
            tt[i] = rows.get(i)[0]; vv[i] = rows.get(i)[1]; uu[i] = rows.get(i)[2]; bb[i] = rows.get(i)[3];
        }
        SysIdFit.Result r = SysIdFit.fit(tt, vv, uu, bb, 12.0, 0.1);
        System.out.printf("true : tau=%.4f kV=%.1f kS=%.1f delay=%.3f%n", plant.tau, plant.kV, plant.kS, plant.delaySec);
        System.out.println("fit  : " + r);
    }
}
