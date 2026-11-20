package org.firstinspires.ftc.teamcode.subsystems.launcher;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Finds MPC weights automatically by trying them on the real launcher. Pure Java: the OpMode and
 * the simulator both drive it the same way:
 * <pre>
 *   LauncherParams cand;
 *   while ((cand = tuner.next()) != null) {
 *       score = run one {@link AutoTuneTrial} with cand;
 *       tuner.report(score);
 *   }
 *   if (tuner.improved()) save(tuner.best());
 * </pre>
 *
 * Search: coordinate descent in log space over {@link #NAMES}.
 * <ul>
 *   <li>One unscored warm-up trial first (the filter and the motor settle in).</li>
 *   <li>For each parameter: re-measure the current best (the launcher drifts as it warms up and the
 *       battery drains, so an old score is not a fair reference), then try ×2 and ÷2 and keep
 *       going in a direction while it keeps improving. A change only counts if it beats the
 *       average of the best's recent scores by {@link #MARGIN}. A candidate that looks better
 *       but not by enough to be sure is tested a second time and judged on its average.</li>
 *   <li>Repeat whole passes until a pass changes nothing (max {@link #MAX_PASSES}) or the trial
 *       budget runs out. Steps stay ×2: smaller steps change the score less than the noise.</li>
 *   <li>Finally the winner and the starting values are re-measured alternately, twice each
 *       (winner, start, winner, start). The result only counts as "improved" if the winner is
 *       still better on average, so neither a lucky trial nor drift can make it win.</li>
 * </ul>
 */
public final class AutoTuner {

    public static final String[] NAMES = {"rDu", "loadNoise", "measNoise"};
    private static final double[] MIN = {1e-3, 0.5, 5};
    private static final double[] MAX = {2.0, 100, 300};
    /** Required relative improvement to accept a change. */
    public static final double MARGIN = 0.03;
    public static final int MAX_PASSES = 3;
    private static final double FACTOR = 2.0;

    private enum Phase { WARMUP, BASELINE, REMEASURE, TRY_UP, TRY_DOWN, CONTINUE, CONFIRM_BEST, CONFIRM_START, DONE }

    private final LauncherParams start;
    private final int maxTrials;

    private LauncherParams best, candidate;
    private final List<Double> bestScores = new ArrayList<>(), startScores = new ArrayList<>();
    private final List<Double> confirmBest = new ArrayList<>(), confirmStart = new ArrayList<>();
    private static final int CONFIRM_PAIRS = 2;
    private Phase phase = Phase.WARMUP;
    private int param = 0, dir = 0, trials = 0, pass = 1;
    private boolean changedThisPass = false;
    /** First score of a candidate being re-tested, or NaN. */
    private double retestFirst = Double.NaN;
    private boolean improved = false;
    private String lastDescription = "";
    public final List<String> history = new ArrayList<>();

    /**
     * @param start     current parameters (model from SysId should already be in them)
     * @param maxTrials search budget, not counting the 2 confirmation trials
     */
    public AutoTuner(LauncherParams start, int maxTrials) {
        this.start = start.sanitized();
        this.best = this.start.copy();
        this.maxTrials = Math.max(1, maxTrials);
    }

    /** Next parameters to try, or null when finished. */
    public LauncherParams next() {
        while (true) {
            switch (phase) {
                case WARMUP:
                    candidate = start.copy();
                    lastDescription = "warm-up (not scored)";
                    return candidate;
                case BASELINE:
                    candidate = start.copy();
                    lastDescription = "baseline (current values)";
                    return candidate;
                case REMEASURE:
                    if (trials >= maxTrials) {
                        beginConfirm();
                        continue;
                    }
                    candidate = best.copy();
                    lastDescription = "re-measure best";
                    return candidate;
                case TRY_UP:
                case TRY_DOWN:
                case CONTINUE: {
                    if (!Double.isNaN(retestFirst)) { // close call: same candidate again
                        lastDescription = "re-test: " + lastDescription.replace("re-test: ", "");
                        return candidate;
                    }
                    if (trials >= maxTrials) {
                        beginConfirm();
                        continue;
                    }
                    int d = phase == Phase.TRY_UP ? 1 : phase == Phase.TRY_DOWN ? -1 : dir;
                    double cur = get(best, param);
                    double v = clamp(param, cur * Math.pow(FACTOR, d));
                    if (Math.abs(v - cur) < 1e-9 * Math.max(1, cur)) { // at a bound: nothing to try
                        reject();
                        continue;
                    }
                    candidate = best.copy();
                    set(candidate, param, v);
                    lastDescription = String.format(Locale.US, "%s %s → %s", NAMES[param], fmt(cur), fmt(v));
                    return candidate;
                }
                case CONFIRM_BEST:
                    candidate = best.copy();
                    lastDescription = "confirm winner";
                    return candidate;
                case CONFIRM_START:
                    candidate = start.copy();
                    lastDescription = "confirm starting values";
                    return candidate;
                default:
                    return null;
            }
        }
    }

    /** Score of the trial run with the last {@link #next()} result. Lower is better. */
    public void report(double score) {
        history.add(String.format(Locale.US, "%2d %-30s %.3f", history.size() + 1, lastDescription, score));
        switch (phase) {
            case WARMUP:
                phase = Phase.BASELINE;
                break;
            case BASELINE:
                bestScores.add(score);
                startScores.add(score);
                phase = Phase.TRY_UP;
                break;
            case REMEASURE:
                trials++;
                bestScores.add(score);
                phase = Phase.TRY_UP;
                break;
            case TRY_UP:
            case TRY_DOWN:
            case CONTINUE:
                trials++;
                double ref = reference();
                if (Double.isNaN(retestFirst) && score < ref && score >= ref * (1 - MARGIN) && trials < maxTrials) {
                    retestFirst = score; // looks better, but within the noise: test it again
                    break;
                }
                if (!Double.isNaN(retestFirst)) {
                    score = (score + retestFirst) / 2;
                    retestFirst = Double.NaN;
                }
                if (score < ref * (1 - MARGIN)) {
                    best = candidate;
                    changedThisPass = true;
                    bestScores.clear();
                    bestScores.add(score);
                    dir = phase == Phase.TRY_UP ? 1 : phase == Phase.TRY_DOWN ? -1 : dir;
                    phase = Phase.CONTINUE;
                } else {
                    reject();
                }
                break;
            case CONFIRM_BEST:
                confirmBest.add(score);
                phase = Phase.CONFIRM_START;
                break;
            case CONFIRM_START:
                confirmStart.add(score);
                if (confirmStart.size() < CONFIRM_PAIRS) {
                    phase = Phase.CONFIRM_BEST;
                } else {
                    // alternating pairs, so drift affects both equally
                    improved = mean(confirmBest) < mean(confirmStart) * (1 - MARGIN);
                    phase = Phase.DONE;
                }
                break;
            default:
                break;
        }
    }

    /** Average of the current best's recent scores (last 3): the bar a candidate must beat. */
    private double reference() {
        int n = bestScores.size();
        double s = 0;
        int k = 0;
        for (int i = Math.max(0, n - 3); i < n; i++, k++) s += bestScores.get(i);
        return s / Math.max(k, 1);
    }

    /** No improvement in this direction: try the other way, or move to the next parameter. */
    private void reject() {
        if (phase == Phase.TRY_UP) {
            phase = Phase.TRY_DOWN;
            return;
        }
        param++;
        dir = 0;
        if (param >= NAMES.length) {
            param = 0;
            if (!changedThisPass || pass >= MAX_PASSES) {
                beginConfirm();
                return;
            }
            pass++;
            changedThisPass = false;
        }
        phase = Phase.REMEASURE;
    }

    private void beginConfirm() {
        if (sameAs(best, start)) { // nothing beat the starting values: no need to confirm
            improved = false;
            phase = Phase.DONE;
        } else {
            phase = Phase.CONFIRM_BEST;
        }
    }

    // ------------------------------------------------------------------ results

    public boolean isDone() {
        return phase == Phase.DONE;
    }

    /** True if the winner is confirmed better than the starting values. Only then save it. */
    public boolean improved() {
        return improved;
    }

    /** Best parameters found (the starting values if nothing was better). */
    public LauncherParams best() {
        return improved ? best.copy() : start.copy();
    }

    /** Starting values' score from the final back-to-back check (or the baseline if no check ran). */
    public double startScore() {
        return confirmStart.isEmpty() ? mean(startScores) : mean(confirmStart);
    }

    /** Winner's score from the final back-to-back check. */
    public double bestScore() {
        if (!improved) return startScore();
        return mean(confirmBest);
    }

    public int pass() {
        return pass;
    }

    /** Trials still to run at most (search budget left + confirmation). */
    public int trialsLeftAtMost() {
        if (phase == Phase.DONE) return 0;
        int confirmLeft = 2 * CONFIRM_PAIRS - confirmBest.size() - confirmStart.size();
        if (phase == Phase.CONFIRM_BEST || phase == Phase.CONFIRM_START) return confirmLeft;
        int pre = phase == Phase.WARMUP ? 2 : phase == Phase.BASELINE ? 1 : 0;
        return pre + (maxTrials - trials) + 2 * CONFIRM_PAIRS;
    }

    public String currentDescription() {
        return lastDescription;
    }

    public String phaseName() {
        return phase.name();
    }

    public static double get(LauncherParams p, int i) {
        switch (i) {
            case 0: return p.rDu;
            case 1: return p.loadNoise;
            default: return p.measNoise;
        }
    }

    private static void set(LauncherParams p, int i, double v) {
        switch (i) {
            case 0: p.rDu = v; break;
            case 1: p.loadNoise = v; break;
            default: p.measNoise = v; break;
        }
    }

    private static double clamp(int i, double v) {
        return Math.max(MIN[i], Math.min(MAX[i], v));
    }

    private static boolean sameAs(LauncherParams a, LauncherParams b) {
        for (int i = 0; i < NAMES.length; i++) if (get(a, i) != get(b, i)) return false;
        return true;
    }

    private static double mean(List<Double> xs) {
        double s = 0;
        for (double x : xs) s += x;
        return xs.isEmpty() ? Double.NaN : s / xs.size();
    }

    public static String fmt(double v) {
        double a = Math.abs(v);
        if (a >= 100) return String.format(Locale.US, "%.0f", v);
        if (a >= 1) return String.format(Locale.US, "%.2f", v);
        return String.format(Locale.US, "%.4f", v);
    }
}
