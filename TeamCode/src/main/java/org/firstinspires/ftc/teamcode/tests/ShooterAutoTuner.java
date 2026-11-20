package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel auto-tuner for {@link Shooter}. The Shooter class itself is never modified.
 *
 * <p>Goal: recover quickly when a launched ball drags the flywheel down, while holding the
 * target as tightly as possible. Every closed-loop test runs through the REAL control law in
 * Shooter.update(), including the useRaw bang-bang zone.
 *
 * <h3>How the real control law is driven</h3>
 * Shooter.update() is the only thing that writes motor power, and it overwrites whatever
 * power(x) was set earlier. So instead of fighting it, this tuner sets the public static gains
 * and lets update() do the work:
 * <ul>
 * <li>Open-loop power p: kP = 0, kS = 0, useRaw = 1e9, target = 1, kV = p. update() then
 * outputs exactly max(0, p * 1) = p.</li>
 * <li>Closed loop: set the real kV/kS/kP/useRaw, target(t), enable().</li>
 * </ul>
 *
 * <h3>Phases</h3>
 * <ol>
 * <li>Ramp: settle at each open-loop power, fit power = kV * v + kS, report R^2 and tau.</li>
 * <li>Feedforward check at the test target: open-loop steady-state error, sensor noise, and an
 * optional one-step kV trim.</li>
 * <li>kP search (log grid, then refine) and useRaw search, using simulated ball drops: the
 * controller holds target, we cut power until velocity falls by DIP_TICKS, then re-enable
 * and measure recovery.</li>
 * <li>Re-test of the top candidates, then one drop at each CHECK_TARGETS value (matches use a
 * range of speeds, not just TEST_TARGET).</li>
 * <li>Live verification: hold the target and measure real ball launches until Stop. Reports the
 * real dip depth so DIP_TICKS can be set from it.</li>
 * </ol>
 *
 * <h3>Cost function (lower is better)</h3>
 * <pre>
 * cost = recoveryTime_ms
 * + W_OVERSHOOT * overshoot_ticks_per_s (above target + noise deadband, after the dip)
 * + W_HOLD * holdVelocityStdDev (ticks/s, while settled, before the drop)
 * + W_OFFSET * |holdMeanError| (ticks/s, steady offset from target while holding)
 * + W_CHATTER * holdPowerStdDev (motor power units, while settled)
 * </pre>
 * Defaults make 1 ms of recovery worth about 0.5 ticks/s of overshoot, 0.1 ticks/s of hold
 * jitter, 0.1 ticks/s of steady offset, or 0.002 of power chatter. Candidates with sustained
 * oscillation (>= 3 deadband crossings) or no recovery within the timeout are rejected outright.
 * The offset term matters because a tiny kP barely corrects feedforward error. Shooter's
 * CachedMotor ignores power changes under 0.01. Writing e = target - v, the commanded power moves
 * kP * e away from the last written value (feedforward uses the target, so only the P term moves
 * with velocity), so the loop is blind to error changes under 0.01 / kP ticks/s. Candidates whose blind zone exceeds
 * MAX_RES_BAND_FRACTION of the band are rejected, and the zone is reported.
 *
 * <h3>useRaw edge</h3>
 * At err = useRaw the P zone outputs FF + kP * useRaw, while bang-bang outputs 1.0. If
 * kP * useRaw < 1 - FF there is a power jump at the edge (the flywheel has to pass through a
 * slower tail). The continuous value (1 - FF) / kP, taken at the lowest target in use, is added to
 * the useRaw candidates and the remaining jump is reported.
 *
 * <h3>Picking the winner</h3>
 * Costs from two drops are noisy and the cost curve is flat near the optimum, so the minimum
 * cost is not the best pick. The best few candidates (within RETEST_WITHIN of the best, at most
 * RETEST_TOP) get extra drops that are merged into their averages before picking. Among candidates within KEEP_WITHIN of the best cost we choose:
 * <ul>
 * <li>kP: the SMALLEST (stays away from the oscillation edge; the old tuner's problem).</li>
 * <li>useRaw: the SMALLEST (bang-bang full power engages for smaller dips, so real balls that
 * dip less than the simulated drop still get a fast recovery; a wide PID zone would
 * recover them slowly). Holding is not affected: candidates are at least 2x the noise
 * band, so the hold always stays in the PID zone.</li>
 * </ul>
 * Candidates whose hold offset is bigger than the noise deadband are skipped as tie-break picks.
 *
 * <p>kV is a duty-cycle gain, so it scales with battery voltage. It is only valid near the
 * voltage printed on the final screen.
 */
@Config
@TeleOp(name = "Shooter AutoTuner", group = "Tests")
public class ShooterAutoTuner extends LinearOpMode {

// ================================================================
// TUNABLES (editable from FTC Dashboard before pressing Start)
// ================================================================

    /** Velocity (ticks/s) the control tests run at. Clamped to 90% of the max measured velocity. */
    public static double TEST_TARGET = 2180;

    // ---- Phase 1: ramp ----
    public static double RAMP_MIN_POWER = 0.15;
    public static double RAMP_MAX_POWER = 0.95;
    public static double RAMP_STEP = 0.05;
    /** Window over which the velocity slope is judged for "settled". */
    public static double SETTLE_WINDOW_MS = 250;
    /** Settled when velocity drifts less than max(this, 1% of velocity) across the window. */
    public static double SETTLE_DRIFT_TICKS = 15;
    /** Never declare settled sooner than this after a power change (hides the motor dead time). */
    public static double SETTLE_MIN_MS = 400;
    public static double SETTLE_TIMEOUT_MS = 2500;
    /** Averaging time after settling. */
    public static double AVERAGE_MS = 300;
    /** Samples below this velocity are ignored by the fit (static friction region). */
    public static double MIN_FIT_VELOCITY = 100;
    /** Abort if velocity is more negative than this under positive power. */
    public static double NEGATIVE_ABORT_VELOCITY = 50;
    /** Timeout for slower transitions (spin-down to the test target, stopping at the start). */
    public static double TRANSITION_TIMEOUT_MS = 5000;

// ---- Phase 2: feedforward check and noise ----
    /** Steady-open-loop hold time used for the noise measurement and FF error. */
    public static double NOISE_MS = 1000;
    public static double DEADBAND_NOISE_MULT = 3.0;
    /** Floor for the deadband so a perfectly quiet sensor does not give zero. */
    public static double DEADBAND_MIN = 5.0;
    /** Skip the kV trim if open-loop error at target is already below this (percent). */
    public static double FF_TRIM_SKIP_PERCENT = 0.5;

    // ---- Phase 2: drop test ----
    public static double DIP_TICKS = 250;
    /** Max time to wait for the flywheel to coast down by DIP_TICKS. */
    public static double DIP_MAX_MS = 1500;
    public static int DROPS_PER_CANDIDATE = 2;
    /** Pre-drop window used to measure hold quality. */
    public static double HOLD_MS = 1000;
    /** Must be inside the band this long before a drop test starts. */
    public static double SETTLE_BAND_STAY_MS = 300;
    public static double BAND_SETTLE_TIMEOUT_MS = 4000;
    /** Recovered = inside the band and staying there this long. */
    public static double RECOVERY_STAY_MS = 150;
    public static double RECOVERY_TIMEOUT_MS = 1500;
    /** Keep watching this long after recovery to catch overshoot and ringing. */
    public static double POST_RECOVERY_MS = 400;
    public static int OSC_CROSSINGS_MAX = 3;
    /** Shooter's CachedMotor skips power writes that change less than this. */
    public static final double CACHE_POWER_STEP = 0.01;
    /** Reject kP whose blind zone 0.01/kP is larger than this fraction of the band. */
    public static double MAX_RES_BAND_FRACTION = 0.5;

    // ---- Cost weights ----
    public static double W_OVERSHOOT = 2.0;
    public static double W_HOLD = 10.0;
    public static double W_OFFSET = 10.0;
    public static double W_CHATTER = 500.0;
    /** Candidates within this fraction of the best cost count as equivalent. */
    public static double KEEP_WITHIN = 0.10;

    // ---- Search ranges ----
    public static double KP_MIN = 0.0005;
    public static double KP_MAX = 0.04;
    public static int KP_GRID_POINTS = 8;
    public static int KP_REFINE_POINTS = 4;
    /** Comma separated. Candidates smaller than 2x the recovery band are skipped. */
    public static String USE_RAW_CANDIDATES = "60,90,120,180,250";
    /** The continuity value (1-FF)/kP is added as a candidate only if it is at most this. */
    public static double USE_RAW_CAP = 400;
    /** Candidates above this multiple of DIP_TICKS are skipped: a simulated drop cannot reach them. */
    public static double USE_RAW_MAX_DIP_MULT = 1.2;
    /** The best candidates within this cost fraction get extra drops (up to RETEST_TOP of them). */
    public static double RETEST_WITHIN = 0.25;
    public static int RETEST_TOP = 3;
    /** Comma separated speeds (ticks/s) where the final kP/useRaw get one drop each. Matches use ~1500-2250. */
    public static String CHECK_TARGETS = "1500,2250";

    // ---- Phase 3: live verification ----
    public static double LIVE_DIP_MIN = 60;
    public static double LIVE_DIP_NOISE_MULT = 5.0;
    public static double LIVE_TARGET_STEP = 50;

    /** Telemetry send period. Slower is better for loop time, faster is nicer for graphs. */
    public static double TELEMETRY_PERIOD_MS = 40;

// ================================================================
// INTERNALS
// ================================================================

    private static final int SCREEN_TUNING = 0;
    private static final int SCREEN_LIVE = 1;
    private static final int LOG_LINES = 10;

    /** Thrown from tick() when Stop is pressed so every wait loop unwinds without checks. */
    private static final class StopRequested extends RuntimeException {
    }

    /** A condition that makes continuing pointless; the message is shown to the driver. */
    private static final class TuneFailure extends RuntimeException {
        TuneFailure(String message) {
            super(message);
        }
    }

    private static final class Hold {
        boolean settled;
        double settleMs;
        double mean;
        /** Std dev of velocity around its own linear trend, i.e. sensor noise. */
        double std;
    }

    private static final class Drop {
        boolean ok;
        String reason = "";
        double recoveryMs, overshoot, holdStd, holdMeanErr, powerStd, dipDepth;
        int crossings;
    }

    private static final class Candidate {
        double kP, useRaw;
        /** Number of successful drops averaged into the metrics. */
        int n;
        double cost = Double.POSITIVE_INFINITY;
        double recoveryMs, overshoot, holdStd, holdMeanErr, powerStd, dipDepth;
        int crossings;
        boolean rejected;
        String reason = "";
    }

    /** Running mean/std with removal, for sliding windows. Values are shifted by the caller. */
    private static final class RunningStats {
        int n;
        double sum, sumSq;

        void add(double x) {
            n++;
            sum += x;
            sumSq += x * x;
        }

        void remove(double x) {
            n--;
            sum -= x;
            sumSq -= x * x;
        }

        void clear() {
            n = 0;
            sum = 0;
            sumSq = 0;
        }

        double mean() {
            return n > 0 ? sum / n : 0;
        }

        double std() {
            if (n < 2) {
                return 0;
            }
            return Math.sqrt(Math.max(0, (sumSq - sum * sum / n) / (n - 1)));
        }
    }

    private Shooter shooter;
    private DcMotorEx rawMotorForSafety;
    private MultipleTelemetry tele;
    private List<LynxModule> hubs;
    private final ElapsedTime clock = new ElapsedTime();

    // loop bookkeeping
    private double nowMs;
    private double lastTickMs = -1;
    private double loopSumMs, loopMaxMs;
    private long loopCount;
    private double lastTeleMs = -1e9;
    private double lastVoltageMs = -1e9;
    private double minVoltage = Double.POSITIVE_INFINITY;
    private double maxVoltage = Double.NEGATIVE_INFINITY;

    // screen
    private String status = "";
    private final List<String> log = new ArrayList<>();
    private int screenMode = SCREEN_TUNING;
    private double graphTarget = 0;
    private double tuneStartMs;

    // original statics, restored if tuning does not finish
    private double origKS, origKV, origKP, origUseRaw, origThreshold;
    private boolean tuningComplete = false;

    // tuning results
    private double kVFit, kSFit, fitR2, tauMs = Double.NaN, maxVelocity;
    private double kVUsed, kSUsed;
    private boolean kVTrimmed;
    private double ffErrFitPercent, ffErrTrimPercent = Double.NaN;
    private double testTarget, noise, deadband, band;
    private double tunedKP, tunedUseRaw;
    private Candidate finalCandidate;
    private int shortDipCount;
    private double lowTarget;
    private final List<Double> checkTargets = new ArrayList<>();
    private final List<String> checkLines = new ArrayList<>();

    // live verification stats
    private int liveShots, liveTimeouts;
    private double liveLastRec = Double.NaN, liveSumRec, liveWorstRec, liveLastOver;
    private final RunningStats liveHold = new RunningStats();
    private double liveTarget;
    private double liveVMin;
    private final List<Double> liveDips = new ArrayList<>();
    private int liveState;
    private boolean prevUp, prevDown;

// ================================================================
// OP MODE
// ================================================================

    @Override
    public void runOpMode() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
// Send to the driver station faster than the 250 ms default so velocity graphs are usable.
        telemetry.setMsTransmissionInterval(50);

        shooter = new Shooter(hardwareMap);
// Independent handle on the same motor, only used to guarantee a hard stop on exit.
        rawMotorForSafety = hardwareMap.get(DcMotorEx.class, "shooterBottom");

        hubs = hardwareMap.getAll(LynxModule.class);

        origKS = Shooter.kS;
        origKV = Shooter.kV;
        origKP = Shooter.kP;
        origUseRaw = Shooter.useRaw;
        origThreshold = Shooter.targetThreshold;

        shooter.disable();

        tele.addLine("Shooter AutoTuner ready.");
        tele.addLine("The flywheel will spin up to near full power. Keep it clear, no balls loaded.");
        tele.addData("Test target", "%.0f ticks/s (edit TEST_TARGET on Dashboard)", TEST_TARGET);
        tele.addLine("Press Start. Takes about 3-4 minutes, then verifies with real balls until Stop.");
        tele.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        for (LynxModule hub : hubs) {
// MANUAL: one bulk read per loop (cleared in tick()), so velocity timing is consistent.
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        clock.reset();
        tuneStartMs = clock.milliseconds();

        String failure = null;
        try {
            tune();
            tuningComplete = true;
            liveVerify();
        } catch (StopRequested e) {
// normal exit
        } catch (TuneFailure e) {
            failure = e.getMessage();
        } finally {
            motorOff();
            if (!tuningComplete) {
                restoreOriginals();
            }
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        if (failure != null) {
// Motor is already off and statics restored; just keep the reason on screen.
            while (opModeIsActive()) {
                tele.addLine("=== TUNING ABORTED ===");
                tele.addLine(failure);
                tele.addLine("Original Shooter values were restored. Press Stop.");
                tele.update();
                sleep(50);
            }
        }
    }

// ================================================================
// TOP LEVEL TUNING FLOW
// ================================================================

    private void tune() {
        waitUntilStopped();

// ---------- Phase 1: kV / kS ----------
        List<double[]> samples = runRamp();
        double[] fit = fitPowerVsVelocity(samples);
        kVFit = fit[0];
        kSFit = Math.max(0, fit[1]);
        fitR2 = rSquared(samples, kVFit, kSFit);
        kVUsed = kVFit;
        kSUsed = kSFit;

        maxVelocity = 0;
        for (double[] s : samples) {
            maxVelocity = Math.max(maxVelocity, s[1]);
        }
        if (kVFit <= 0) {
            throw new TuneFailure("Fit gave kV <= 0; velocity did not increase with power. "
                    + "Check the encoder and the motor direction.");
        }

        double maxAllowed = 0.9 * maxVelocity;
        testTarget = Math.min(TEST_TARGET, maxAllowed);
        if (testTarget < 3 * MIN_FIT_VELOCITY) {
            throw new TuneFailure("Test target " + fmt(testTarget, 0)
                    + " ticks/s is too low to tune on. Raise TEST_TARGET or check the max velocity ("
                    + fmt(maxVelocity, 0) + ").");
        }
        graphTarget = testTarget;
        buildCheckTargets();
        addLog(String.format(Locale.US, "kV %.8f kS %.4f R2 %.4f tau %s ms max %.0f",
                kVFit, kSFit, fitR2, Double.isNaN(tauMs) ? "n/a" : fmt(tauMs, 0), maxVelocity));
        if (testTarget < TEST_TARGET) {
            addLog("Target clamped " + fmt(TEST_TARGET, 0) + " -> " + fmt(testTarget, 0)
                    + " (90% of max)");
        }

// ---------- Phase 2a: feedforward check + noise ----------
        feedforwardCheck();

// ---------- Phase 2b: kP search ----------
        double kpStageUseRaw = Math.max(origUseRaw, 2 * band);
        List<Candidate> kpResults = new ArrayList<>();
        double kpFloor = CACHE_POWER_STEP / (Math.max(0.05, MAX_RES_BAND_FRACTION) * band);
        double kpMin = Math.max(Math.max(1e-5, KP_MIN), kpFloor);
        if (kpMin > KP_MIN) {
            addLog(String.format(Locale.US, "kP floor raised to %.5f (0.01 power step must stay under "
                    + "%.0f%% of band %.0f)", kpMin, 100 * MAX_RES_BAND_FRACTION, band));
        }
        double kpMax = Math.max(kpMin * 2, KP_MAX);
        int gridPoints = Math.max(2, KP_GRID_POINTS);
        double ratio = Math.pow(kpMax / kpMin, 1.0 / (gridPoints - 1));
        double[] grid = new double[gridPoints];
        for (int i = 0; i < gridPoints; i++) {
            grid[i] = kpMin * Math.pow(ratio, i);
        }
        for (int i = 0; i < grid.length; i++) {
            status = String.format(Locale.US, "kP grid %d/%d", i + 1, grid.length);
            kpResults.add(evaluate(grid[i], kpStageUseRaw));
        }

        int bestIdx = bestIndex(kpResults);
        if (bestIdx < 0) {
            throw new TuneFailure("Every kP candidate was rejected (oscillation or no recovery). "
                    + "Check that the flywheel is free, DIP_TICKS is sensible and the "
                    + "feedforward result above looks right.");
        }
// Refine log-uniformly around the best grid point (extend past the edge if it is an edge).
        double lo = bestIdx == 0 ? Math.max(kpMin, grid[0] / ratio) : grid[bestIdx - 1];
        double hi = bestIdx == grid.length - 1 ? grid[grid.length - 1] * ratio : grid[bestIdx + 1];
        for (int i = 1; i <= KP_REFINE_POINTS; i++) {
            double frac = i / (double) (KP_REFINE_POINTS + 1);
            double kp = lo * Math.pow(hi / lo, frac);
            status = String.format(Locale.US, "kP refine %d/%d", i, KP_REFINE_POINTS);
            kpResults.add(evaluate(kp, kpStageUseRaw));
        }

        retestTop(kpResults);
        Candidate kpPick = pickWithinBest(kpResults, true);
        if (kpPick == null) {
            throw new TuneFailure("All refined kP candidates were rejected.");
        }
        tunedKP = kpPick.kP;
        addLog(String.format(Locale.US, "kP pick %.5f (smallest within %.0f%% of best cost)",
                tunedKP, KEEP_WITHIN * 100));

// ---------- Phase 2c: useRaw search ----------
        List<Candidate> rawResults = new ArrayList<>();
        List<Double> rawCandidates = parseCsv(USE_RAW_CANDIDATES);
// Continuous edge: P zone output at err = useRaw equals 1.0 when useRaw = (1 - FF) / kP.
// Taken at the lowest target in use (smallest FF = largest value, so it is continuous there
// and merely saturated at higher targets).
        double ffLow = clamp01(kVUsed * lowTarget + kSUsed);
        double cont = ffLow < 1 ? Math.round((1 - ffLow) / tunedKP) : 0;
        if (cont >= 2 * band && cont <= USE_RAW_CAP) {
            boolean dup = false;
            for (double u : rawCandidates) {
                dup |= Math.abs(u - cont) <= 0.1 * cont;
            }
            if (!dup) {
                rawCandidates.add(cont);
                addLog(String.format(Locale.US, "useRaw %.0f added: continuous edge (1-FF)/kP at %.0f",
                        cont, lowTarget));
            }
        }
        Collections.sort(rawCandidates);
        int idx = 0;
        for (double u : rawCandidates) {
            idx++;
            if (u < 2 * band) {
                addLog(String.format(Locale.US, "useRaw %.0f skipped (< 2x band %.0f)", u, band));
                continue;
            }
            if (u > USE_RAW_MAX_DIP_MULT * DIP_TICKS) {
                addLog(String.format(Locale.US, "useRaw %.0f skipped (sim dip %.0f never reaches it)",
                        u, DIP_TICKS));
                continue;
            }
            status = String.format(Locale.US, "useRaw %d/%d", idx, rawCandidates.size());
            if (Math.abs(u - kpStageUseRaw) < 1e-9) {
// Exactly the configuration the kP stage already measured; do not re-measure.
                rawResults.add(kpPick);
            } else {
                rawResults.add(evaluate(tunedKP, u));
            }
        }
        if (rawResults.isEmpty()) {
            rawResults.add(kpPick);
        }
        retestTop(rawResults);
        Candidate rawPick = pickWithinBest(rawResults, false);
        if (rawPick == null) {
            rawPick = kpPick;
        }
        tunedUseRaw = rawPick.useRaw;
        finalCandidate = rawPick;

        runChecks();
        graphTarget = testTarget;
        addLog(String.format(Locale.US, "Tuning took %.0f s", (clock.milliseconds() - tuneStartMs) / 1000.0));

// Leave the tuned values in the statics so Shooter Test can use them immediately.
        Shooter.kV = kVUsed;
        Shooter.kS = kSUsed;
        Shooter.kP = tunedKP;
        Shooter.useRaw = tunedUseRaw;

        for (String line : resultLines()) {
            RobotLog.ii("ShooterAutoTuner", "%s", line); // never pass text as the format (it has % signs)
        }
    }

// ================================================================
// PHASE 1: RAMP
// ================================================================

    private List<double[]> runRamp() {
        List<double[]> samples = new ArrayList<>();
// 0.01 is the smallest power change Shooter's CachedMotor will actually write.
        double rampStep = Math.max(0.02, RAMP_STEP);
        double rampMin = clamp01(RAMP_MIN_POWER);
        double rampMax = Math.max(rampMin + rampStep, Math.min(1.0, RAMP_MAX_POWER));
        int steps = (int) Math.round((rampMax - rampMin) / rampStep);
        double prevMean = 0;
        boolean tauDone = false;

        for (int i = 0; i <= steps; i++) {
            double power = clamp01(rampMin + i * rampStep);
            status = String.format(Locale.US, "Ramp %d/%d power %.2f", i + 1, steps + 1, power);

            ArrayList<double[]> rise = tauDone ? null : new ArrayList<double[]>();
            Hold h = measureOpenLoop(power, SETTLE_TIMEOUT_MS, AVERAGE_MS, rise);
            sampleVoltage();

            if (h.mean < -NEGATIVE_ABORT_VELOCITY) {
                throw new TuneFailure(String.format(Locale.US,
                        "Velocity is NEGATIVE (%.0f ticks/s) under positive power %.2f. The motor "
                                + "direction and encoder direction disagree. Fix the motor direction "
                                + "(or encoder sign) in Shooter, then rerun.", h.mean, power));
            }

            if (!tauDone && h.settled && h.mean - prevMean >= 2 * MIN_FIT_VELOCITY) {
                tauMs = timeTo63Percent(rise, prevMean, h.mean);
                tauDone = !Double.isNaN(tauMs);
            }
            prevMean = h.mean;

// Unsettled points are biased, so they are only kept as a last resort below.
            samples.add(new double[]{power, h.mean, h.settled ? 1 : 0});
        }

        List<double[]> good = new ArrayList<>();
        List<double[]> any = new ArrayList<>();
        for (double[] s : samples) {
            if (s[1] >= MIN_FIT_VELOCITY) {
                any.add(s);
                if (s[2] > 0.5) {
                    good.add(s);
                }
            }
        }
        List<double[]> use = good.size() >= 4 ? good : any;
        if (use.size() < 3) {
            throw new TuneFailure("Fewer than 3 usable ramp points. The flywheel is not responding to "
                    + "power; check the encoder, motor name and direction.");
        }
        if (use != good) {
            addLog("WARNING: few settled ramp points, fit includes unsettled ones");
        }
        return use;
    }

    /** Time from the power command until velocity first reaches 63.2% of the step (two samples in a row). */
    private double timeTo63Percent(List<double[]> rise, double vStart, double vFinal) {
        double level = vStart + 0.632 * (vFinal - vStart);
        for (int i = 0; i + 1 < rise.size(); i++) {
            if (rise.get(i)[1] >= level && rise.get(i + 1)[1] >= level) {
                return rise.get(i)[0];
            }
        }
        return Double.NaN;
    }

    /**
     * Least squares of power = kV * v + kS. If the intercept comes out negative it is clamped to
     * zero and kV is refit through the origin (just clamping would leave kV biased).
     */
    private double[] fitPowerVsVelocity(List<double[]> samples) {
        int n = samples.size();
        double sx = 0, sy = 0, sxy = 0, sxx = 0;
        for (double[] s : samples) {
            double v = s[1];
            double p = s[0];
            sx += v;
            sy += p;
            sxy += v * p;
            sxx += v * v;
        }
        double denom = n * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) {
            return new double[]{0, sy / n};
        }
        double slope = (n * sxy - sx * sy) / denom;
        double intercept = (sy - slope * sx) / n;
        if (intercept < 0) {
            slope = sxy / sxx;
            intercept = 0;
        }
        return new double[]{slope, intercept};
    }

    private double rSquared(List<double[]> samples, double kV, double kS) {
        double mean = 0;
        for (double[] s : samples) {
            mean += s[0];
        }
        mean /= samples.size();
        double ssRes = 0, ssTot = 0;
        for (double[] s : samples) {
            double pred = kV * s[1] + kS;
            ssRes += (s[0] - pred) * (s[0] - pred);
            ssTot += (s[0] - mean) * (s[0] - mean);
        }
        return ssTot < 1e-12 ? 0 : 1 - ssRes / ssTot;
    }

// ================================================================
// PHASE 2A: FEEDFORWARD CHECK + NOISE
// ================================================================

    private void feedforwardCheck() {
        status = "Feedforward check at target";
        double pA = clamp01(kVUsed * testTarget + kSUsed);
// Long settle timeout: this step may have to spin DOWN from the top of the ramp.
        Hold a = measureOpenLoop(pA, TRANSITION_TIMEOUT_MS, NOISE_MS, null);
        sampleVoltage();

        noise = a.std;
        deadband = Math.max(DEADBAND_MIN, DEADBAND_NOISE_MULT * noise);
// A band tighter than the noise can never be "stayed in" for 150 ms, so never go below it.
        band = Math.max(Shooter.targetThreshold, deadband);

        ffErrFitPercent = 100.0 * (a.mean - testTarget) / testTarget;
        addLog(String.format(Locale.US, "FF @%.0f: p=%.3f -> %.0f (%+.2f%%) noise %.1f band %.0f%s",
                testTarget, pA, a.mean, ffErrFitPercent, noise, band, a.settled ? "" : " (not settled)"));

        if (Math.abs(ffErrFitPercent) < FF_TRIM_SKIP_PERCENT) {
            addLog("kV trim not needed");
            return;
        }

// One Newton-style step using the fitted slope (dPower/dVelocity = kV), then verify by
// measuring. Only kept if the measured error actually improves.
        status = "kV trim";
        double pB = clamp01(pA + (testTarget - a.mean) * kVFit);
        Hold b = measureOpenLoop(pB, SETTLE_TIMEOUT_MS * 2, NOISE_MS / 2, null);
        ffErrTrimPercent = 100.0 * (b.mean - testTarget) / testTarget;
        double kVTrim = (pB - kSUsed) / testTarget;
        if (Math.abs(ffErrTrimPercent) < Math.abs(ffErrFitPercent) && kVTrim > 0) {
            kVUsed = kVTrim;
            kVTrimmed = true;
            addLog(String.format(Locale.US, "kV trimmed %.8f -> %.8f (error %+.2f%% -> %+.2f%%)",
                    kVFit, kVUsed, ffErrFitPercent, ffErrTrimPercent));
        } else {
            addLog(String.format(Locale.US, "kV trim did not help (%+.2f%%), keeping fitted kV",
                    ffErrTrimPercent));
        }
    }

// ================================================================
// PHASE 2B/C: DROP TESTS
// ================================================================

    /** Width of the zone where the 0.01 CachedMotor step hides the error from the controller. */
    private double blindZone(double kP) {
        return CACHE_POWER_STEP / kP;
    }

    private double costOf(Candidate c) {
        return c.recoveryMs + W_OVERSHOOT * c.overshoot + W_HOLD * c.holdStd
                + W_OFFSET * c.holdMeanErr + W_CHATTER * c.powerStd;
    }

    private Candidate evaluate(double kP, double useRaw) {
        Candidate c = new Candidate();
        c.kP = kP;
        c.useRaw = useRaw;
        double blind = blindZone(kP);
        if (blind > Math.max(0.05, MAX_RES_BAND_FRACTION) * band) {
            c.rejected = true;
            c.reason = String.format(Locale.US, "0.01 power step hides errors < %.1f ticks/s (band %.0f)",
                    blind, band);
        } else {
            c = measure(kP, useRaw);
        }
        logCandidate("", c);
        return c;
    }

    private void logCandidate(String prefix, Candidate c) {
        if (c.rejected) {
            addLog(String.format(Locale.US, "%skP %.5f uR %.0f REJECT: %s", prefix, c.kP, c.useRaw, c.reason));
        } else {
            addLog(String.format(Locale.US,
                    "%skP %.5f uR %.0f rec %.0fms ov %.0f hold %.1f off %.1f pw %.3f blind %.1f cost %.0f (n=%d)",
                    prefix, c.kP, c.useRaw, c.recoveryMs, c.overshoot, c.holdStd, c.holdMeanErr, c.powerStd,
                    blindZone(c.kP), c.cost, c.n));
        }
    }

    /** Runs DROPS_PER_CANDIDATE simulated drops and averages them. */
    private Candidate measure(double kP, double useRaw) {
        Candidate c = new Candidate();
        c.kP = kP;
        c.useRaw = useRaw;

        double sRec = 0, sOver = 0, sHoldStd = 0, sHoldErr = 0, sPow = 0, sDip = 0;
        int done = 0;
        for (int d = 0; d < Math.max(1, DROPS_PER_CANDIDATE); d++) {
            Drop r = runDrop(kP, useRaw, testTarget);
            c.crossings = Math.max(c.crossings, r.crossings);
            if (!r.ok) {
                c.rejected = true;
                c.reason = r.reason;
                break; // no point spending time on a candidate that already failed
            }
            sRec += r.recoveryMs;
            sOver += r.overshoot;
            sHoldStd += r.holdStd;
            sHoldErr += Math.abs(r.holdMeanErr); // abs per drop so +/- offsets do not cancel
            sPow += r.powerStd;
            sDip += r.dipDepth;
            done++;
        }

        if (!c.rejected && done > 0) {
            c.n = done;
            c.recoveryMs = sRec / done;
            c.overshoot = sOver / done;
            c.holdStd = sHoldStd / done;
            c.holdMeanErr = sHoldErr / done;
            c.powerStd = sPow / done;
            c.dipDepth = sDip / done;
            c.cost = costOf(c);
            if (c.dipDepth < 0.5 * DIP_TICKS) {
                shortDipCount++;
            }
        }
        sampleVoltage();
        return c;
    }

    /** Folds extra measurements b into a (weighted by drop count). A failure in b rejects a. */
    private void merge(Candidate a, Candidate b) {
        a.crossings = Math.max(a.crossings, b.crossings);
        if (b.rejected) {
            a.rejected = true;
            a.reason = "failed on re-test: " + b.reason;
            return;
        }
        if (a.rejected || b.n == 0) {
            return;
        }
        double wa = a.n, wb = b.n, w = wa + wb;
        a.recoveryMs = (a.recoveryMs * wa + b.recoveryMs * wb) / w;
        a.overshoot = (a.overshoot * wa + b.overshoot * wb) / w;
        a.holdStd = (a.holdStd * wa + b.holdStd * wb) / w;
        a.holdMeanErr = (a.holdMeanErr * wa + b.holdMeanErr * wb) / w;
        a.powerStd = (a.powerStd * wa + b.powerStd * wb) / w;
        a.dipDepth = (a.dipDepth * wa + b.dipDepth * wb) / w;
        a.n += b.n;
        a.cost = costOf(a);
    }

    /**
     * Two noisy drops cannot separate candidates that are within about 10% of each other, so the
     * best few get more drops before the "smallest within KEEP_WITHIN" pick is made.
     */
    private void retestTop(List<Candidate> list) {
        int b = bestIndex(list);
        if (b < 0 || RETEST_TOP <= 0) {
            return;
        }
        double limit = list.get(b).cost * (1 + RETEST_WITHIN);
        List<Candidate> top = new ArrayList<>();
        for (Candidate c : list) {
            if (!c.rejected && c.cost <= limit && !top.contains(c)) {
                top.add(c);
            }
        }
        Collections.sort(top, new Comparator<Candidate>() {
            @Override
            public int compare(Candidate x, Candidate y) {
                return Double.compare(x.cost, y.cost);
            }
        });
        for (int i = 0; i < Math.min(top.size(), RETEST_TOP); i++) {
            Candidate c = top.get(i);
            status = String.format(Locale.US, "re-test %d/%d kP %.5f uR %.0f", i + 1,
                    Math.min(top.size(), RETEST_TOP), c.kP, c.useRaw);
            merge(c, measure(c.kP, c.useRaw));
            logCandidate("retest ", c);
        }
    }

    private void buildCheckTargets() {
        double hi = 0.9 * maxVelocity;
        lowTarget = testTarget;
        for (double t : parseCsv(CHECK_TARGETS)) {
            boolean usable = t >= 3 * MIN_FIT_VELOCITY && t <= hi
                    && Math.abs(t - testTarget) >= 0.05 * testTarget;
            if (usable && checkTargets.size() < 3) {
                checkTargets.add(t);
                lowTarget = Math.min(lowTarget, t);
            }
        }
    }

    /** One drop at each check target with the final gains; reported, not used for selection. */
    private void runChecks() {
        for (double t : checkTargets) {
            status = String.format(Locale.US, "check @%.0f", t);
            Drop r = runDrop(tunedKP, tunedUseRaw, t);
            String line;
            if (r.ok) {
                line = String.format(Locale.US,
                        "Check @%.0f: recover %.0f ms, overshoot %.0f, hold std %.1f, offset %.1f, dip %.0f",
                        t, r.recoveryMs, r.overshoot, r.holdStd, Math.abs(r.holdMeanErr), r.dipDepth);
            } else {
                line = String.format(Locale.US, "Check @%.0f: FAILED (%s) - gains may not hold across speeds",
                        t, r.reason);
            }
            checkLines.add(line);
            addLog(line);
        }
    }

    /** One simulated ball drop at the given gains. */
    private Drop runDrop(double kP, double useRaw, double target) {
        Drop out = new Drop();
        graphTarget = target;
        driveClosedLoop(kP, useRaw, target);

// 1. Get to steady state under these gains.
        status = String.format(Locale.US, "kP %.5f uR %.0f: settling", kP, useRaw);
        if (!waitInBand(BAND_SETTLE_TIMEOUT_MS)) {
            out.reason = "never settled in band";
            return out;
        }

// 2. Hold quality, measured before the disturbance.
        status = String.format(Locale.US, "kP %.5f uR %.0f: holding", kP, useRaw);
        RunningStats vel = new RunningStats();
        RunningStats pow = new RunningStats();
        double h0 = nowMs;
        while (nowMs - h0 < HOLD_MS) {
            tick();
            vel.add(shooter.velocity() - target);
            pow.add(shooter.power());
        }
        out.holdStd = vel.std();
        out.holdMeanErr = vel.mean();
        out.powerStd = pow.std();

// 3. Cut power until velocity has dropped by DIP_TICKS. Power 0 comes from the real
// update() (disabled branch), never from us writing the motor directly.
        status = String.format(Locale.US, "kP %.5f uR %.0f: dropping", kP, useRaw);
        double vBefore = shooter.velocity();
        shooter.disable();
        double d0 = nowMs;
        double vMin = vBefore;
        while (true) {
            tick();
            vMin = Math.min(vMin, shooter.velocity());
            if (shooter.velocity() <= target - DIP_TICKS || nowMs - d0 >= DIP_MAX_MS) {
                break;
            }
        }
        out.dipDepth = target - vMin;

// 4. Let the controller recover, and watch for overshoot and ringing.
        status = String.format(Locale.US, "kP %.5f uR %.0f: recovering", kP, useRaw);
        shooter.enable();
        double t0 = nowMs; // same clock as every other timestamp in this loop
        double vMax = Double.NEGATIVE_INFINITY;
        double stayStart = -1;
        double recoveredEntry = -1;
        double confirmAt = -1;
// Start on whichever side of target the flywheel is on right after the dip.
        int sideState = shooter.velocity() > target + deadband ? 1 : -1;
        int crossings = 0;

        while (true) {
            tick();
            double v = shooter.velocity();
            double err = v - target;
            vMax = Math.max(vMax, v);

// Hysteresis: only count a crossing once error leaves the noise deadband on the far side.
            int newSide = err > deadband ? 1 : (err < -deadband ? -1 : sideState);
            if (newSide != sideState) {
                crossings++;
                sideState = newSide;
            }

            if (Math.abs(err) <= band) {
                if (stayStart < 0) {
                    stayStart = nowMs;
                }
                if (recoveredEntry < 0 && nowMs - stayStart >= RECOVERY_STAY_MS) {
                    recoveredEntry = stayStart;
                    confirmAt = nowMs;
                }
            } else {
                stayStart = -1;
            }

            if (recoveredEntry >= 0 && nowMs - confirmAt >= POST_RECOVERY_MS) {
                break;
            }
            if (recoveredEntry < 0 && nowMs - t0 >= RECOVERY_TIMEOUT_MS) {
                break;
            }
        }

        out.crossings = crossings;
        if (recoveredEntry < 0) {
            out.reason = "no recovery in " + fmt(RECOVERY_TIMEOUT_MS, 0) + " ms";
            return out;
        }
        if (crossings >= OSC_CROSSINGS_MAX) {
            out.reason = "oscillation (" + crossings + " crossings)";
            return out;
        }
        out.recoveryMs = recoveredEntry - t0;
// Overshoot beyond the noise deadband; otherwise every candidate gets a free ~3 sigma penalty.
        out.overshoot = Math.max(0, vMax - target - deadband);
        out.ok = true;
        return out;
    }

    /** Waits until the velocity has stayed inside the band for SETTLE_BAND_STAY_MS. */
    private boolean waitInBand(double timeoutMs) {
        double t0 = nowMs;
        double stayStart = -1;
        while (nowMs - t0 < timeoutMs) {
            tick();
            double err = shooter.velocity() - shooter.target();
            if (Math.abs(err) <= band) {
                if (stayStart < 0) {
                    stayStart = nowMs;
                }
                if (nowMs - stayStart >= SETTLE_BAND_STAY_MS) {
                    return true;
                }
            } else {
                stayStart = -1;
            }
        }
        return false;
    }

    private int bestIndex(List<Candidate> list) {
        int best = -1;
        for (int i = 0; i < list.size(); i++) {
            Candidate c = list.get(i);
            if (!c.rejected && (best < 0 || c.cost < list.get(best).cost)) {
                best = i;
            }
        }
        return best;
    }

    /**
     * Among non-rejected candidates within KEEP_WITHIN of the best cost, returns the one with the
     * smallest kP (preferSmallKp) or the smallest useRaw.
     */
    private Candidate pickWithinBest(List<Candidate> list, boolean preferSmallKp) {
        int b = bestIndex(list);
        if (b < 0) {
            return null;
        }
        double limit = list.get(b).cost * (1 + KEEP_WITHIN);
        Candidate pick = list.get(b);
        for (Candidate c : list) {
// Skip rejects, clearly worse costs, and candidates that hold with a visible offset.
            if (c.rejected || c.cost > limit || c.holdMeanErr > deadband) {
                continue;
            }
            if (preferSmallKp ? c.kP < pick.kP : c.useRaw < pick.useRaw) {
                pick = c;
            }
        }
        return pick;
    }

// ================================================================
// PHASE 3: LIVE VERIFICATION WITH REAL BALLS
// ================================================================

    private void liveVerify() {
        screenMode = SCREEN_LIVE;
        double hiLimit = 0.9 * maxVelocity;
        double loLimit = Math.min(500, hiLimit);
        liveTarget = Math.max(loLimit, Math.min(hiLimit, testTarget));
        graphTarget = liveTarget;
        driveTunedClosedLoop(liveTarget);

        double dipThreshold = Math.max(LIVE_DIP_NOISE_MULT * noise, LIVE_DIP_MIN);
        ArrayDeque<double[]> window = new ArrayDeque<>();
        double inBandSince = -1;
        liveState = 0; // 0 settling, 1 armed (holding), 2 shot in progress

        double dipAt = 0, stayStart = -1, recoveredEntry = -1, confirmAt = -1, vMax = 0;

        while (true) {
            tick();

// Manual edge detection on the raw dpad booleans (works on every SDK 10.x build).
            boolean up = gamepad1.dpad_up && !prevUp;
            boolean down = gamepad1.dpad_down && !prevDown;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            if (up || down) {
                liveTarget += up ? LIVE_TARGET_STEP : -LIVE_TARGET_STEP;
                liveTarget = Math.max(loLimit, Math.min(hiLimit, liveTarget));
                graphTarget = liveTarget;
                shooter.target(liveTarget);
                liveState = 0;
                inBandSince = -1;
                window.clear();
                liveHold.clear();
            }

            double v = shooter.velocity();
            double err = v - liveTarget;

            if (liveState == 0) {
// Wait for the flywheel to be steady before watching for dips, so spin-up and
// target changes are not counted as shots.
                if (Math.abs(err) <= band) {
                    if (inBandSince < 0) {
                        inBandSince = nowMs;
                    }
                    if (nowMs - inBandSince >= SETTLE_BAND_STAY_MS) {
                        liveState = 1;
                    }
                } else {
                    inBandSince = -1;
                }
            } else if (liveState == 1) {
                if (err < -dipThreshold) {
                    liveState = 2;
                    dipAt = nowMs;
                    vMax = v;
                    liveVMin = v;
                    stayStart = -1;
                    recoveredEntry = -1;
                } else {
                    window.addLast(new double[]{nowMs, err});
                    liveHold.add(err);
                    while (window.size() > 1 && nowMs - window.peekFirst()[0] > HOLD_MS) {
                        liveHold.remove(window.removeFirst()[1]);
                    }
                }
            } else {
                vMax = Math.max(vMax, v);
                if (recoveredEntry < 0) {
                    liveVMin = Math.min(liveVMin, v); // only the dip itself, not post-recovery noise
                }
                if (Math.abs(err) <= band) {
                    if (stayStart < 0) {
                        stayStart = nowMs;
                    }
                    if (recoveredEntry < 0 && nowMs - stayStart >= RECOVERY_STAY_MS) {
                        recoveredEntry = stayStart;
                        confirmAt = nowMs;
                    }
                } else {
                    stayStart = -1;
                }

                boolean recovered = recoveredEntry >= 0;
                if (recovered && err < -dipThreshold) {
// Next ball fired before the post-recovery window finished.
                    recordShot(recoveredEntry - dipAt, Math.max(0, vMax - liveTarget - deadband));
                    dipAt = nowMs;
                    vMax = v;
                    liveVMin = v;
                    stayStart = -1;
                    recoveredEntry = -1;
                } else if (recovered && nowMs - confirmAt >= POST_RECOVERY_MS) {
                    recordShot(recoveredEntry - dipAt, Math.max(0, vMax - liveTarget - deadband));
                    liveState = 1;
                    window.clear();
                    liveHold.clear();
                } else if (!recovered && nowMs - dipAt >= 2 * RECOVERY_TIMEOUT_MS) {
                    liveTimeouts++;
                    liveState = 0;
                    inBandSince = -1;
                    window.clear();
                    liveHold.clear();
                }
            }
        }
    }

    /** Recovery is measured from the moment the dip was detected, so it slightly underestimates. */
    private void recordShot(double recoveryMs, double overshoot) {
        if (liveDips.size() < 200) {
            liveDips.add(liveTarget - liveVMin);
        }
        liveShots++;
        liveLastRec = recoveryMs;
        liveSumRec += recoveryMs;
        liveWorstRec = Math.max(liveWorstRec, recoveryMs);
        liveLastOver = overshoot;
    }

// ================================================================
// DRIVING THE REAL SHOOTER
// ================================================================

    /**
     * Makes Shooter.update() output exactly {@code power}: with kP = kS = 0, a huge useRaw and
     * target = 1, the control law reduces to max(0, kV * 1).
     */
    private void driveOpenLoop(double power) {
        Shooter.kP = 0;
        Shooter.kS = 0;
        Shooter.useRaw = 1e9;
        Shooter.kV = power;
        shooter.target(1.0);
        shooter.enable();
    }

    private void driveClosedLoop(double kP, double useRaw, double target) {
        Shooter.kV = kVUsed;
        Shooter.kS = kSUsed;
        Shooter.kP = kP;
        Shooter.useRaw = useRaw;
        shooter.target(target);
        shooter.enable();
    }

    private void driveTunedClosedLoop(double target) {
        driveClosedLoop(tunedKP, tunedUseRaw, target);
    }

    private void motorOff() {
// Never call tick() here: it throws StopRequested once Stop is pressed.
        try {
            shooter.disable();
            shooter.update();
        } catch (RuntimeException ignored) {
// Fall through to the direct write below.
        }
        try {
// CachedMotor skips tiny power changes; write a hard zero to the device as well.
            rawMotorForSafety.setPower(0);
        } catch (RuntimeException ignored) {
// Hardware may already be gone during shutdown; nothing more we can do.
        }
    }

    private void restoreOriginals() {
        Shooter.kS = origKS;
        Shooter.kV = origKV;
        Shooter.kP = origKP;
        Shooter.useRaw = origUseRaw;
        Shooter.targetThreshold = origThreshold;
    }

// ================================================================
// MEASUREMENT HELPERS
// ================================================================

    /** Open-loop at the given power: wait until settled, then collect collectMs of samples. */
    private Hold measureOpenLoop(double power, double settleTimeoutMs, double collectMs,
                                 List<double[]> riseOut) {
        driveOpenLoop(power);
        double t0 = clock.milliseconds();
        ArrayDeque<double[]> window = new ArrayDeque<>();
        Hold h = new Hold();

        while (true) {
            tick();
            double v = shooter.velocity();
            if (riseOut != null) {
                riseOut.add(new double[]{nowMs - t0, v});
            }
            window.addLast(new double[]{nowMs, v});
            while (window.size() > 1 && nowMs - window.peekFirst()[0] > SETTLE_WINDOW_MS) {
                window.removeFirst();
            }
            double elapsed = nowMs - t0;
            if (elapsed >= SETTLE_MIN_MS && nowMs - window.peekFirst()[0] >= 0.9 * SETTLE_WINDOW_MS) {
                double drift = Math.abs(slopePerMs(window)) * SETTLE_WINDOW_MS;
                if (drift <= Math.max(SETTLE_DRIFT_TICKS, 0.01 * Math.abs(v))) {
                    h.settled = true;
                    break;
                }
            }
            if (elapsed >= settleTimeoutMs) {
                break;
            }
        }
        h.settleMs = nowMs - t0;

        List<double[]> pts = new ArrayList<>();
        double c0 = nowMs;
        while (nowMs - c0 < collectMs) {
            tick();
            pts.add(new double[]{nowMs, shooter.velocity()});
        }

        int n = pts.size();
        double sumT = 0, sumV = 0;
        for (double[] p : pts) {
            sumT += p[0];
            sumV += p[1];
        }
        h.mean = n > 0 ? sumV / n : shooter.velocity();
        if (n >= 3) {
            double slope = slopePerMs(pts);
            double meanT = sumT / n;
            double ss = 0;
            for (double[] p : pts) {
                double resid = p[1] - (h.mean + slope * (p[0] - meanT));
                ss += resid * resid;
            }
            h.std = Math.sqrt(ss / (n - 1));
        }
        return h;
    }

    /** Least-squares slope of value over time, in value per millisecond. Points are {t, value}. */
    private static double slopePerMs(Iterable<double[]> pts) {
        int n = 0;
        double sumT = 0, sumV = 0;
        for (double[] p : pts) {
            sumT += p[0];
            sumV += p[1];
            n++;
        }
        if (n < 2) {
            return 0;
        }
        double meanT = sumT / n;
        double meanV = sumV / n;
        double num = 0, den = 0;
        for (double[] p : pts) {
            num += (p[0] - meanT) * (p[1] - meanV);
            den += (p[0] - meanT) * (p[0] - meanT);
        }
        return den < 1e-9 ? 0 : num / den;
    }

    /** Coast to a stop at 0 power so the first ramp step and the tau measurement start from rest. */
    private void waitUntilStopped() {
        status = "Waiting for flywheel to be stopped";
        driveOpenLoop(0);
        double t0 = clock.milliseconds();
        double stillSince = -1;
        while (true) {
            tick();
            if (Math.abs(shooter.velocity()) < 25) {
                if (stillSince < 0) {
                    stillSince = nowMs;
                }
                if (nowMs - stillSince >= 300) {
                    return;
                }
            } else {
                stillSince = -1;
            }
            if (nowMs - t0 > 15000) {
                addLog("WARNING: flywheel still moving at start; tau may be off");
                return;
            }
        }
    }

// ================================================================
// LOOP, TELEMETRY
// ================================================================

    /**
     * One control loop iteration: clear the bulk cache, run the real Shooter.update(), keep timing,
     * and send telemetry at a fixed rate. Every wait in this class loops over this; there are no
     * sleeps, so the controller under test always runs at full loop speed.
     */
    private void tick() {
        if (isStopRequested()) {
            throw new StopRequested();
        }
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        shooter.update();
        nowMs = clock.milliseconds();

        if (lastTickMs >= 0) {
            double dt = nowMs - lastTickMs;
            loopSumMs += dt;
            loopMaxMs = Math.max(loopMaxMs, dt);
            loopCount++;
        }
        lastTickMs = nowMs;

        if (nowMs - lastTeleMs >= TELEMETRY_PERIOD_MS) {
            lastTeleMs = nowMs;
            render();
        }
    }

    private void render() {
        tele.addLine("=== Shooter AutoTuner ===");
        tele.addData("Status", status);
        if (screenMode == SCREEN_LIVE) {
            addLiveLines();
        } else {
            tele.addData("Elapsed", "%.0f s", (nowMs - tuneStartMs) / 1000.0);
            for (int i = Math.max(0, log.size() - LOG_LINES); i < log.size(); i++) {
                tele.addLine(log.get(i));
            }
        }
// Series for Dashboard graphing.
        tele.addData("velocity", shooter.velocity());
        tele.addData("target", graphTarget);
        tele.addData("power", shooter.power());
        tele.addData("loop ms (avg/max)", "%.2f / %.1f",
                loopCount > 0 ? loopSumMs / loopCount : 0, loopMaxMs);
        tele.update();
    }

    private void addLiveLines() {
        for (String line : resultLines()) {
            tele.addLine(line);
        }
        tele.addLine("--- Live verification (load balls and shoot) ---");
        tele.addLine("Statics are set to these values now and stay until the app restarts, so Shooter Test can use them.");
        tele.addData("Live target", "%.0f ticks/s (dpad up/down = +/-%.0f)", liveTarget, LIVE_TARGET_STEP);
        String state = liveState == 0 ? "settling" : (liveState == 1 ? "armed" : "shot in progress");
        tele.addData("State", state);
        tele.addData("Shots", liveShots);
        if (liveShots > 0) {
            tele.addData("Recovery ms last/avg/worst", "%.0f / %.0f / %.0f",
                    liveLastRec, liveSumRec / liveShots, liveWorstRec);
            tele.addData("Last overshoot", "%.0f ticks/s", liveLastOver);
        }
        if (liveTimeouts > 0) {
            tele.addData("Recovery timeouts", liveTimeouts);
        }
        if (!liveDips.isEmpty()) {
            double med = median(liveDips);
            tele.addData("Real dip depth last/median", "%.0f / %.0f ticks/s",
                    liveDips.get(liveDips.size() - 1), med);
            tele.addLine(String.format(Locale.US, "Suggested DIP_TICKS = %.0f (now %.0f). Dip is measured "
                    + "with the controller running, so it is shallower than a bare ball hit.", med, DIP_TICKS));
            if (med < tunedUseRaw) {
                tele.addLine(String.format(Locale.US, "Real dips (%.0f) are smaller than useRaw (%.0f): real "
                                + "shots recover in the P zone, never bang-bang. Rerun with DIP_TICKS = %.0f.",
                        med, tunedUseRaw, med));
            }
        }
        tele.addData("Hold std dev (last 1 s)", "%.1f ticks/s", liveHold.std());
        tele.addData("Hold mean error", "%.1f ticks/s", liveHold.mean());
    }

    /** The lines to paste into Shooter.java plus the supporting numbers. */
    private List<String> resultLines() {
        List<String> lines = new ArrayList<>();
        lines.add("=== RESULTS (paste into Shooter.java) ===");
        lines.add(String.format(Locale.US, "kV = %.8f", kVUsed));
        lines.add(String.format(Locale.US, "kS = %.4f", kSUsed));
        lines.add(String.format(Locale.US, "kP = %.5f", tunedKP));
        lines.add(String.format(Locale.US, "useRaw = %.0f", tunedUseRaw));
        lines.add(String.format(Locale.US, "kV fit %.8f%s R2 %.4f tau %s ms max %.0f ticks/s",
                kVFit, kVTrimmed ? ", trimmed to " + fmt8(kVUsed) : " (used as is)", fitR2,
                Double.isNaN(tauMs) ? "n/a" : fmt(tauMs, 0), maxVelocity));
        lines.add(String.format(Locale.US, "Open-loop FF error at %.0f: fit %+.2f%%%s", testTarget,
                ffErrFitPercent, Double.isNaN(ffErrTrimPercent) ? ""
                        : String.format(Locale.US, ", after trim %+.2f%%", ffErrTrimPercent)));
        lines.add(String.format(Locale.US, "Sensor noise %.1f ticks/s, band +/-%.0f", noise, band));
        double ffLowNow = clamp01(kVUsed * lowTarget + kSUsed);
        lines.add(String.format(Locale.US,
                "Power step 0.01: blind zone +/-%.1f ticks/s. Power jump at useRaw edge %.2f at %.0f "
                        + "ticks/s (0 = continuous)",
                blindZone(tunedKP), Math.max(0, 1 - (ffLowNow + tunedKP * tunedUseRaw)), lowTarget));
        if (finalCandidate != null) {
            lines.add(String.format(Locale.US,
                    "Sim drop (-%.0f): recover %.0f ms, overshoot %.0f, hold std %.1f, offset %.1f, "
                            + "power std %.3f",
                    DIP_TICKS, finalCandidate.recoveryMs, finalCandidate.overshoot,
                    finalCandidate.holdStd, finalCandidate.holdMeanErr, finalCandidate.powerStd));
        }
        lines.addAll(checkLines);
        if (shortDipCount > 0) {
            lines.add("Note: " + shortDipCount + " candidate(s) coasted down less than half of "
                    + "DIP_TICKS before the time cutoff; their recovery numbers are optimistic.");
        }
        if (maxVoltage >= minVoltage) {
            lines.add(String.format(Locale.US,
                    "Battery during test: min %.2f V (max %.2f V). kV was measured at this voltage; "
                            + "it drifts as the battery sags.", minVoltage, maxVoltage));
        }
        return lines;
    }

    private void sampleVoltage() {
        if (nowMs - lastVoltageMs < 500) {
            return;
        }
        lastVoltageMs = nowMs;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            minVoltage = Math.min(minVoltage, v);
            maxVoltage = Math.max(maxVoltage, v);
        }
    }

    private void addLog(String line) {
        log.add(line);
    }

// ================================================================
// SMALL UTILITIES
// ================================================================

    private static double clamp01(double x) {
        return Math.max(0, Math.min(1, x));
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int n = sorted.size();
        return n == 0 ? 0 : (n % 2 == 1 ? sorted.get(n / 2) : 0.5 * (sorted.get(n / 2 - 1) + sorted.get(n / 2)));
    }

    private static String fmt(double x, int decimals) {
        return String.format(Locale.US, "%." + decimals + "f", x);
    }

    private static String fmt8(double x) {
        return String.format(Locale.US, "%.8f", x);
    }

    private static List<Double> parseCsv(String csv) {
        List<Double> out = new ArrayList<>();
        if (csv == null) {
            return out;
        }
        for (String part : csv.split(",")) {
            String s = part.trim();
            if (s.isEmpty()) {
                continue;
            }
            try {
                out.add(Double.parseDouble(s));
            } catch (NumberFormatException ignored) {
// skip malformed entries rather than killing a tuning run
            }
        }
        return out;
    }
}
