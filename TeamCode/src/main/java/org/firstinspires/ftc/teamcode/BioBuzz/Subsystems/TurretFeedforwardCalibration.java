package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import java.util.Arrays;

/** Timed open-loop speed measurements. Power is normalized servo command, not volts. */
public final class TurretFeedforwardCalibration {
    public enum State { IDLE, RUNNING, DONE, FAILED, STOPPED, CHECKED }
    private enum Phase { COAST, RAMP, SETTLE, SAMPLE }
    private static final double[] POWERS = {0.42, 0.36, 0.30, 0.24, 0.18, 0.12};
    private static final double SAMPLE_SECONDS = 2.4;
    private final double[] speeds = new double[2 * POWERS.length];
    private final Point[] points = new Point[2 * POWERS.length];
    private final double[] binDistance = new double[12], binTime = new double[12];
    private final int[] slowPoints = new int[2], unevenPoints = new int[2];
    private State state = State.IDLE;
    private Phase phase;
    private int step, samples;
    private double phaseStart, lastTime, lastAngle, power, coastStableSince;
    private double stepStartAngle, rampStartPower;
    private boolean quickCheck;
    private String message = "A = start calibration";
    private String lastPointResult = "NONE YET";
    private double lastMeasuredSpeed = Double.NaN;
    private Fit forward, reverse;

    public void start(double now, double angle) {
        Arrays.fill(speeds, Double.NaN);
        Arrays.fill(points, null);
        quickCheck = false;
        Arrays.fill(slowPoints, 0);
        Arrays.fill(unevenPoints, 0);
        lastPointResult = "NONE YET";
        lastMeasuredSpeed = Double.NaN;
        forward = reverse = null;
        step = 0;
        power = 0;
        lastTime = now;
        lastAngle = angle;
        state = State.RUNNING;
        enter(Phase.COAST, now);
        if (!Double.isFinite(now) || !Double.isFinite(angle)) fail("ENCODER ERROR");
    }

    /** Two diagnostic holds, not enough data to fit or save a calibration. */
    public void startQuickCheck(double now, double angle) {
        start(now, angle);
        quickCheck = true;
    }

    private void enter(Phase next, double now) {
        phase = next;
        phaseStart = now;
        coastStableSince = Double.NaN;
        if (next == Phase.RAMP) {
            stepStartAngle = lastAngle;
            rampStartPower = power;
        }
        if (next == Phase.SAMPLE) {
            Arrays.fill(binDistance, 0);
            Arrays.fill(binTime, 0);
            samples = 0;
        }
        message = next == Phase.COAST ? "PAUSING" : next == Phase.RAMP ? "RAMPING"
                : next == Phase.SETTLE ? "SETTLING" : "MEASURING";
    }

    public double update(double now, double angle) {
        if (state != State.RUNNING) return 0;
        double dt = now - lastTime;
        if (!Double.isFinite(now) || !Double.isFinite(angle) || dt < 0) return fail("ENCODER/TIME ERROR");
        if (dt > 0.15) return fail("READING DELAY - A rerun");
        if (dt == 0) return power;
        double speed = (angle - lastAngle) / dt;
        double previousTime = lastTime;
        lastTime = now;
        lastAngle = angle;
        double elapsed = now - phaseStart;
        double command = direction() * POWERS[step % POWERS.length];
        switch (phase) {
            case COAST:
                power = 0;
                if (Math.abs(speed) < 3) {
                    if (!Double.isFinite(coastStableSince)) coastStableSince = now;
                    if (elapsed >= 0.6 && now - coastStableSince >= 0.2) enter(Phase.RAMP, now);
                } else coastStableSince = Double.NaN;
                if (elapsed > 3) return fail("NOT STOPPING - check servo neutral");
                break;
            case RAMP:
                power = rampStartPower + (command - rampStartPower) * Math.min(1, elapsed / 0.6);
                if (elapsed >= 0.6) enter(Phase.SETTLE, now);
                break;
            case SETTLE:
                power = command;
                if (elapsed >= 2.0) enter(Phase.SAMPLE, now);
                break;
            case SAMPLE:
                power = command;
                // Average displacement over twelve 0.2-second bins, rather than noisy
                // per-update velocity. Split intervals that cross a bin boundary.
                for (int bin = 0; bin < binTime.length; bin++) {
                    double begin = phaseStart + bin * 0.2;
                    double overlap = Math.max(0, Math.min(now, begin + 0.2)
                            - Math.max(previousTime, begin));
                    binDistance[bin] += speed * overlap;
                    binTime[bin] += overlap;
                }
                samples++;
                if (elapsed >= SAMPLE_SECONDS) finishPoint(now);
                break;
        }
        return power;
    }

    private void finishPoint(double now) {
        double distance = 0, duration = 0, min = Double.POSITIVE_INFINITY, max = -min;
        double first = 0, second = 0;
        for (int i = 0; i < binTime.length; i++) {
            if (binTime[i] < 0.15) { fail("NOT ENOUGH READINGS - A rerun"); return; }
            double speed = direction() * binDistance[i] / binTime[i];
            min = Math.min(min, speed);
            max = Math.max(max, speed);
            distance += direction() * binDistance[i];
            duration += binTime[i];
            if (i < binTime.length / 2) first += direction() * binDistance[i];
            else second += direction() * binDistance[i];
        }
        double mean = distance / duration;
        double steadyMin = Double.POSITIVE_INFINITY, steadyMax = -steadyMin;
        for (int i = 0; i < binTime.length; i += 3) {
            double windowDistance = 0, windowTime = 0;
            for (int j = i; j < i + 3; j++) {
                windowDistance += direction() * binDistance[j];
                windowTime += binTime[j];
            }
            double windowSpeed = windowDistance / windowTime;
            steadyMin = Math.min(steadyMin, windowSpeed);
            steadyMax = Math.max(steadyMax, windowSpeed);
        }
        double steadySpread = steadyMax - steadyMin;
        lastMeasuredSpeed = Math.abs(mean);
        if (mean < -3) {
            points[step] = new Point(POWERS[step % POWERS.length], direction() * (lastAngle - stepStartAngle),
                    distance, mean, max - min, steadySpread, "WRONG DIRECTION");
            fail("WRONG ENCODER DIRECTION");
            return;
        }
        // Characterize steady average speed, rather than rejecting a constant mean
        // solely because short encoder intervals fluctuate. Keep the half-run drift check.
        boolean steady = samples >= 24 && steadySpread <= Math.max(8, 0.25 * mean)
                && Math.abs((first - second) / (SAMPLE_SECONDS / 2)) <= Math.max(5, 0.15 * mean);
        // A stalled/uneven point is omitted, never used as zero-speed fit evidence.
        int side = direction() > 0 ? 0 : 1;
        if (mean < 3) {
            slowPoints[side]++;
            lastPointResult = "TOO SLOW";
        } else if (!steady) {
            unevenPoints[side]++;
            lastPointResult = "UNEVEN SPEED";
        } else {
            speeds[step] = mean;
            lastPointResult = "ACCEPTED";
        }
        points[step] = new Point(POWERS[step % POWERS.length], direction() * (lastAngle - stepStartAngle),
                distance, mean, max - min, steadySpread, lastPointResult);
        if (quickCheck) {
            power = 0;
            if (direction() > 0) {
                step = POWERS.length;
                enter(Phase.COAST, now);
            } else {
                state = State.CHECKED;
                message = "CHECK DONE - Y switches direction";
            }
            return;
        }
        boolean wasForward = direction() > 0;
        if (++step < speeds.length) {
            if (wasForward != (direction() > 0)) {
                power = 0;
                enter(Phase.COAST, now);
            } else {
                // Keep turning between powers: lower effort may sustain motion even
                // when it cannot overcome starting friction from a standstill.
                enter(Phase.RAMP, now);
            }
        }
        else {
            power = 0;
            forward = fit(Arrays.copyOfRange(speeds, 0, POWERS.length), POWERS);
            reverse = fit(Arrays.copyOfRange(speeds, POWERS.length, speeds.length), POWERS);
            state = forward.valid && reverse.valid ? State.DONE : State.FAILED;
            message = state == State.DONE ? "DONE - X saves calibration"
                    : (!forward.valid ? "FORWARD: " + forward.reason : "REVERSE: " + reverse.reason);
        }
    }

    public double fail(String reason) { state = State.FAILED; message = reason; power = 0; return 0; }
    public void stop() {
        power = 0;
        if (state == State.RUNNING) { state = State.STOPPED; message = "STOPPED - A restarts"; }
    }
    public State getState() { return state; }
    public String getMessage() { return message; }
    public int getStep() { return quickCheck ? (direction() > 0 ? 1 : 2) : Math.min(step + 1, speeds.length); }
    public int getStepCount() { return quickCheck ? 2 : speeds.length; }
    public int direction() { return step < POWERS.length ? 1 : -1; }
    public Fit getForward() { return forward; }
    public Fit getReverse() { return reverse; }
    public String getLastPointResult() { return lastPointResult; }
    public double getLastMeasuredSpeed() { return lastMeasuredSpeed; }
    public int getSlowPoints(boolean forward) { return slowPoints[forward ? 0 : 1]; }
    public int getUnevenPoints(boolean forward) { return unevenPoints[forward ? 0 : 1]; }
    public Point getHighPowerPoint(boolean forward) { return points[forward ? 0 : POWERS.length]; }

    public static final class Point {
        public final double power, stepTravel, measuredTravel, meanSpeed, speedSpread, steadySpread;
        public final String result;
        private Point(double power, double stepTravel, double measuredTravel,
                      double meanSpeed, double speedSpread, double steadySpread, String result) {
            this.power = power; this.stepTravel = stepTravel; this.measuredTravel = measuredTravel;
            this.meanSpeed = meanSpeed; this.speedSpread = speedSpread;
            this.steadySpread = steadySpread; this.result = result;
        }
    }

    public static final class Fit {
        public final boolean valid;
        public final String reason;
        public final int count;
        public final double kS, kV, rSquared;
        private Fit(boolean valid, String reason, int count, double kS, double kV, double rSquared) {
            this.valid = valid; this.reason = reason; this.count = count;
            this.kS = kS; this.kV = kV; this.rSquared = rSquared;
        }
    }

    public static Fit fit(double[] speed, double[] power) {
        if (speed.length != power.length) throw new IllegalArgumentException("mismatched points");
        int n = 0;
        double sx = 0, sy = 0, sxx = 0, sxy = 0, min = Double.POSITIVE_INFINITY, max = -min;
        for (int i = 0; i < speed.length; i++) {
            if (!Double.isFinite(speed[i]) || speed[i] < 3 || !Double.isFinite(power[i])
                    || power[i] <= 0 || power[i] > 0.50) continue;
            n++;
            sx += speed[i]; sy += power[i];
            sxx += speed[i] * speed[i]; sxy += speed[i] * power[i];
            min = Math.min(min, speed[i]); max = Math.max(max, speed[i]);
        }
        if (n < 3) return new Fit(false, "ONLY " + n + " STEADY POINTS", n, 0, 0, 0);
        if (max - min < 10) return new Fit(false, "SPEEDS TOO SIMILAR", n, 0, 0, 0);
        double variance = sxx - sx * sx / n;
        if (variance <= 0) return new Fit(false, "SPEED RANGE TOO SMALL", n, 0, 0, 0);
        double kv = (sxy - sx * sy / n) / variance;
        double ks = (sy - kv * sx) / n;
        // Near-zero intercept noise may be refit through zero, never made negative.
        if (ks >= -0.01 && ks < 0) { ks = 0; kv = sxy / sxx; }
        double squaredError = 0, squaredTotal = 0, worstResidual = 0;
        for (int i = 0; i < speed.length; i++) {
            if (!Double.isFinite(speed[i]) || speed[i] < 3 || !Double.isFinite(power[i])
                    || power[i] <= 0 || power[i] > 0.50) continue;
            double residual = power[i] - ks - kv * speed[i];
            squaredError += residual * residual;
            squaredTotal += (power[i] - sy / n) * (power[i] - sy / n);
            worstResidual = Math.max(worstResidual, Math.abs(residual));
        }
        double r2 = squaredTotal > 0 ? 1 - squaredError / squaredTotal : 0;
        boolean valid = Double.isFinite(kv) && kv > 0 && Double.isFinite(ks)
                && ks >= 0 && ks < 0.50 && r2 >= 0.90 && worstResidual <= 0.025;
        return new Fit(valid, valid ? "OK" : "SPEEDS DO NOT FIT - A rerun", n, ks, kv, r2);
    }
}
