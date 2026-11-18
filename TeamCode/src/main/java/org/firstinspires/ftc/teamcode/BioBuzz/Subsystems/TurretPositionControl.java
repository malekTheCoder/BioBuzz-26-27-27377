package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

// Shared by the tuned encoder test and the real turret subsystem.
public final class TurretPositionControl {
    private final AnalogInput encoder;
    private double lastRaw, encoderDegrees, angle, lastAngle, speed;
    private long lastNanos;
    private boolean settled, tripped;
    private double boost;
    private Double guardTarget;
    private double startError;
    private long startTime;
    private double lastTarget = Double.NaN, targetRate;
    private long lastTargetNanos;
    private boolean lastContinuous;
    private double trackingIntegral;
    private boolean excessiveLag, overshot;

    public TurretPositionControl(AnalogInput encoder) {
        this.encoder = encoder;
        zeroForward();
    }

    private double rawAngle() {
        double volts = encoder.getVoltage();
        double fullTurn = encoder.getMaxVoltage(); // same voltage scale as today's test
        if (!Double.isFinite(volts) || !Double.isFinite(fullTurn) || fullTurn <= 0
                || volts < 0 || volts > fullTurn) return Double.NaN;
        return volts / fullTurn * 360.0;
    }

    public void zeroForward() {
        lastRaw = rawAngle();
        encoderDegrees = angle = lastAngle = speed = 0;
        lastNanos = System.nanoTime();
        stop();
    }

    public void update() {
        double raw = rawAngle();
        long now = System.nanoTime();
        if (!Double.isFinite(raw) || !Double.isFinite(lastRaw)
                || !Double.isFinite(Turret.GEAR_RATIO) || Turret.GEAR_RATIO <= 0) {
            angle = Double.NaN;
            return; // re-zero after fixing invalid feedback
        }
        // Unwrap the ENCODER first. Dividing a wrapped angle loses whole turns.
        encoderDegrees += Turret.normalizeDegrees(raw - lastRaw);
        lastRaw = raw;
        double signedDegrees = Turret.LEFT_ENCODER_REVERSED ? -encoderDegrees : encoderDegrees;
        angle = signedDegrees / Turret.GEAR_RATIO;
        double dt = (now - lastNanos) / 1e9;
        if (dt > 0) speed += Turret.VEL_FILTER * ((angle - lastAngle) / dt - speed);
        lastAngle = angle;
        lastNanos = now;
    }

    public double powerTo(double target) {
        return powerTo(target, false);
    }

    public double powerTo(double target, boolean continuous) {
        return powerTo(target, continuous, Double.NaN, Double.NaN);
    }

    // Separate stationary stop/restart thresholds keep a quiet band around the aim.
    public double powerTo(double target, boolean continuous,
                          double positionHoldTolerance, double positionRestartTolerance) {
        boolean preciseHold = Double.isFinite(positionHoldTolerance) && positionHoldTolerance > 0
                && Double.isFinite(positionRestartTolerance) && positionRestartTolerance >= positionHoldTolerance;
        double stopTolerance = preciseHold ? positionHoldTolerance : Turret.READY_ANGLE_TOLERANCE_DEGREES;
        double restartTolerance = preciseHold ? positionRestartTolerance : Turret.RESTART_DEG;
        if (!Double.isFinite(angle) || !Double.isFinite(target)) {
            stop();
            return 0;
        }
        if (tripped) return 0; // preserve the first fault until an explicit reset
        long now = System.nanoTime();
        double targetDt = (now - lastTargetNanos) / 1e9;
        boolean modeChanged = continuous != lastContinuous;
        boolean targetStep = Double.isFinite(lastTarget)
                && Math.abs(target - lastTarget) > Turret.RESTART_DEG;
        lastContinuous = continuous;
        if (!continuous || modeChanged || targetStep) trackingIntegral = 0;
        if (continuous && Double.isFinite(lastTarget) && targetDt > 0 && targetDt < 0.25
                && Math.abs(target - lastTarget) <= Turret.RESTART_DEG) {
            double rate = (target - lastTarget) / targetDt;
            targetRate += Turret.VEL_FILTER * (rate - targetRate);
        } else targetRate = 0; // a new pose is a position step, not an enormous target velocity
        lastTarget = continuous ? target : Double.NaN;
        lastTargetNanos = now;
        double error = target - angle;
        // Tiny odometry target changes must not reset the settled deadband every loop.
        if (guardTarget == null || modeChanged || targetStep
                || (!continuous && Math.abs(guardTarget - target) > Turret.RESTART_DEG)) {
            guardTarget = target;
            startError = Math.abs(error);
            startTime = System.currentTimeMillis();
            settled = false;
            boost = 0;
        }
        if (System.currentTimeMillis() - startTime > Turret.GUARD_GRACE_MS) {
            // d(error)/dt = target speed - turret speed. A negative product
            // means the absolute gap is already shrinking, even after an overshoot.
            boolean recovering = continuous && error * (targetRate - speed) < 0;
            if (continuous && Math.abs(error) > Turret.CONTINUOUS_MAX_LAG_DEG) {
                excessiveLag = tripped = true;
            } else if (Math.abs(error) > startError + Turret.GUARD_GROWTH_DEG
                    && !recovering) tripped = true;
            if (continuous && !tripped) {
                // Compare recent growth; a steady following offset is not a runaway.
                startError = Math.abs(error);
                startTime = System.currentTimeMillis();
            }
        }
        if (tripped) {
            // The same growing gap can mean running ahead, not just falling behind.
            overshot = continuous && Math.abs(targetRate) > 0.5 && error * targetRate < 0;
            return 0; // B/stop or re-zero resets the guard
        }

        if (continuous) settled = false; // follow small target changes without 3-degree stop/restart bursts
        else if (Math.abs(error) <= stopTolerance) {
            settled = true;
            boost = 0;
        } else if (Math.abs(error) > restartTolerance) settled = false;
        if (settled) return 0;

        boolean stalled = Math.abs(speed) < Turret.STALL_SPEED_DEG_S
                && (!continuous || Math.abs(error) > Turret.RESTART_DEG);
        if (stalled) {
            boost = Math.min(Turret.BOOST_MAX, boost + Turret.BOOST_STEP);
        } else boost = Math.max(0, boost - 0.01);

        double dir = Math.signum(error);
        if (continuous) {
            // Target-velocity feedforward keeps driving as the aim moves through the old deadband.
            double power = Turret.KP * error + Turret.continuousFeedforward(targetRate)
                    + Turret.CONTINUOUS_KD * (targetRate - speed);
            if (targetDt > 0 && targetDt < 0.25
                    && Math.abs(error) > Turret.READY_ANGLE_TOLERANCE_DEGREES
                    && error * trackingIntegral < 0) {
                // After crossing the aim, shed stored effort that still drives the old way.
                trackingIntegral *= Math.exp(-targetDt / 0.15);
            }
            double total = power + trackingIntegral;
            if (!modeChanged && !targetStep && targetDt > 0 && targetDt < 0.25
                    && !(total >= Turret.MAX_POWER && error > 0)
                    && !(total <= -Turret.MAX_POWER && error < 0)) {
                // Learn the extra effort needed to follow, without winding up at the power cap.
                double limit = Math.max(0, Turret.CONTINUOUS_MAX_I_POWER);
                trackingIntegral = Range.clip(trackingIntegral
                        + Turret.CONTINUOUS_KI * error * targetDt, -limit, limit);
            }
            power += trackingIntegral;
            if (stalled && dir * power > 0) {
                power = dir * Math.max(dir * power, Turret.MIN_POWER + boost);
            }
            if (Math.abs(targetRate) < 0.5 && Math.abs(error) < 0.35) power = 0;
            return Range.clip(power, -Turret.MAX_POWER, Turret.MAX_POWER);
        }
        double p = Turret.KP * error;
        boolean brakingZone = Math.abs(error) < Turret.BRAKE_ZONE_DEG;
        if (brakingZone) p -= Turret.KD * speed;
        // While already moving toward the goal, allow braking to coast at zero.
        // Keep the minimum + boost when slow/stalled so it can still reach the target.
        boolean movingTowardGoal = dir * speed > Turret.STALL_SPEED_DEG_S;
        double magnitude = brakingZone && movingTowardGoal
                ? Math.max(0, dir * p) : Math.max(dir * p, Turret.MIN_POWER + boost);
        return Range.clip(dir * magnitude, -Turret.MAX_POWER, Turret.MAX_POWER);
    }

    public void stop() {
        guardTarget = null;
        settled = tripped = false;
        boost = 0;
        lastTarget = Double.NaN;
        targetRate = 0;
        lastTargetNanos = 0;
        lastContinuous = false;
        trackingIntegral = 0;
        excessiveLag = overshot = false;
    }
    public double getAngle() { return angle; }
    public double getEncoderRevs() {
        return (Turret.LEFT_ENCODER_REVERSED ? -encoderDegrees : encoderDegrees) / 360.0;
    }
    public double getSpeed() { return speed; }
    public double getTargetSpeed() { return targetRate; }
    public double getBoost() { return boost; }
    public boolean isTripped() { return tripped; }
    public String getFaultReason() {
        return overshot ? "OVERSHOT TARGET" : excessiveLag ? "TRACKING TOO SLOW" : "ERROR GREW";
    }
}
