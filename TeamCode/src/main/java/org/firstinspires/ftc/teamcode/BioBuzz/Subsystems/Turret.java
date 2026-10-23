package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Supplier;

@Config
public class Turret extends Subsystem<Turret.TurretStates> {
    public enum TurretStates { IDLE, ODOM_TRACKING, MANUAL }

    public static final String LEFT_SERVO_NAME = "leftTurretServo";
    public static final String LEFT_ENCODER_NAME = "leftTurretEncoder";
    // Right turret is still being built: no right hardware is initialized or commanded.
    // public static final String RIGHT_SERVO_NAME = "rightTurretServo";
    // public static final String RIGHT_ENCODER_NAME = "rightTurretEncoder";

    // Inches from robot center: +X forward, +Y left.
    public static double LEFT_OFFSET_X = -7.125, LEFT_OFFSET_Y = -0.125;
    public static boolean LEFT_ENCODER_REVERSED = false;
    public static boolean LEFT_SERVO_REVERSED = true;
    // Mirror the field aim into this turret's existing encoder/servo angle direction.
    public static boolean LEFT_AIM_REVERSED = true;
    // Encoder revolutions per turret revolution, measured in Encoder Revs Counter.
    public static double GEAR_RATIO = 4.782;

    // Your tuned encoder test values, shared with the real subsystem.
    public static double KP = 0.02, KD = 0.0005;
    public static double CONTINUOUS_KD = 0.0025; // damp speed relative to a moving target
    public static double CONTINUOUS_KV = 0.002; // power per degree/second of moving target
    // A negative directional kV uses the original shared kV until calibration is saved.
    public static double CONTINUOUS_KS_FORWARD = 0, CONTINUOUS_KV_FORWARD = -1;
    public static double CONTINUOUS_KS_REVERSE = 0, CONTINUOUS_KV_REVERSE = -1;
    public static double CONTINUOUS_KI = 0.0, CONTINUOUS_MAX_I_POWER = 0.20;
    public static double CONTINUOUS_MAX_LAG_DEG = 45.0;
    public static double BRAKE_ZONE_DEG = 20.0;
    public static double MIN_POWER = 0.10, MAX_POWER = 0.5, MANUAL_MAX_POWER = 0.25;
    public static double READY_ANGLE_TOLERANCE_DEGREES = 1.5, RESTART_DEG = 3.0;
    public static double VEL_FILTER = 0.3;
    public static double BOOST_STEP = 0.003, BOOST_MAX = 0.15, STALL_SPEED_DEG_S = 3.0;
    public static long GUARD_GRACE_MS = 700;
    public static double GUARD_GROWTH_DEG = 15.0;
    public static double SERVO_SETTLE_TIME_SECONDS = 0.20;

    // Keep these until the safe mechanism/cable travel has been measured.
    public static double LEFT_MAX_CLOCKWISE_DEGREES = 30.0;
    public static double LEFT_MAX_COUNTERCLOCKWISE_DEGREES = 30.0;

    private final CRServo leftServo;
    private final AnalogInput leftEncoder;
    private final TurretPositionControl control;
    private final Supplier<Pose> robotPose;
    private TurretStates state = TurretStates.IDLE;
    private Pose goal;
    private double leftTargetAngle, leftCommandedAngle, leftManualPower = Double.NaN, leftPower;
    private boolean continuousTracking;
    private double positionHoldTolerance = Double.NaN;
    private double positionRestartTolerance = Double.NaN;
    private boolean fullRotation, hasTrackingRoute;
    private boolean leftTargetReachable;
    private double clockwiseTravelLimit = Double.NaN, counterclockwiseTravelLimit = Double.NaN;
    private double powerRampPerSecond = Double.POSITIVE_INFINITY;
    private double powerBrakeRampPerSecond = Double.POSITIVE_INFINITY;
    private long lastPowerNanos;
    private long atTargetSinceNanos;

    public Turret(HardwareMap hardwareMap, Follower drivetrain) {
        this(hardwareMap, drivetrain::getPose);
    }

    // A bench test can supply a pretend pose without initializing drive hardware.
    public Turret(HardwareMap hardwareMap, Supplier<Pose> robotPose) {
        this.robotPose = robotPose;
        leftServo = hardwareMap.get(CRServo.class, LEFT_SERVO_NAME);
        leftEncoder = hardwareMap.get(AnalogInput.class, LEFT_ENCODER_NAME);
        setPower(0);
        control = new TurretPositionControl(leftEncoder);
        setAlliance();
    }

    @Override public void set(TurretStates newState) {
        state = newState;
        atTargetSinceNanos = 0;
        leftTargetReachable = false;
        hasTrackingRoute = false;
        control.stop();
        setPower(0);
    }
    @Override public TurretStates get() { return state; }

    public void setAlliance() { setAlliance(Common.isRed); }
    public void setAlliance(boolean isRed) {
        Common.isRed = isRed;
        goal = Common.getGoalForAlliance(isRed,
                robotPose.get().getY() >= Common.FIELD_MIDLINE_Y);
        atTargetSinceNanos = 0;
    }

    // With a geared multi-turn encoder, forward must be known at startup.
    public void zeroLeftForward() {
        stow();
        control.zeroForward();
        leftTargetAngle = 0;
        leftCommandedAngle = 0;
    }

    public double getDistance() {
        Pose p = robotPose.get();
        Pose currentGoal = Common.getGoalForAlliance(Common.isRed,
                p.getY() >= Common.FIELD_MIDLINE_Y);
        return Math.hypot(currentGoal.getX() - p.getX(), currentGoal.getY() - p.getY());
    }
    public double getLeftTargetAngle() { return leftTargetAngle; }
    public double getLeftCommandedAngle() { return leftCommandedAngle; }
    public double getLeftEncoderVoltage() { return leftEncoder.getVoltage(); }
    public double getLeftAngle() { return control.getAngle(); }
    public double getLeftPower() { return leftPower; }
    public double getLeftSpeed() { return control.getSpeed(); }
    public double getLeftTargetSpeed() { return control.getTargetSpeed(); }
    public double getLeftTrackingError() { return leftCommandedAngle - control.getAngle(); }
    public boolean isLeftTargetReachable() { return leftTargetReachable; }
    public boolean isLeftControlTripped() { return control.isTripped(); }
    public String getLeftFaultReason() { return control.getFaultReason(); }
    public void setContinuousTracking(boolean enabled) { continuousTracking = enabled; }
    // Per-instance stationary precision; NaN restores the existing position deadband.
    public void setPositionHoldTolerance(double stopDegrees, double restartDegrees) {
        boolean valid = Double.isFinite(stopDegrees) && stopDegrees > 0
                && Double.isFinite(restartDegrees) && restartDegrees >= stopDegrees;
        positionHoldTolerance = valid ? stopDegrees : Double.NaN;
        positionRestartTolerance = valid ? restartDegrees : Double.NaN;
    }
    private double readyAngleTolerance() {
        // Accept the quiet band too, so a settled hold cannot block alignment there.
        return Double.isFinite(positionRestartTolerance)
                ? positionRestartTolerance : READY_ANGLE_TOLERANCE_DEGREES;
    }
    // Opt-in for the unmounted bench turret; normal robot travel remains bounded.
    public void setFullRotation(boolean enabled) {
        if (fullRotation != enabled) hasTrackingRoute = false;
        fullRotation = enabled;
    }
    public void setPowerRampPerSecond(double rate) {
        setPowerRampPerSecond(rate, rate);
    }
    public void setPowerRampPerSecond(double rate, double brakeRate) {
        powerRampPerSecond = Double.isFinite(rate) && rate > 0 ? rate : Double.POSITIVE_INFINITY;
        powerBrakeRampPerSecond = Double.isFinite(brakeRate) && brakeRate > 0
                ? Math.max(powerRampPerSecond, brakeRate) : powerRampPerSecond;
    }
    // Per-instance bench travel; ordinary robot tracking keeps the shared defaults.
    public void setTravelLimits(double clockwise, double counterclockwise) {
        clockwiseTravelLimit = Double.isFinite(clockwise) ? Math.max(0, clockwise) : 0;
        counterclockwiseTravelLimit = Double.isFinite(counterclockwise) ? Math.max(0, counterclockwise) : 0;
    }
    private double clockwiseLimit() {
        return Double.isNaN(clockwiseTravelLimit) ? LEFT_MAX_CLOCKWISE_DEGREES : clockwiseTravelLimit;
    }
    private double counterclockwiseLimit() {
        return Double.isNaN(counterclockwiseTravelLimit)
                ? LEFT_MAX_COUNTERCLOCKWISE_DEGREES : counterclockwiseTravelLimit;
    }
    // Compatibility for existing callers only; right hardware is disabled.
    public double getRightTargetAngle() { return 0; }
    public double getRightEncoderVoltage() { return 0; }
    public double getRightAngle() { return 0; }

    public static Pose calculateTurretPosition(Pose p, double forward, double left) {
        double h = p.getHeading();
        return new Pose(p.getX() + forward * Math.cos(h) - left * Math.sin(h),
                p.getY() + forward * Math.sin(h) + left * Math.cos(h), h);
    }
    public double calculateAngleToGoal(Pose p) {
        return Math.toDegrees(Math.atan2(goal.getY() - p.getY(), goal.getX() - p.getX()));
    }
    public static double normalizeDegrees(double angle) {
        double value = (angle + 180.0) % 360.0;
        if (value < 0.0) value += 360.0;
        return value - 180.0;
    }

    public static double continuousFeedforward(double rate) {
        if (!Double.isFinite(rate)) return 0;
        double kv = rate >= 0 ? CONTINUOUS_KV_FORWARD : CONTINUOUS_KV_REVERSE;
        double ks = rate >= 0 ? CONTINUOUS_KS_FORWARD : CONTINUOUS_KS_REVERSE;
        if (!Double.isFinite(kv) || kv < 0) { kv = CONTINUOUS_KV; ks = 0; }
        if (!Double.isFinite(ks) || ks < 0) ks = 0;
        // Fade static effort near zero target speed so small jitter cannot flip full kS.
        return kv * rate + ks * Range.clip(rate / 5.0, -1, 1);
    }

    public void setManualPower(double left, double right) {
        state = TurretStates.MANUAL;
        leftManualPower = Range.clip(left, -MANUAL_MAX_POWER, MANUAL_MAX_POWER);
        control.stop();
        atTargetSinceNanos = 0;
        // right is intentionally ignored until that turret is built
    }
    public void setManualAngles(double left, double right) {
        state = TurretStates.MANUAL;
        leftTargetAngle = left; // actual turret degrees: keep 180/270 instead of wrapping them
        leftManualPower = Double.NaN;
        atTargetSinceNanos = 0;
    }
    public void resumeTracking() { set(TurretStates.ODOM_TRACKING); }
    public void stow() { set(TurretStates.IDLE); }

    @Override public void run() {
        control.update(); // count encoder wraps even while stopped
        if (state == TurretStates.IDLE) { setPower(0); return; }
        double angle = control.getAngle();
        if (!Double.isFinite(angle)) { stopInvalid(); return; }

        if (state == TurretStates.MANUAL && !Double.isNaN(leftManualPower)) {
            setPower(limitManual(leftManualPower, angle));
            return;
        }
        if (state == TurretStates.ODOM_TRACKING) updateTracking();
        if (!Double.isFinite(leftTargetAngle)) { stopInvalid(); return; }

        double target = leftTargetAngle;
        double clockwise = clockwiseLimit(), counterclockwise = counterclockwiseLimit();
        if (state == TurretStates.ODOM_TRACKING) {
            if (fullRotation) {
                // Unwrap against the previous command, so crossing rear/zero cannot reverse the route.
                double reference = hasTrackingRoute ? leftCommandedAngle : angle;
                target = reference + normalizeDegrees(target - reference);
                hasTrackingRoute = true;
            } else target = chooseRoute(target, angle, clockwise, counterclockwise);
        } else if (!fullRotation && (target < -clockwise || target > counterclockwise)) target = Double.NaN;
        leftTargetReachable = Double.isFinite(target);
        if (!leftTargetReachable) target = Range.clip(leftTargetAngle,
                -clockwise, counterclockwise);
        leftCommandedAngle = target;
        boolean continuous = continuousTracking && state == TurretStates.ODOM_TRACKING;
        double power = control.powerTo(target, continuous, positionHoldTolerance, positionRestartTolerance);
        boolean smoothZero = continuous && !control.isTripped()
                && (fullRotation || (angle > -clockwise && angle < counterclockwise));
        setPower(limitManual(power, angle), smoothZero);
        if (state == TurretStates.ODOM_TRACKING && leftTargetReachable && !control.isTripped()
                && Math.abs(target - angle) <= readyAngleTolerance()) {
            if (atTargetSinceNanos == 0) atTargetSinceNanos = System.nanoTime();
        } else atTargetSinceNanos = 0;
    }

    private void stopInvalid() {
        leftTargetReachable = false;
        atTargetSinceNanos = 0;
        control.stop();
        setPower(0);
    }
    private void updateTracking() {
        Pose robot = robotPose.get();
        Pose selected = Common.getGoalForAlliance(Common.isRed,
                robot.getY() >= Common.FIELD_MIDLINE_Y);
        if (goal.getX() != selected.getX() || goal.getY() != selected.getY()) atTargetSinceNanos = 0;
        goal = selected;
        double relativeAim = calculateAngleToGoal(calculateTurretPosition(
                robot, LEFT_OFFSET_X, LEFT_OFFSET_Y)) - Math.toDegrees(robot.getHeading());
        leftTargetAngle = normalizeDegrees(LEFT_AIM_REVERSED ? -relativeAim : relativeAim);
    }
    private static double chooseRoute(double target, double current, double clockwise, double counterclockwise) {
        double first = Math.ceil((-clockwise - target) / 360.0);
        double last = Math.floor((counterclockwise - target) / 360.0);
        if (first > last) return Double.NaN;
        double turn = Range.clip(Math.rint((current - target) / 360.0), first, last);
        return target + turn * 360.0;
    }
    private double limitManual(double power, double angle) {
        if (fullRotation) return power;
        if (power < 0 && angle <= -clockwiseLimit()) return 0;
        if (power > 0 && angle >= counterclockwiseLimit()) return 0;
        return power;
    }
    private void setPower(double power) {
        setPower(power, false);
    }
    private void setPower(double power, boolean smoothZero) {
        long now = System.nanoTime();
        double dt = Math.min((now - lastPowerNanos) / 1e9, 0.05);
        lastPowerNanos = now;
        if ((power != 0 || smoothZero) && Double.isFinite(powerRampPerSecond)) {
            // Reducing effort must be able to brake faster than a gentle startup ramps up.
            double rate = Math.abs(power) < Math.abs(leftPower)
                    ? powerBrakeRampPerSecond : powerRampPerSecond;
            double step = rate * Math.max(0, dt);
            double ramped = Range.clip(power, leftPower - step, leftPower + step);
            // Never keep driving the old direction when the controller requests a reversal.
            power = ramped * power < 0 ? 0 : ramped;
        }
        // Explicit stops, invalid feedback, tripped guard, and travel boundaries bypass the ramp.
        leftPower = power;
        leftServo.setPower(LEFT_SERVO_REVERSED ? -power : power);
        // rightServo.setPower(...) disabled while the right turret is being built
    }
    public boolean isReadyToShoot() {
        return state == TurretStates.ODOM_TRACKING && atTargetSinceNanos != 0
                && (System.nanoTime() - atTargetSinceNanos) / 1e9 >= SERVO_SETTLE_TIME_SECONDS;
    }
    @Override public void printTelemetry() {
        if (Common.dashTelemetry == null) return;
        Common.dashTelemetry.addLine("LEFT TURRET ONLY - point forward before Start");
        Common.dashTelemetry.addData("State", state);
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Goal", "(%.1f, %.1f)", goal.getX(), goal.getY());
        Common.dashTelemetry.addData("Distance", "%.1f in", getDistance());
        Common.dashTelemetry.addData("Encoder revs / ratio", "%.3f / %.3f",
                control.getEncoderRevs(), GEAR_RATIO);
        Common.dashTelemetry.addData("Left V / angle / target / power", "%.3f / %.1f / %.1f / %.2f",
                getLeftEncoderVoltage(), getLeftAngle(), leftTargetAngle, leftPower);
        Common.dashTelemetry.addData("Speed / boost", "%.1f / %.3f", control.getSpeed(), control.getBoost());
        Common.dashTelemetry.addData("Growing-error guard tripped", control.isTripped());
        Common.dashTelemetry.addData("Target reachable", leftTargetReachable);
        Common.dashTelemetry.addData("Right turret", "DISABLED - still being built");
        Common.dashTelemetry.addData("Ready to shoot (left only)", isReadyToShoot());
    }
}
