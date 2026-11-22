package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import android.content.Context;
import android.content.SharedPreferences;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Supplier;

@Config
public class Turret extends Subsystem<Turret.TurretStates> {
    public enum TurretStates { IDLE, ODOM_TRACKING, MANUAL }

    public static final int GEAR_SCALE_REVISION = 5;
    private static final float EFFECTIVE_TRAVEL_DEGREES = 102.5f;
    private static final float ENDPOINT_FRACTION = 0.45f;
    private static final double[] PREVIOUS_TRAVEL_DEGREES = {360, 75, 100, 90, 97.5};

    public static final String LEFT_SERVO_NAME = "leftTurretServo";
    public static double LEFT_MAX_CLOCKWISE_DEGREES = 20;
    public static double LEFT_MAX_COUNTERCLOCKWISE_DEGREES = 20;
    public static double SERVO_TRAVEL_DEGREES = 360;
    public static double SERVO_FORWARD_POSITION = 0.5;
    public static boolean LEFT_SERVO_REVERSED = false;
    public static boolean LEFT_AIM_REVERSED = false;
    public static double LEFT_AIM_TRIM_DEGREES = 0;
    public static double LEFT_OFFSET_X = 5, LEFT_OFFSET_Y = -4;
    public static double READY_ANGLE_TOLERANCE_DEGREES = 2;
    public static double SERVO_SETTLE_TIME_SECONDS = 0.35;
    public static double MANUAL_MAX_POWER = 0.25;
    public static double MANUAL_SPEED_DEGREES_PER_SECOND = 60;
    public static double MAX_COMMAND_SPEED_DEGREES_PER_SECOND = 240;
    public static final String CALIBRATION_PREFS = "left_axon_mini_mk2_turret_v2";
    public static void configureAxonPwm(Servo servo) {
        if (!(servo instanceof ServoImplEx))
            throw new IllegalStateException("Configure leftTurretServo as a REV hub servo");
        ((ServoImplEx) servo).setPwmRange(new PwmRange(500, 2500));
    }
    public static boolean hasSavedCalibration(Context context) {
        return context.getSharedPreferences(CALIBRATION_PREFS, Context.MODE_PRIVATE)
                .getBoolean("saved", false);
    }
    public static void loadCalibration(Context context) {
        SharedPreferences p = context.getSharedPreferences(CALIBRATION_PREFS, Context.MODE_PRIVATE);
        if (!p.getBoolean("saved", false)) return;
        double savedTravel = p.getFloat("travel", 0);
        double savedCw = p.getFloat("clockwise", 0);
        double savedCcw = p.getFloat("counterclockwise", 0);
        if (p.getInt("gear_scale_revision", 0) < GEAR_SCALE_REVISION
                && isPreviousScale(savedTravel, savedCw, savedCcw)) {
            float limit = EFFECTIVE_TRAVEL_DEGREES * ENDPOINT_FRACTION;
            p.edit().putFloat("travel", EFFECTIVE_TRAVEL_DEGREES)
                    .putFloat("clockwise", limit)
                    .putFloat("counterclockwise", limit)
                    .putBoolean("gear_scale_calibrated", true)
                    .putInt("gear_scale_revision", GEAR_SCALE_REVISION).apply();
        }
        SERVO_TRAVEL_DEGREES = p.getFloat("travel", 360);
        SERVO_FORWARD_POSITION = p.getFloat("center", 0.5f);
        LEFT_SERVO_REVERSED = p.getBoolean("reversed", false);
        LEFT_MAX_CLOCKWISE_DEGREES = p.getFloat("clockwise", 20);
        LEFT_MAX_COUNTERCLOCKWISE_DEGREES = p.getFloat("counterclockwise", 20);
        LEFT_AIM_REVERSED = p.getBoolean("aim_reversed", false);
        LEFT_AIM_TRIM_DEGREES = p.getFloat("aim_trim", 0);
    }

    private static boolean isPreviousScale(double travel, double clockwise, double counterclockwise) {
        for (double oldTravel : PREVIOUS_TRAVEL_DEGREES) {
            double oldLimit = oldTravel * ENDPOINT_FRACTION;
            if (Math.abs(travel - oldTravel) < 0.01
                    && Math.abs(clockwise - oldLimit) < 0.01
                    && Math.abs(counterclockwise - oldLimit) < 0.01) {
                return true;
            }
        }
        return false;
    }

    @Deprecated public static final String LEFT_ENCODER_NAME = "leftTurretEncoder";
    @Deprecated public static boolean LEFT_ENCODER_REVERSED = false;
    @Deprecated public static double GEAR_RATIO = 4.782, KP = 0.01, KD = 0.0035;
    @Deprecated public static double BRAKE_ZONE_DEG = 20, MIN_POWER = 0.1, MAX_POWER = 0.5;
    @Deprecated public static double RESTART_DEG = 3, VEL_FILTER = 0.3;
    @Deprecated public static double BOOST_STEP = 0.003, BOOST_MAX = 0.03, STALL_SPEED_DEG_S = 3;
    @Deprecated public static long GUARD_GRACE_MS = 700;
    @Deprecated public static double GUARD_GROWTH_DEG = 15;
    @Deprecated public static double CONTINUOUS_KD = 0.0025, CONTINUOUS_KI = 0;
    @Deprecated public static double CONTINUOUS_MAX_I_POWER = 0.2, CONTINUOUS_MAX_LAG_DEG = 45;
    @Deprecated public static double CONTINUOUS_KV = 0.002;
    @Deprecated public static double CONTINUOUS_KS_FORWARD = 0, CONTINUOUS_KV_FORWARD = -1;
    @Deprecated public static double CONTINUOUS_KS_REVERSE = 0, CONTINUOUS_KV_REVERSE = -1;
    @Deprecated public static double continuousFeedforward(double rate) { return 0; }

    private final Servo leftServo;
    private final Supplier<Pose> robotPose;
    private TurretStates state = TurretStates.IDLE;
    private Pose goal;
    private double targetAngle, commandedAngle;
    private double manualSpeed = Double.NaN;
    private double clockwiseLimit = Double.NaN, counterclockwiseLimit = Double.NaN;
    private long lastRunNanos, settledSinceNanos;
    private boolean targetReachable;

    public Turret(HardwareMap hardwareMap, Follower drivetrain) {
        this(hardwareMap, drivetrain::getPose);
    }

    public Turret(HardwareMap hardwareMap, Supplier<Pose> robotPose) {
        this.robotPose = robotPose;
        loadCalibration(hardwareMap.appContext);
        leftServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        configureAxonPwm(leftServo);
        setAlliance();
    }

    @Override public void set(TurretStates newState) {
        state = newState;
        manualSpeed = Double.NaN;
        settledSinceNanos = 0;
        targetReachable = false;
    }
    @Override public TurretStates get() { return state; }
    public void setAlliance() { setAlliance(Common.isRed); }
    public void setAlliance(boolean red) {
        Common.isRed = red;
        Pose pose = robotPose.get();
        goal = Common.getGoalForAlliance(red, pose.getY() >= Common.FIELD_MIDLINE_Y);
        settledSinceNanos = 0;
    }
    public void zeroLeftForward() {
        stow();
        commandedAngle = 0;
        targetAngle = 0;
    }
    public void centerLeftForward() {
        state = TurretStates.MANUAL;
        manualSpeed = Double.NaN;
        commandedAngle = 0;
        targetAngle = 0;
        targetReachable = true;
        settledSinceNanos = 0;
        lastRunNanos = System.nanoTime();
        leftServo.setPosition(Range.clip(SERVO_FORWARD_POSITION, 0, 1));
        ((ServoImplEx) leftServo).setPwmEnable();
        leftServo.setPosition(Range.clip(SERVO_FORWARD_POSITION, 0, 1));
    }
    public void disablePwm() {
        stow();
        ((ServoImplEx) leftServo).setPwmDisable();
    }
    public double getDistance() {
        Pose p = robotPose.get();
        Pose g = Common.getGoalForAlliance(Common.isRed, p.getY() >= Common.FIELD_MIDLINE_Y);
        return Math.hypot(g.getX() - p.getX(), g.getY() - p.getY());
    }
    public double getLeftTargetAngle() { return targetAngle; }
    @Deprecated public double getLeftEncoderVoltage() { return Double.NaN; }
    @Deprecated public double getRightEncoderVoltage() { return Double.NaN; }
    public double getLeftCommandedAngle() { return commandedAngle; }
    public double getLeftAngle() { return commandedAngle; }
    public double getLeftPower() { return leftServo.getPosition(); }
    public double getLeftSpeed() { return 0; }
    public double getLeftTargetSpeed() { return 0; }
    public double getLeftTrackingError() { return targetAngle - commandedAngle; }
    public boolean isLeftTargetReachable() { return targetReachable; }
    public boolean isLeftControlTripped() { return false; }
    public boolean isPausedForDrive() { return false; }
    public String getLeftFaultReason() { return ""; }
    public void setContinuousTracking(boolean enabled) { }
    public void setPositionHoldTolerance(double stop, double restart) { }
    public void setFullRotation(boolean enabled) { }
    public void setPowerRampPerSecond(double rate) { }
    public void setPowerRampPerSecond(double rate, double brakeRate) { }
    public void setTravelLimits(double clockwise, double counterclockwise) {
        clockwiseLimit = Math.max(0, clockwise);
        counterclockwiseLimit = Math.max(0, counterclockwise);
    }
    private double clockwiseLimit() {
        return Double.isNaN(clockwiseLimit) ? LEFT_MAX_CLOCKWISE_DEGREES : Math.min(clockwiseLimit, LEFT_MAX_CLOCKWISE_DEGREES);
    }
    private double counterclockwiseLimit() {
        return Double.isNaN(counterclockwiseLimit) ? LEFT_MAX_COUNTERCLOCKWISE_DEGREES : Math.min(counterclockwiseLimit, LEFT_MAX_COUNTERCLOCKWISE_DEGREES);
    }
    public double getRightTargetAngle() { return 0; }
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
        double value = (angle + 180) % 360;
        if (value < 0) value += 360;
        return value - 180;
    }
    public void setManualPower(double left, double right) {
        state = TurretStates.MANUAL;
        manualSpeed = Range.clip(left / Math.max(0.001, MANUAL_MAX_POWER), -1, 1)
                * MANUAL_SPEED_DEGREES_PER_SECOND;
        settledSinceNanos = 0;
    }
    public void setManualAngles(double left, double right) {
        state = TurretStates.MANUAL;
        manualSpeed = Double.NaN;
        targetAngle = left;
        settledSinceNanos = 0;
    }
    public void resumeTracking() { set(TurretStates.ODOM_TRACKING); }
    public void stow() { set(TurretStates.IDLE); }

    @Override public void run() {
        long now = System.nanoTime();
        double dt = lastRunNanos == 0 ? 0 : Math.min((now - lastRunNanos) / 1e9, 0.1);
        lastRunNanos = now;
        if (state == TurretStates.IDLE) return;
        if (state == TurretStates.ODOM_TRACKING) {
            Pose robot = robotPose.get();
            goal = Common.getGoalForAlliance(Common.isRed, robot.getY() >= Common.FIELD_MIDLINE_Y);
            double relative = calculateAngleToGoal(calculateTurretPosition(
                    robot, LEFT_OFFSET_X, LEFT_OFFSET_Y)) - Math.toDegrees(robot.getHeading());
            targetAngle = normalizeDegrees((LEFT_AIM_REVERSED ? -relative : relative)
                    + LEFT_AIM_TRIM_DEGREES);
        } else if (Double.isFinite(manualSpeed)) {
            targetAngle = commandedAngle + manualSpeed * dt;
        }
        if (!Double.isFinite(targetAngle) || !Double.isFinite(SERVO_TRAVEL_DEGREES)
                || SERVO_TRAVEL_DEGREES <= 0 || !Double.isFinite(SERVO_FORWARD_POSITION)) {
            targetReachable = false;
            settledSinceNanos = 0;
            return;
        }
        double min = Math.max(-clockwiseLimit(), LEFT_SERVO_REVERSED
                ? (SERVO_FORWARD_POSITION - 1) * SERVO_TRAVEL_DEGREES
                : -SERVO_FORWARD_POSITION * SERVO_TRAVEL_DEGREES);
        double max = Math.min(counterclockwiseLimit(), LEFT_SERVO_REVERSED
                ? SERVO_FORWARD_POSITION * SERVO_TRAVEL_DEGREES
                : (1 - SERVO_FORWARD_POSITION) * SERVO_TRAVEL_DEGREES);
        targetReachable = targetAngle >= min && targetAngle <= max;
        double desiredAngle = Range.clip(targetAngle, min, max);
        double step = Math.max(0, MAX_COMMAND_SPEED_DEGREES_PER_SECOND) * dt;
        commandedAngle += Range.clip(desiredAngle - commandedAngle, -step, step);
        if (Math.abs(desiredAngle - commandedAngle) > READY_ANGLE_TOLERANCE_DEGREES)
            settledSinceNanos = 0;
        double position = SERVO_FORWARD_POSITION
                + (LEFT_SERVO_REVERSED ? -1 : 1) * commandedAngle / SERVO_TRAVEL_DEGREES;
        leftServo.setPosition(Range.clip(position, 0, 1));
        if (state == TurretStates.ODOM_TRACKING && targetReachable
                && Math.abs(desiredAngle - commandedAngle) <= READY_ANGLE_TOLERANCE_DEGREES) {
            if (settledSinceNanos == 0) settledSinceNanos = now;
        } else settledSinceNanos = 0;
    }
    public boolean isReadyToShoot() {
        return state == TurretStates.ODOM_TRACKING && targetReachable && settledSinceNanos != 0
                && (System.nanoTime() - settledSinceNanos) / 1e9 >= SERVO_SETTLE_TIME_SECONDS;
    }
    @Override public void printTelemetry() {
        if (Common.dashTelemetry == null) return;
        Common.dashTelemetry.addData("Turret", "LEFT POSITIONAL SERVO; right disabled");
        Common.dashTelemetry.addData("State / alliance", "%s / %s", state, Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Target / commanded estimate", "%.1f / %.1f deg", targetAngle, commandedAngle);
        Common.dashTelemetry.addData("Servo position", "%.3f", leftServo.getPosition());
        Common.dashTelemetry.addData("Reachable / ready", "%s / %s", targetReachable, isReadyToShoot());
    }
}
