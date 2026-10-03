package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret extends Subsystem<Turret.TurretStates> {
    public enum TurretStates { IDLE, ODOM_TRACKING, MANUAL }

    // Two continuous rotation servos and one analog encoder drive each turret.
    public static final String LEFT_SERVO_NAME = "leftTurretServo";
    public static final String LEFT_SECOND_SERVO_NAME = "leftTurretServo2";
    public static final String RIGHT_SERVO_NAME = "rightTurretServo";
    public static final String RIGHT_SECOND_SERVO_NAME = "rightTurretServo2";
    public static final String LEFT_ENCODER_NAME = "leftTurretEncoder";
    public static final String RIGHT_ENCODER_NAME = "rightTurretEncoder";

    // Turret centers from robot center, in inches: +X forward, +Y left.
    public static double LEFT_OFFSET_X = 0.0, LEFT_OFFSET_Y = 0.0;
    public static double RIGHT_OFFSET_X = 0.0, RIGHT_OFFSET_Y = 0.0;

    // Record these voltages when the turrets point straight forward.
    public static double LEFT_ZERO_VOLTAGE = 0.0, RIGHT_ZERO_VOLTAGE = 0.0;
    public static boolean LEFT_ENCODER_REVERSED = false, RIGHT_ENCODER_REVERSED = false;
    public static double ENCODER_FULL_TURN_VOLTS = 3.3;

    // Tune directions with the gears or belts disconnected first.
    public static boolean LEFT_SERVO_REVERSED = false, LEFT_SECOND_SERVO_REVERSED = true;
    public static boolean RIGHT_SERVO_REVERSED = false, RIGHT_SECOND_SERVO_REVERSED = true;
    public static double LEFT_SECOND_POWER_SCALE = 1.0, RIGHT_SECOND_POWER_SCALE = 1.0;

    // Start narrow; increase only after checking mechanical and cable clearance.
    public static double LEFT_MAX_CLOCKWISE_DEGREES = 30.0;
    public static double LEFT_MAX_COUNTERCLOCKWISE_DEGREES = 30.0;
    public static double RIGHT_MAX_CLOCKWISE_DEGREES = 30.0;
    public static double RIGHT_MAX_COUNTERCLOCKWISE_DEGREES = 30.0;
    public static double KP = 0.01, MAX_POWER = 0.30, MANUAL_MAX_POWER = 0.25;
    public static double READY_ANGLE_TOLERANCE_DEGREES = 2.0;
    public static double SERVO_SETTLE_TIME_SECONDS = 0.20;

    private final CRServo leftServo, leftSecondServo, rightServo, rightSecondServo;
    private final AnalogInput leftEncoder, rightEncoder;
    private final Follower drivetrain;
    private TurretStates state = TurretStates.IDLE;
    private Pose goal;
    private boolean lastAllianceRed, targetingTopHalf;
    private double leftTargetAngle, rightTargetAngle;
    private double leftManualPower = Double.NaN, rightManualPower = Double.NaN;
    private double leftPower, rightPower;
    private boolean leftTargetReachable, rightTargetReachable;
    private long atTargetSinceNanos;
    private double lastLeftRawAngle = Double.NaN, lastRightRawAngle = Double.NaN;
    private double leftContinuousAngle, rightContinuousAngle;

    public Turret(HardwareMap hardwareMap, Follower drivetrain) {
        this.drivetrain = drivetrain;
        leftServo = hardwareMap.get(CRServo.class, LEFT_SERVO_NAME);
        leftSecondServo = hardwareMap.get(CRServo.class, LEFT_SECOND_SERVO_NAME);
        rightServo = hardwareMap.get(CRServo.class, RIGHT_SERVO_NAME);
        rightSecondServo = hardwareMap.get(CRServo.class, RIGHT_SECOND_SERVO_NAME);
        leftEncoder = hardwareMap.get(AnalogInput.class, LEFT_ENCODER_NAME);
        rightEncoder = hardwareMap.get(AnalogInput.class, RIGHT_ENCODER_NAME);
        setPower(0, 0);
        setAlliance();
    }

    @Override public void set(TurretStates newState) {
        state = newState;
        atTargetSinceNanos = 0;
        setPower(0, 0);
    }

    @Override public TurretStates get() { return state; }
    public void setAlliance() { setAlliance(Common.isRed); }
    public void setAlliance(boolean isRed) {
        Common.isRed = isRed;
        targetingTopHalf = drivetrain.getPose().getY() >= Common.FIELD_MIDLINE_Y;
        goal = Common.getGoalForAlliance(isRed, targetingTopHalf);
        lastAllianceRed = isRed;
        atTargetSinceNanos = 0;
    }

    public double getDistance() {
        Pose p = drivetrain.getPose();
        return Math.hypot(goal.getX() - p.getX(), goal.getY() - p.getY());
    }
    public double getLeftTargetAngle() { return leftTargetAngle; }
    public double getRightTargetAngle() { return rightTargetAngle; }
    public double getLeftEncoderVoltage() { return leftEncoder.getVoltage(); }
    public double getRightEncoderVoltage() { return rightEncoder.getVoltage(); }
    public double getLeftAngle() {
        double raw = readAngle(leftEncoder, LEFT_ZERO_VOLTAGE, LEFT_ENCODER_REVERSED);
        if (Double.isNaN(raw)) return Double.NaN;
        if (Double.isNaN(lastLeftRawAngle)) leftContinuousAngle = raw;
        else leftContinuousAngle += normalizeDegrees(raw - lastLeftRawAngle);
        lastLeftRawAngle = raw;
        return leftContinuousAngle;
    }
    public double getRightAngle() {
        double raw = readAngle(rightEncoder, RIGHT_ZERO_VOLTAGE, RIGHT_ENCODER_REVERSED);
        if (Double.isNaN(raw)) return Double.NaN;
        if (Double.isNaN(lastRightRawAngle)) rightContinuousAngle = raw;
        else rightContinuousAngle += normalizeDegrees(raw - lastRightRawAngle);
        lastRightRawAngle = raw;
        return rightContinuousAngle;
    }

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

    private static double readAngle(AnalogInput encoder, double zero, boolean reversed) {
        double volts = encoder.getVoltage();
        if (!Double.isFinite(volts) || volts < 0.0 || volts > ENCODER_FULL_TURN_VOLTS
                || ENCODER_FULL_TURN_VOLTS <= 0.0) return Double.NaN;
        double angle = (volts - zero) * 360.0 / ENCODER_FULL_TURN_VOLTS;
        return normalizeDegrees(reversed ? -angle : angle);
    }

    // Gamepad sticks give speed to each turret pair.
    public void setManualPower(double left, double right) {
        state = TurretStates.MANUAL;
        leftManualPower = Range.clip(left, -MANUAL_MAX_POWER, MANUAL_MAX_POWER);
        rightManualPower = Range.clip(right, -MANUAL_MAX_POWER, MANUAL_MAX_POWER);
        atTargetSinceNanos = 0;
    }

    public void setManualAngles(double left, double right) {
        state = TurretStates.MANUAL;
        leftTargetAngle = normalizeDegrees(left);
        rightTargetAngle = normalizeDegrees(right);
        leftManualPower = rightManualPower = Double.NaN;
        atTargetSinceNanos = 0;
    }

    public void resumeTracking() { set(TurretStates.ODOM_TRACKING); }
    public void stow() { set(TurretStates.IDLE); }

    @Override public void run() {
        if (lastAllianceRed != Common.isRed) setAlliance();
        if (state == TurretStates.ODOM_TRACKING) updateTracking();
        if (state == TurretStates.IDLE) { setPower(0, 0); return; }

        double leftAngle = getLeftAngle(), rightAngle = getRightAngle();
        if (Double.isNaN(leftAngle) || Double.isNaN(rightAngle)) {
            leftTargetReachable = rightTargetReachable = false;
            atTargetSinceNanos = 0;
            setPower(0, 0);
            return;
        }
        if (state == TurretStates.MANUAL && !Double.isNaN(leftManualPower)) {
            setPower(limitManual(leftManualPower, leftAngle,
                            LEFT_MAX_CLOCKWISE_DEGREES, LEFT_MAX_COUNTERCLOCKWISE_DEGREES),
                    limitManual(rightManualPower, rightAngle,
                            RIGHT_MAX_CLOCKWISE_DEGREES, RIGHT_MAX_COUNTERCLOCKWISE_DEGREES));
            return;
        }

        // Choose an equivalent aim (+/- 360) only when the cable limits allow it.
        double leftRoute = chooseRoute(leftTargetAngle, leftAngle,
                LEFT_MAX_CLOCKWISE_DEGREES, LEFT_MAX_COUNTERCLOCKWISE_DEGREES);
        double rightRoute = chooseRoute(rightTargetAngle, rightAngle,
                RIGHT_MAX_CLOCKWISE_DEGREES, RIGHT_MAX_COUNTERCLOCKWISE_DEGREES);
        leftTargetReachable = !Double.isNaN(leftRoute);
        rightTargetReachable = !Double.isNaN(rightRoute);
        double leftError = (leftTargetReachable ? leftRoute : Range.clip(leftTargetAngle,
                -LEFT_MAX_CLOCKWISE_DEGREES, LEFT_MAX_COUNTERCLOCKWISE_DEGREES)) - leftAngle;
        double rightError = (rightTargetReachable ? rightRoute : Range.clip(rightTargetAngle,
                -RIGHT_MAX_CLOCKWISE_DEGREES, RIGHT_MAX_COUNTERCLOCKWISE_DEGREES)) - rightAngle;
        setPower(aimPower(leftError), aimPower(rightError));

        if (state == TurretStates.ODOM_TRACKING && leftTargetReachable && rightTargetReachable
                && Math.abs(leftError) <= READY_ANGLE_TOLERANCE_DEGREES
                && Math.abs(rightError) <= READY_ANGLE_TOLERANCE_DEGREES) {
            if (atTargetSinceNanos == 0) atTargetSinceNanos = System.nanoTime();
        } else atTargetSinceNanos = 0;
    }

    private void updateTracking() {
        Pose robot = drivetrain.getPose();
        boolean topHalf = robot.getY() >= Common.FIELD_MIDLINE_Y;
        if (topHalf != targetingTopHalf) atTargetSinceNanos = 0;
        targetingTopHalf = topHalf;
        goal = Common.getGoalForAlliance(Common.isRed, topHalf);
        double heading = Math.toDegrees(robot.getHeading());
        leftTargetAngle = normalizeDegrees(calculateAngleToGoal(calculateTurretPosition(
                robot, LEFT_OFFSET_X, LEFT_OFFSET_Y)) - heading);
        rightTargetAngle = normalizeDegrees(calculateAngleToGoal(calculateTurretPosition(
                robot, RIGHT_OFFSET_X, RIGHT_OFFSET_Y)) - heading);
    }

    private static double chooseRoute(double target, double current, double cw, double ccw) {
        double best = Double.NaN;
        double bestTravel = Double.POSITIVE_INFINITY;
        for (int turns = -1; turns <= 1; turns++) {
            double route = target + 360.0 * turns;
            if (route < -cw || route > ccw) continue;
            double travel = Math.abs(route - current);
            if (travel < bestTravel) {
                best = route;
                bestTravel = travel;
            }
        }
        return best;
    }
    private static double limitManual(double power, double angle, double cw, double ccw) {
        if (power < 0.0 && angle <= -cw) return 0.0;
        if (power > 0.0 && angle >= ccw) return 0.0;
        return power;
    }
    private static double aimPower(double error) {
        if (Math.abs(error) <= READY_ANGLE_TOLERANCE_DEGREES) return 0.0;
        return Range.clip(KP * error, -MAX_POWER, MAX_POWER);
    }

    private void setPower(double left, double right) {
        leftPower = left;
        rightPower = right;
        leftServo.setPower(LEFT_SERVO_REVERSED ? -left : left);
        leftSecondServo.setPower(Range.clip((LEFT_SECOND_SERVO_REVERSED ? -left : left)
                * LEFT_SECOND_POWER_SCALE, -1.0, 1.0));
        rightServo.setPower(RIGHT_SERVO_REVERSED ? -right : right);
        rightSecondServo.setPower(Range.clip((RIGHT_SECOND_SERVO_REVERSED ? -right : right)
                * RIGHT_SECOND_POWER_SCALE, -1.0, 1.0));
    }

    public boolean isReadyToShoot() {
        return state == TurretStates.ODOM_TRACKING && atTargetSinceNanos != 0
                && (System.nanoTime() - atTargetSinceNanos) / 1e9 >= SERVO_SETTLE_TIME_SECONDS;
    }

    @Override public void printTelemetry() {
        if (Common.dashTelemetry == null) return;
        Common.dashTelemetry.addLine("TURRET");
        Common.dashTelemetry.addData("State", state);
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Goal", "(%.1f, %.1f)", goal.getX(), goal.getY());
        Common.dashTelemetry.addData("Distance", "%.1f in", getDistance());
        Common.dashTelemetry.addData("Left V / angle / target / power", "%.3f / %.1f / %.1f / %.2f",
                getLeftEncoderVoltage(), getLeftAngle(), leftTargetAngle, leftPower);
        Common.dashTelemetry.addData("Right V / angle / target / power", "%.3f / %.1f / %.1f / %.2f",
                getRightEncoderVoltage(), getRightAngle(), rightTargetAngle, rightPower);
        Common.dashTelemetry.addData("Targets reachable", "%s / %s",
                leftTargetReachable, rightTargetReachable);
        Common.dashTelemetry.addData("Ready to shoot", isReadyToShoot());
    }
}
