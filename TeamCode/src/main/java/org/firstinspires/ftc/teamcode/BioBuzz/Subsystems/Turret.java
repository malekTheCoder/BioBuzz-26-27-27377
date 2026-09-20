package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret extends Subsystem<Turret.TurretStates> {

    // idle is stowed tracking aims by itself and manual listens to the test
    public enum TurretStates {
        IDLE,
        ODOM_TRACKING,
        MANUAL
    }

    // names that have to match the robot config
    public static final String LEFT_SERVO_NAME = "leftTurretServo";
    public static final String RIGHT_SERVO_NAME = "rightTurretServo";

    // turret offsets from robot center in inches +x is forward and +y is left
    public static double LEFT_OFFSET_X = 0.0;
    public static double LEFT_OFFSET_Y = 0.0;
    public static double RIGHT_OFFSET_X = 0.0;
    public static double RIGHT_OFFSET_Y = 0.0;

    // stow positions
    public static double LEFT_STOW_POSITION = 0.5;
    public static double RIGHT_STOW_POSITION = 0.5;

    // servo center position and what angle it points at
    public static double LEFT_CENTER_POSITION = 0.5;
    public static double RIGHT_CENTER_POSITION = 0.5;
    public static double LEFT_CENTER_ANGLE_DEGREES = 0.0;
    public static double RIGHT_CENTER_ANGLE_DEGREES = 0.0;

    // total servo travel in degrees from 0 to 1
    public static double LEFT_TRAVEL_DEGREES = 300.0;
    public static double RIGHT_TRAVEL_DEGREES = 300.0;

    // flip these if a turret moves backwards
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;

    // servo limits so they dont hit the hard stops
    public static double LEFT_MIN_POSITION = 0.02;
    public static double LEFT_MAX_POSITION = 0.98;
    public static double RIGHT_MIN_POSITION = 0.02;
    public static double RIGHT_MAX_POSITION = 0.98;

    // Safe turret rotation from its center. CW is negative; CCW is positive.
    // Wrapping only works if the servo and gearing can physically reach the other angle.
    public static double LEFT_MAX_CLOCKWISE_DEGREES = 140.0;
    public static double LEFT_MAX_COUNTERCLOCKWISE_DEGREES = 140.0;
    public static double RIGHT_MAX_CLOCKWISE_DEGREES = 140.0;
    public static double RIGHT_MAX_COUNTERCLOCKWISE_DEGREES = 140.0;

    public static double COMMAND_DEADBAND = 0.001;
    public static double READY_ANGLE_TOLERANCE_DEGREES = 1.5;
    public static double SERVO_SETTLE_TIME_SECONDS = 0.20;

    // actual hardware and odo object
    private final Servo leftServo;
    private final Servo rightServo;
    private final Follower drivetrain;

    // values the turret keeps track of every loop
    private TurretStates currentState = TurretStates.ODOM_TRACKING;
    private Pose goal;
    private Pose leftTurretPose = new Pose();
    private Pose rightTurretPose = new Pose();

    private double leftTargetAngle;
    private double rightTargetAngle;
    // used to see if the target stopped moving long enough to shoot
    private double leftSettledReferencePosition = Double.NaN;
    private double rightSettledReferencePosition = Double.NaN;
    private double leftCommand = Double.NaN;
    private double rightCommand = Double.NaN;
    private boolean leftTargetReachable;
    private boolean rightTargetReachable;
    private boolean lastAllianceRed;
    private boolean customGoal;
    private boolean targetingTopHalf = true;
    private long lastMeaningfulTargetChangeNanos = System.nanoTime();

    public Turret(HardwareMap hardwareMap, Follower drivetrain) {
        // gets both servos from robot config and starts them at stow
        this.drivetrain = drivetrain;
        leftServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);
        setAlliance();
        commandServoPositions(LEFT_STOW_POSITION, RIGHT_STOW_POSITION);
    }

    @Override
    public void set(TurretStates state) {
        currentState = state;
        leftSettledReferencePosition = Double.NaN;
        rightSettledReferencePosition = Double.NaN;
        lastMeaningfulTargetChangeNanos = System.nanoTime();
    }

    @Override
    public TurretStates get() {
        return currentState;
    }

    public void setAlliance() {
        setAlliance(Common.isRed);
    }

    public void setAlliance(boolean isRed) {
        Common.isRed = isRed;
        customGoal = false;
        targetingTopHalf = drivetrain.getPose().getY() >= Common.FIELD_MIDLINE_Y;
        goal = Common.getGoalForAlliance(isRed, targetingTopHalf);
        lastAllianceRed = isRed;
        resetSettleTimer();
    }

    public void setGoal(Pose newGoal) {
        if (newGoal == null) {
            throw new IllegalArgumentException("Turret goal cannot be nothing");
        }
        goal = new Pose(newGoal.getX(), newGoal.getY(), newGoal.getHeading());
        customGoal = true;
        lastAllianceRed = Common.isRed;
        resetSettleTimer();
    }

    public Pose getGoal() {
        return goal;
    }

    public double getDistance() {
        // distance is used later for shooter tuning
        Pose robotPose = drivetrain.getPose();
        return Math.hypot(goal.getX() - robotPose.getX(), goal.getY() - robotPose.getY());
    }

    public double getLeftDistance() {
        return Math.hypot(goal.getX() - leftTurretPose.getX(), goal.getY() - leftTurretPose.getY());
    }

    public double getRightDistance() {
        return Math.hypot(goal.getX() - rightTurretPose.getX(), goal.getY() - rightTurretPose.getY());
    }

    public double getLeftTargetAngle() {
        return leftTargetAngle;
    }

    public double getRightTargetAngle() {
        return rightTargetAngle;
    }

    public double getLeftServoPosition() {
        return leftCommand;
    }

    public double getRightServoPosition() {
        return rightCommand;
    }

    public boolean isLeftTargetReachable() {
        return leftTargetReachable;
    }

    public boolean isRightTargetReachable() {
        return rightTargetReachable;
    }

    // gets each turrets real field position
    public static Pose calculateTurretPosition(
            Pose robotPose,
            double forwardOffset,
            double leftOffset
    ) {
        // rotates the turret offset with the robot then adds it to field position
        double heading = robotPose.getHeading();
        double fieldX = robotPose.getX()
                + forwardOffset * Math.cos(heading)
                - leftOffset * Math.sin(heading);
        double fieldY = robotPose.getY()
                + forwardOffset * Math.sin(heading)
                + leftOffset * Math.cos(heading);
        return new Pose(fieldX, fieldY, heading);
    }

    // raw field angle from this turret to the goal
    public double calculateAngleToGoal(Pose turretPose) {
        // atan2 gives the field angle between the turret and goal
        return Math.toDegrees(Math.atan2(
                goal.getY() - turretPose.getY(),
                goal.getX() - turretPose.getX()
        ));
    }

    // keeps the angle from -180 to 180
    public static double normalizeDegrees(double angle) {
        double normalized = (angle + 180.0) % 360.0;
        if (normalized < 0.0) normalized += 360.0;
        return normalized - 180.0;
    }

    // manual servo positions from 0 to 1
    public void setManual(double leftPosition, double rightPosition) {
        // skips angle math and sends servo positions directly
        currentState = TurretStates.MANUAL;
        leftTargetReachable = true;
        rightTargetReachable = true;
        commandServoPositions(leftPosition, rightPosition);
    }

    // manual angles relative to the robot
    public void setManualAngles(double leftAngleDegrees, double rightAngleDegrees) {
        // converts each angle into the servo position for that turret
        currentState = TurretStates.MANUAL;
        leftTargetAngle = normalizeDegrees(leftAngleDegrees);
        rightTargetAngle = normalizeDegrees(rightAngleDegrees);

        ServoTarget leftTarget = angleToServoPosition(
                leftTargetAngle,
                LEFT_CENTER_ANGLE_DEGREES,
                LEFT_CENTER_POSITION,
                LEFT_TRAVEL_DEGREES,
                LEFT_REVERSED,
                LEFT_MIN_POSITION,
                LEFT_MAX_POSITION,
                LEFT_MAX_CLOCKWISE_DEGREES,
                LEFT_MAX_COUNTERCLOCKWISE_DEGREES
        );
        ServoTarget rightTarget = angleToServoPosition(
                rightTargetAngle,
                RIGHT_CENTER_ANGLE_DEGREES,
                RIGHT_CENTER_POSITION,
                RIGHT_TRAVEL_DEGREES,
                RIGHT_REVERSED,
                RIGHT_MIN_POSITION,
                RIGHT_MAX_POSITION,
                RIGHT_MAX_CLOCKWISE_DEGREES,
                RIGHT_MAX_COUNTERCLOCKWISE_DEGREES
        );

        leftTargetReachable = leftTarget.reachable;
        rightTargetReachable = rightTarget.reachable;
        commandServoPositions(leftTarget.position, rightTarget.position);
    }

    public void resumeTracking() {
        set(TurretStates.ODOM_TRACKING);
    }

    public void stow() {
        set(TurretStates.IDLE);
    }

    @Override
    public void run() {
        if (lastAllianceRed != Common.isRed) {
            setAlliance();
        }

        switch (currentState) {
            case IDLE:
                leftTargetReachable = true;
                rightTargetReachable = true;
                commandServoPositions(LEFT_STOW_POSITION, RIGHT_STOW_POSITION);
                break;

            case ODOM_TRACKING:
                updateTracking();
                break;

            case MANUAL:
                break;
        }
    }

    private void updateTracking() {
        Pose robotPose = drivetrain.getPose();
        selectGoal(robotPose.getY());
        leftTurretPose = calculateTurretPosition(robotPose, LEFT_OFFSET_X, LEFT_OFFSET_Y);
        rightTurretPose = calculateTurretPosition(robotPose, RIGHT_OFFSET_X, RIGHT_OFFSET_Y);

        double robotHeadingDegrees = Math.toDegrees(robotPose.getHeading());
        double newLeftTarget = normalizeDegrees(calculateAngleToGoal(leftTurretPose) - robotHeadingDegrees);
        double newRightTarget = normalizeDegrees(calculateAngleToGoal(rightTurretPose) - robotHeadingDegrees);

        leftTargetAngle = newLeftTarget;
        rightTargetAngle = newRightTarget;

        ServoTarget leftTarget = angleToServoPosition(
                leftTargetAngle,
                LEFT_CENTER_ANGLE_DEGREES,
                LEFT_CENTER_POSITION,
                LEFT_TRAVEL_DEGREES,
                LEFT_REVERSED,
                LEFT_MIN_POSITION,
                LEFT_MAX_POSITION,
                LEFT_MAX_CLOCKWISE_DEGREES,
                LEFT_MAX_COUNTERCLOCKWISE_DEGREES
        );
        ServoTarget rightTarget = angleToServoPosition(
                rightTargetAngle,
                RIGHT_CENTER_ANGLE_DEGREES,
                RIGHT_CENTER_POSITION,
                RIGHT_TRAVEL_DEGREES,
                RIGHT_REVERSED,
                RIGHT_MIN_POSITION,
                RIGHT_MAX_POSITION,
                RIGHT_MAX_CLOCKWISE_DEGREES,
                RIGHT_MAX_COUNTERCLOCKWISE_DEGREES
        );

        // Compare servo commands so a jump to the other side restarts the settle timer.
        if (Double.isNaN(leftSettledReferencePosition)
                || Double.isNaN(rightSettledReferencePosition)
                || Math.abs(leftTarget.position - leftSettledReferencePosition) * LEFT_TRAVEL_DEGREES
                    > READY_ANGLE_TOLERANCE_DEGREES
                || Math.abs(rightTarget.position - rightSettledReferencePosition) * RIGHT_TRAVEL_DEGREES
                    > READY_ANGLE_TOLERANCE_DEGREES) {
            lastMeaningfulTargetChangeNanos = System.nanoTime();
            leftSettledReferencePosition = leftTarget.position;
            rightSettledReferencePosition = rightTarget.position;
        }

        leftTargetReachable = leftTarget.reachable;
        rightTargetReachable = rightTarget.reachable;
        commandServoPositions(leftTarget.position, rightTarget.position);
    }

    private void selectGoal(double robotY) {
        if (customGoal) return;
        boolean topHalf = robotY >= Common.FIELD_MIDLINE_Y;
        if (topHalf != targetingTopHalf) {
            resetSettleTimer();
        }
        targetingTopHalf = topHalf;
        goal = Common.getGoalForAlliance(Common.isRed, topHalf);
    }

    private void resetSettleTimer() {
        leftSettledReferencePosition = Double.NaN;
        rightSettledReferencePosition = Double.NaN;
        lastMeaningfulTargetChangeNanos = System.nanoTime();
    }

    private static ServoTarget angleToServoPosition(
            double targetAngle,
            double centerAngle,
            double centerPosition,
            double travelDegrees,
            boolean reversed,
            double minPosition,
            double maxPosition,
            double maxClockwiseDegrees,
            double maxCounterclockwiseDegrees
    ) {
        // bad travel number would divide by zero so just hold center
        if (travelDegrees <= 0.0) {
            return new ServoTarget(Range.clip(centerPosition, minPosition, maxPosition), false);
        }

        double directAngle = normalizeDegrees(targetAngle - centerAngle);
        double direction = reversed ? -1.0 : 1.0;
        double clockwiseLimit = Math.max(0.0, maxClockwiseDegrees);
        double counterclockwiseLimit = Math.max(0.0, maxCounterclockwiseDegrees);

        // The same aim can be reached at angles 360 degrees apart.
        double directPosition = centerPosition + direction * directAngle / travelDegrees;
        if (directAngle >= -clockwiseLimit && directAngle <= counterclockwiseLimit
                && directPosition >= minPosition && directPosition <= maxPosition) {
            return new ServoTarget(directPosition, true);
        }

        double wrapAngle = directAngle > 0.0 ? directAngle - 360.0 : directAngle + 360.0;
        double wrapPosition = centerPosition + direction * wrapAngle / travelDegrees;
        if (wrapAngle >= -clockwiseLimit && wrapAngle <= counterclockwiseLimit
                && wrapPosition >= minPosition && wrapPosition <= maxPosition) {
            return new ServoTarget(wrapPosition, true);
        }

        // Neither route reaches the target. Hold at the nearest safe angle.
        double safeAngle = Range.clip(directAngle, -clockwiseLimit, counterclockwiseLimit);
        double safePosition = centerPosition + direction * safeAngle / travelDegrees;
        return new ServoTarget(Range.clip(safePosition, minPosition, maxPosition), false);
    }

    private void commandServoPositions(double requestedLeft, double requestedRight) {
        // never let a command go past the tuned safe limits
        double safeLeft = Range.clip(requestedLeft, LEFT_MIN_POSITION, LEFT_MAX_POSITION);
        double safeRight = Range.clip(requestedRight, RIGHT_MIN_POSITION, RIGHT_MAX_POSITION);

        // deadband stops tiny changes from making the servos buzz
        if (Double.isNaN(leftCommand) || Math.abs(safeLeft - leftCommand) >= COMMAND_DEADBAND) {
            leftServo.setPosition(safeLeft);
            leftCommand = safeLeft;
        }
        if (Double.isNaN(rightCommand) || Math.abs(safeRight - rightCommand) >= COMMAND_DEADBAND) {
            rightServo.setPosition(safeRight);
            rightCommand = safeRight;
        }
    }

    public boolean isReadyToShoot() {
        // no servo encoders so this only checks reach and how long the target stayed still
        if (currentState != TurretStates.ODOM_TRACKING
                || !leftTargetReachable
                || !rightTargetReachable) {
            return false;
        }
        double stableSeconds = (System.nanoTime() - lastMeaningfulTargetChangeNanos) / 1e9;
        return stableSeconds >= SERVO_SETTLE_TIME_SECONDS;
    }

    @Override
    public void printTelemetry() {
        if (Common.dashTelemetry == null) return;

        Common.dashTelemetry.addLine("Diddy TURRET");
        Common.dashTelemetry.addData("State", currentState);
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Goal half", customGoal ? "CUSTOM" : targetingTopHalf ? "TOP" : "BOTTOM");
        Common.dashTelemetry.addData("Goal", "(%.1f, %.1f)", goal.getX(), goal.getY());
        Common.dashTelemetry.addData("Distance", "%.1f in", getDistance());
        Common.dashTelemetry.addData("Left angle / servo", "%.1f deg / %.3f",
                leftTargetAngle, leftCommand);
        Common.dashTelemetry.addData("Right angle / servo", "%.1f deg / %.3f",
                rightTargetAngle, rightCommand);
        Common.dashTelemetry.addData("Targets reachable", "%s / %s",
                leftTargetReachable, rightTargetReachable);
        Common.dashTelemetry.addData("Ready to shoot", isReadyToShoot());
    }

    private static final class ServoTarget {
        final double position;
        final boolean reachable;

        ServoTarget(double position, boolean reachable) {
            this.position = position;
            this.reachable = reachable;
        }
    }
}
