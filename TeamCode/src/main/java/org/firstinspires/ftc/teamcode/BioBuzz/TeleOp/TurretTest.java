package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    // tells the loop what we are testing right now
    private enum TestMode {
        STOP,
        MANUAL_POWER,
        MANUAL_ANGLE,
        ODOM_TRACKING
    }

    // starting pose for odo tracking tests
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEGREES = 0.0;
    // how fast the sticks change the turret and max flywheel power
    public static double ANGLE_SPEED = 90.0;
    public static double FLYWHEEL_MAX_POWER = 1.0;

    // values we change with the sticks in manual mode
    private TestMode mode = TestMode.STOP;
    private double leftAngle;
    private double rightAngle;

    // old button values so holding a button only changes modes once
    private boolean lastA;
    private boolean lastB;
    private boolean lastX;
    private boolean lastY;
    private boolean lastDpadUp;
    private boolean lastDpadDown;
    private boolean lastLeftStickButton;
    private boolean lastRightStickButton;
    private boolean lastLeftBumper;
    private String relocalizeStatus = "not tried";

    @Override
    public void runOpMode() {
        // sends the same telemetry to driver station and dashboard
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // only makes the hardware needed for this test
        // test the four CR servos and two analog encoders
        Follower drivetrain = Constants.createFollower(hardwareMap);
        Turret turret = new Turret(hardwareMap, drivetrain);
        Shooter shooter = new Shooter(hardwareMap);
        LimelightEx limelight = null;
        try {
            limelight = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
        } catch (IllegalArgumentException ignored) {
            relocalizeStatus = "no limelight configured";
        }

        // gives odo a starting field position
        drivetrain.setPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEGREES)
        ));
        drivetrain.startTeleOpDrive(true);

        // starts safe so the turret doesnt auto move in init
        turret.stow();
        turret.run();
        shooter.stop();

        printTelemetry(drivetrain, turret, shooter, 0.0, 0.0);
        Common.dashTelemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // used so stick movement stays the same even if loop speed changes
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();

            // buttons pick the mode then the sticks update that mode
            readModeButtons(turret);
            updateTurretControls(turret, dt);

            // in manual mode the sticks move the turrets instead of driving
            if (mode == TestMode.MANUAL_POWER || mode == TestMode.MANUAL_ANGLE) {
                drivetrain.setTeleOpDrive(0.0, 0.0, 0.0);
            } else {
                drivetrain.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
            }
            // update odo before doing the turret tracking math
            drivetrain.update();
            // One button press asks vision to correct Pedro X/Y; turret uses the new pose.
            if (gamepad1.left_bumper && !lastLeftBumper && limelight != null) {
                relocalizeStatus = limelight.relocalize(drivetrain)
                        ? "applied" : "no fresh fix or jump too large";
            }
            lastLeftBumper = gamepad1.left_bumper;
            turret.run();

            // each trigger runs the flywheel for that turret
            double leftFlywheelPower = Range.clip(
                    gamepad1.left_trigger * FLYWHEEL_MAX_POWER,
                    0.0,
                    1.0
            );
            double rightFlywheelPower = Range.clip(
                    gamepad1.right_trigger * FLYWHEEL_MAX_POWER,
                    0.0,
                    1.0
            );
            shooter.setPower(leftFlywheelPower, rightFlywheelPower);

            printTelemetry(
                    drivetrain,
                    turret,
                    shooter,
                    leftFlywheelPower,
                    rightFlywheelPower
            );
            Common.dashTelemetry.update();
        }

        // stop everything when the opmode ends
        shooter.stop();
        turret.stow();
        turret.run();
        if (limelight != null) limelight.stop();
    }

    private void readModeButtons(Turret turret) {
        // A stops both turret pairs
        if (gamepad1.a && !lastA) {
            mode = TestMode.STOP;
            turret.stow();
        }

        // B makes each stick directly command one turret pair's speed
        if (gamepad1.b && !lastB) {
            mode = TestMode.MANUAL_POWER;
        }

        // X makes each stick change a turret angle in degrees
        if (gamepad1.x && !lastX) {
            mode = TestMode.MANUAL_ANGLE;
            leftAngle = turret.getLeftAngle();
            rightAngle = turret.getRightAngle();
        }

        // Y lets odo aim both turrets at the selected goal by itself
        if (gamepad1.y && !lastY) {
            mode = TestMode.ODOM_TRACKING;
            turret.resumeTracking();
        }

        // dpad picks which alliance goal the turrets aim at
        if (gamepad1.dpad_up && !lastDpadUp) {
            turret.setAlliance(true);
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            turret.setAlliance(false);
        }

        // pressing a stick sets that turret's angle target to zero
        if (gamepad1.left_stick_button && !lastLeftStickButton) {
            leftAngle = 0.0;
        }

        if (gamepad1.right_stick_button && !lastRightStickButton) {
            rightAngle = 0.0;
        }

        // save buttons for the next loop
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastLeftStickButton = gamepad1.left_stick_button;
        lastRightStickButton = gamepad1.right_stick_button;
    }

    private void updateTurretControls(Turret turret, double dt) {
        switch (mode) {
            case STOP:
                // turret.run keeps both pairs stopped
                break;

            case MANUAL_POWER:
                // letting go of a stick stops that turret pair
                turret.setManualPower(-gamepad1.left_stick_y * Turret.MANUAL_MAX_POWER,
                        -gamepad1.right_stick_y * Turret.MANUAL_MAX_POWER);
                break;

            case MANUAL_ANGLE:
                // same stick setup but these numbers are degrees not servo positions
                leftAngle = Turret.normalizeDegrees(
                        leftAngle - gamepad1.left_stick_y * ANGLE_SPEED * dt
                );
                rightAngle = Turret.normalizeDegrees(
                        rightAngle - gamepad1.right_stick_y * ANGLE_SPEED * dt
                );
                turret.setManualAngles(leftAngle, rightAngle);
                break;

            case ODOM_TRACKING:
                // turret.run does all the tracking work in this mode
                break;
        }
    }

    private void printControls() {
        // this shows the controls on driver station and dashboard
        Common.dashTelemetry.addLine("TURRET TEST CONTROLS");
        Common.dashTelemetry.addLine("gamepad1 sticks = drive in stop or tracking");
        Common.dashTelemetry.addLine("gamepad1 left trigger = left flywheel");
        Common.dashTelemetry.addLine("gamepad1 right trigger = right flywheel");
        Common.dashTelemetry.addLine("gamepad1 A = stop turrets");
        Common.dashTelemetry.addLine("gamepad1 B = manual turret speed");
        Common.dashTelemetry.addLine("gamepad1 X = manual angles");
        Common.dashTelemetry.addLine("gamepad1 Y = odo tracking");
        Common.dashTelemetry.addLine("in manual the left/right sticks move each turret");
        Common.dashTelemetry.addLine("gamepad1 dpad up = red down = blue");
        Common.dashTelemetry.addLine("gamepad1 left bumper = relocalize from Limelight");
        Common.dashTelemetry.addLine("press either stick = reset that turret");
    }

    private void printTelemetry(
            Follower drivetrain,
            Turret turret,
            Shooter shooter,
            double leftFlywheelPower,
            double rightFlywheelPower
    ) {
        // live values that help with tuning and finding problems
        printControls();
        Common.dashTelemetry.addLine("");
        Common.dashTelemetry.addData("Test mode", mode);
        Common.dashTelemetry.addData("Relocalize", relocalizeStatus);
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
                drivetrain.getPose().getX(),
                drivetrain.getPose().getY(),
                Math.toDegrees(drivetrain.getPose().getHeading()));
        Common.dashTelemetry.addData("Encoder volts", "%.3f / %.3f",
                turret.getLeftEncoderVoltage(), turret.getRightEncoderVoltage());
        Common.dashTelemetry.addData("Manual angles", "%.1f / %.1f",
                leftAngle, rightAngle);
        Common.dashTelemetry.addData("Left flywheel power / velocity", "%.2f / %.0f",
                leftFlywheelPower, shooter.getLeftVelocity());
        Common.dashTelemetry.addData("Right flywheel power / velocity", "%.2f / %.0f",
                rightFlywheelPower, shooter.getRightVelocity());
        turret.printTelemetry();
    }
}
