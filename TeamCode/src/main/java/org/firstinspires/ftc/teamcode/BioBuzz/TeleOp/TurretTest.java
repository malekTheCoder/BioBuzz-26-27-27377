package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    // tells the loop what we are testing right now
    private enum TestMode {
        STOW,
        MANUAL_POSITION,
        MANUAL_ANGLE,
        ODOM_TRACKING
    }

    // starting pose for odo tracking tests
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEGREES = 0.0;
    // how fast the sticks change the turret and max flywheel power
    public static double POSITION_SPEED = 0.25;
    public static double ANGLE_SPEED = 90.0;
    public static double FLYWHEEL_MAX_POWER = 1.0;

    // values we change with the sticks in manual mode
    private TestMode mode = TestMode.STOW;
    private double leftPosition = 0.5;
    private double rightPosition = 0.5;
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

    @Override
    public void runOpMode() {
        // sends the same telemetry to driver station and dashboard
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // only makes the hardware needed for this test
        // turret in Robot is still commented out
        Follower drivetrain = Constants.createFollower(hardwareMap);
        Turret turret = new Turret(hardwareMap, drivetrain);
        Shooter shooter = new Shooter(hardwareMap);

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

        // saves the real starting commands so manual mode doesnt jump
        leftPosition = turret.getLeftServoPosition();
        rightPosition = turret.getRightServoPosition();

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
            if (mode == TestMode.MANUAL_POSITION || mode == TestMode.MANUAL_ANGLE) {
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
    }

    private void readModeButtons(Turret turret) {
        // A puts both turrets in their safe stow positions
        if (gamepad1.a && !lastA) {
            mode = TestMode.STOW;
            turret.stow();
        }

        // B makes each stick directly change a servo position from 0 to 1
        if (gamepad1.b && !lastB) {
            mode = TestMode.MANUAL_POSITION;
            leftPosition = turret.getLeftServoPosition();
            rightPosition = turret.getRightServoPosition();
        }

        // X makes each stick change a turret angle in degrees
        if (gamepad1.x && !lastX) {
            mode = TestMode.MANUAL_ANGLE;
            leftAngle = turret.getLeftTargetAngle();
            rightAngle = turret.getRightTargetAngle();
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

        // pressing a stick resets only that turrets test value
        if (gamepad1.left_stick_button && !lastLeftStickButton) {
            leftPosition = 0.5;
            leftAngle = 0.0;
        }

        if (gamepad1.right_stick_button && !lastRightStickButton) {
            rightPosition = 0.5;
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
            case STOW:
                // turret.run keeps sending the stow positions
                break;

            case MANUAL_POSITION:
                // left stick moves left servo and right stick moves right servo
                leftPosition = Range.clip(
                        leftPosition - gamepad1.left_stick_y * POSITION_SPEED * dt,
                        Turret.LEFT_MIN_POSITION,
                        Turret.LEFT_MAX_POSITION
                );
                rightPosition = Range.clip(
                        rightPosition - gamepad1.right_stick_y * POSITION_SPEED * dt,
                        Turret.RIGHT_MIN_POSITION,
                        Turret.RIGHT_MAX_POSITION
                );
                turret.setManual(leftPosition, rightPosition);
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
        Common.dashTelemetry.addLine("gamepad1 sticks = drive in stow or tracking");
        Common.dashTelemetry.addLine("gamepad1 left trigger = left flywheel");
        Common.dashTelemetry.addLine("gamepad1 right trigger = right flywheel");
        Common.dashTelemetry.addLine("gamepad1 A = stow");
        Common.dashTelemetry.addLine("gamepad1 B = manual servo positions");
        Common.dashTelemetry.addLine("gamepad1 X = manual angles");
        Common.dashTelemetry.addLine("gamepad1 Y = odo tracking");
        Common.dashTelemetry.addLine("in manual the left/right sticks move each turret");
        Common.dashTelemetry.addLine("gamepad1 dpad up = red down = blue");
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
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
                drivetrain.getPose().getX(),
                drivetrain.getPose().getY(),
                Math.toDegrees(drivetrain.getPose().getHeading()));
        Common.dashTelemetry.addData("Manual positions", "%.3f / %.3f",
                leftPosition, rightPosition);
        Common.dashTelemetry.addData("Manual angles", "%.1f / %.1f",
                leftAngle, rightAngle);
        Common.dashTelemetry.addData("Left flywheel power / velocity", "%.2f / %.0f",
                leftFlywheelPower, shooter.getLeftVelocity());
        Common.dashTelemetry.addData("Right flywheel power / velocity", "%.2f / %.0f",
                rightFlywheelPower, shooter.getRightVelocity());
        turret.printTelemetry();
    }
}
