package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Simple Turret Tracking", group = "Test")
public class SimpleTurretTeleOp extends LinearOpMode {
    // Set these in Dashboard to where the robot really starts (inches, degrees).
    public static double START_X = 72.0, START_Y = 72.0;
    public static double START_HEADING_DEGREES = 0.0;

    @Override
    public void runOpMode() {
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        Follower drivetrain = Constants.createFollower(hardwareMap);
        drivetrain.setPose(new Pose(START_X, START_Y,
                Math.toRadians(START_HEADING_DEGREES)));
        Turret turret = new Turret(hardwareMap, drivetrain);
        turret.stow(); // no movement until you press A after Start

        try {
            while (opModeInInit()) {
                if (gamepad1.dpad_up) turret.setAlliance(true);
                if (gamepad1.dpad_down) turret.setAlliance(false);
                turret.run();
                showTelemetry(drivetrain, turret);
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            turret.zeroLeftForward(); // physically point forward before Start

            drivetrain.setPose(new Pose(START_X, START_Y,
                    Math.toRadians(START_HEADING_DEGREES)));
            drivetrain.startTeleOpDrive(true); // robot-centric driving
            boolean lastA = false;

            while (opModeIsActive()) {
                // One press starts tracking; B stops it even if A is held.
                if (gamepad1.a && !lastA) turret.resumeTracking();
                if (gamepad1.b) turret.stow();
                lastA = gamepad1.a;
                if (gamepad1.dpad_up) turret.setAlliance(true);
                if (gamepad1.dpad_down) turret.setAlliance(false);

                drivetrain.setTeleOpDrive(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                drivetrain.update(); // update robot position before aiming
                turret.run(); // this is what keeps tracking after you release A
                showTelemetry(drivetrain, turret);
            }
        } finally {
            turret.stow(); // stop servo power when the OpMode ends
            drivetrain.setTeleOpDrive(0, 0, 0);
            drivetrain.update();
        }
    }

    private void showTelemetry(Follower drivetrain, Turret turret) {
        Common.dashTelemetry.addLine("GAMEPAD 1: A = track, B = stop turrets");
        Common.dashTelemetry.addLine("LEFT ONLY: point turret FORWARD before Start; ratio = 4.782");
        Common.dashTelemetry.addLine("D-pad up = red, down = blue");
        Common.dashTelemetry.addLine("Left stick = drive, right stick X = turn");
        Common.dashTelemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
                drivetrain.getPose().getX(), drivetrain.getPose().getY(),
                Math.toDegrees(drivetrain.getPose().getHeading()));
        turret.printTelemetry();
        Common.dashTelemetry.update();
    }
}
