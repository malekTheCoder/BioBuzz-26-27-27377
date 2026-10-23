package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Simple Turret Tracking", group = "Test")
public class SimpleTurretTeleOp extends LinearOpMode {
    // Set these in Dashboard to where the robot really starts (inches, degrees).
    public static double START_X = 55.5, START_Y = 8.5;
    public static double START_HEADING_DEGREES = 90.0;
    // Keep mounted travel bounded until the mechanism and cable clearance are measured.
    public static boolean FULL_ROTATION = false;
    private String relocalizeStatus = "not requested";
    private String visibleTags = "none";

    @Override
    public void runOpMode() {
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        Follower drivetrain = Constants.createFollower(hardwareMap);
        drivetrain.setPose(new Pose(START_X, START_Y,
                Math.toRadians(START_HEADING_DEGREES)));
        Turret turret = new Turret(hardwareMap, drivetrain);
        turret.setAlliance(true); // default to red for this mounted tracking test
        turret.setFullRotation(FULL_ROTATION);
        turret.stow(); // no movement until you press A after Start

        LimelightEx limelight = null;
        try {
            limelight = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
            limelight.setPipeline(LimelightEx.PIPELINE_APRILTAG);
        } catch (IllegalArgumentException ignored) {
            relocalizeStatus = "camera 'limelight' not configured";
        }

        try {
            while (opModeInInit()) {
                turret.setFullRotation(FULL_ROTATION);
                if (gamepad1.dpad_left) turret.setAlliance(true);
                if (gamepad1.dpad_right) turret.setAlliance(false);
                updateVision(limelight, drivetrain);
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
            boolean lastDpadUp = gamepad1.dpad_up;

            while (opModeIsActive()) {
                turret.setFullRotation(FULL_ROTATION);
                // One press starts tracking; B stops it even if A is held.
                if (gamepad1.a && !lastA) turret.resumeTracking();
                if (gamepad1.b) turret.stow();
                lastA = gamepad1.a;
                if (gamepad1.dpad_left) turret.setAlliance(true);
                if (gamepad1.dpad_right) turret.setAlliance(false);

                drivetrain.setTeleOpDrive(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                drivetrain.update(); // update robot position before aiming
                updateVision(limelight, drivetrain);
                if (gamepad1.dpad_up && !lastDpadUp && limelight != null) {
                    relocalizeStatus = limelight.relocalize(drivetrain)
                            ? "applied" : limelight.getRelocalizeStatus();
                }
                lastDpadUp = gamepad1.dpad_up;
                turret.run(); // this is what keeps tracking after you release A
                showTelemetry(drivetrain, turret);
            }
        } finally {
            turret.stow(); // stop servo power when the OpMode ends
            drivetrain.setTeleOpDrive(0, 0, 0);
            drivetrain.update();
            if (limelight != null) limelight.stop();
        }
    }

    private void updateVision(LimelightEx limelight, Follower drivetrain) {
        if (limelight == null) return;
        // Feed odometry yaw continuously so MegaTag2 is ready when D-pad up is pressed.
        // Reading a pose here does not apply it; only the button corrects Pedro X/Y.
        limelight.getPoseEstimate(drivetrain.getHeading());
        LLResult result = limelight.getResult();
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != LimelightEx.PIPELINE_APRILTAG
                || result.getStaleness() >= LimelightEx.STALENESS_TIME) {
            visibleTags = "none (no fresh AprilTag result)";
            return;
        }
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (ids.length() > 0) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        visibleTags = ids.length() == 0 ? "none" : ids.toString();
    }

    private void showTelemetry(Follower drivetrain, Turret turret) {
        Common.dashTelemetry.addLine("GAMEPAD 1: A = track, B = stop turrets");
        Common.dashTelemetry.addLine("LEFT ONLY: point turret FORWARD before Start; ratio = 4.782");
        Common.dashTelemetry.addLine("D-pad UP = Limelight relocalize");
        Common.dashTelemetry.addLine("D-pad LEFT = red, RIGHT = blue");
        Common.dashTelemetry.addLine("Left stick = drive, right stick X = turn");
        Common.dashTelemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
                drivetrain.getPose().getX(), drivetrain.getPose().getY(),
                Math.toDegrees(drivetrain.getPose().getHeading()));
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Tags seen", visibleTags);
        Common.dashTelemetry.addData("Relocalize", relocalizeStatus);
        turret.printTelemetry();
        Common.dashTelemetry.update();
    }
}
