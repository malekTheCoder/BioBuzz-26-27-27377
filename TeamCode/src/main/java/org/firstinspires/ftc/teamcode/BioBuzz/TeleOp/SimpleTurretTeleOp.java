package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // ---- Jitter tuning (all live-editable in Dashboard) ----
    // Quiet band around the aim: turret stops inside STOP and restarts only outside RESTART.
    // Wider = less jitter, less precise. RESTART must be >= STOP.
    public static double HOLD_STOP_DEG = 2.0, HOLD_RESTART_DEG = 4.0;
    // Smooth follow with feedforward instead of stop/start bursts.
    public static boolean CONTINUOUS = false;
    // Power change allowed per second. 0 = off. Try 4 to 8 to soften kicks.
    public static double POWER_RAMP_PER_SEC = 0.0;

    private String relocalizeStatus = "not requested";
    private String visibleTags = "none";
    private String limelightDebug = "no camera";
    private long lastPipelineSwitchMs = 0;
    private boolean lastSwitchOk = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100); // less telemetry traffic = faster loop
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bulk reads: one hub read per loop instead of one per sensor.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
                applyTurretSettings(turret);
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
            loopTimer.reset();

            while (opModeIsActive()) {
                loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                applyTurretSettings(turret);
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

    // Pushes the Dashboard tuning values into the turret every loop.
    private void applyTurretSettings(Turret turret) {
        turret.setFullRotation(FULL_ROTATION);
        turret.setContinuousTracking(CONTINUOUS);
        turret.setPositionHoldTolerance(HOLD_STOP_DEG, HOLD_RESTART_DEG);
        turret.setPowerRampPerSecond(POWER_RAMP_PER_SEC);
    }

    private void updateVision(LimelightEx limelight, Follower drivetrain) {
        if (limelight == null) {
            limelightDebug = "camera not configured";
            return;
        }
        // Feed odometry yaw continuously so MegaTag2 is ready when D-pad up is pressed.
        // Reading a pose here does not apply it; only the button corrects Pedro X/Y.
        limelight.getPoseEstimate(drivetrain.getHeading());
        LLResult result = limelight.getResult();

        // If there is no result yet, or it comes from the wrong pipeline, resend the
        // switch (max twice a second) until the camera is on the wanted pipeline.
        int wanted = LimelightEx.PIPELINE_APRILTAG;
        boolean wrongPipeline = result == null || result.getPipelineIndex() != wanted;
        if (wrongPipeline && System.currentTimeMillis() - lastPipelineSwitchMs > 500) {
            lastSwitchOk = limelight.getLimelight().pipelineSwitch(wanted);
            lastPipelineSwitchMs = System.currentTimeMillis();
        }

        LLStatus status = limelight.getLimelight().getStatus();
        limelightDebug = "wanted=" + wanted + " switchOk=" + lastSwitchOk
                + " camPipe=" + (status == null ? "?" : status.getPipelineIndex()) + " | "
                + (result == null ? "result null"
                : "valid=" + result.isValid() + " pipe=" + result.getPipelineIndex()
                + " stale=" + result.getStaleness() + "ms tags=" + result.getBotposeTagCount());

        if (result == null || !result.isValid()
                || result.getPipelineIndex() != wanted
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
        Common.dashTelemetry.addLine("A=track  B=stop  DPad: UP=relocalize  L/R=red/blue");
        Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addData("Pose", "%.1f, %.1f, %.1f deg",
                drivetrain.getPose().getX(), drivetrain.getPose().getY(),
                Math.toDegrees(drivetrain.getPose().getHeading()));

        double actual = turret.getLeftAngle();
        double target = turret.getLeftCommandedAngle();
        double offBy = turret.getLeftTrackingError(); // target minus actual
        boolean moving = Math.abs(turret.getLeftPower()) > 0.001;

        String status;
        if (turret.isLeftControlTripped()) status = "FAULT: " + turret.getLeftFaultReason() + " (press A)";
        else if (turret.get() == Turret.TurretStates.IDLE) status = "STOPPED (press A to track)";
        else if (turret.isPausedForDrive()) status = "WAITING FOR ROBOT TO STOP";
        else if (!turret.isLeftTargetReachable()) status = "TARGET OUT OF RANGE";
        else if (turret.isReadyToShoot()) status = "READY - on target";
        else if (moving) status = "MOVING";
        else status = "SETTLED (not ready yet)";

        Common.dashTelemetry.addData("STATUS", status);
        Common.dashTelemetry.addData("Turret actual (deg)", "%.1f", actual);
        Common.dashTelemetry.addData("Turret target (deg)", "%.1f", target);
        Common.dashTelemetry.addData("OFF BY (deg)", "%.1f", offBy);
        Common.dashTelemetry.addData("Turret speed (deg/s)", "%.1f", turret.getLeftSpeed());
        Common.dashTelemetry.addData("Power", "%.2f", turret.getLeftPower());
        Common.dashTelemetry.addData("Goal distance (in)", "%.1f", turret.getDistance());

        Common.dashTelemetry.addData("Tags", visibleTags);
        Common.dashTelemetry.addData("Limelight", limelightDebug);
        Common.dashTelemetry.addData("Relocalize", relocalizeStatus);
        Common.dashTelemetry.addData("Loop ms", "%.1f", loopMs);
        Common.dashTelemetry.update();
    }
}
