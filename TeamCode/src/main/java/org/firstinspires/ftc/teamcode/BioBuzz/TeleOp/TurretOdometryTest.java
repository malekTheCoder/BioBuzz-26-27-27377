//package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
///**
// * Turret tracking from the REAL Pedro pose, turret + drivetrain only (no shooter/intake).
// * Align -> continuous switching is done inside Turret (setAutoTracking); tune ENTER_CONTINUOUS_DEG
// * and REALIGN_DEG in Dashboard under Turret.
// */
//@Config
//@TeleOp(name = "Turret Odometry Test", group = "Test")
//public class TurretOdometryTest extends LinearOpMode {
//    // Where the robot really starts (inches, degrees; 0 deg = field +X).
//    public static double START_X = 72.0, START_Y = 72.0, START_HEADING_DEGREES = 0.0;
//    // Same ramps the bench test was tuned with. 0 disables.
//    public static double POWER_RAMP_PER_SECOND = 1.0, BRAKE_RAMP_PER_SECOND = 4.0;
//    // Bench test used true. On the mounted turret keep false so cable limits apply.
//    public static boolean FULL_ROTATION = false;
//    // Limit driver spin so the turret can counter-rotate fast enough. 1.0 = full speed.
//    public static double TURN_SCALE = 0.5;
//
//    private boolean lastA, lastLeftBumper;
//    private double worstLag, maxLoopMs;
//    private String relocalizeStatus = "not tried";
//
//    @Override public void runOpMode() {
//        Common.dashTelemetry = new MultipleTelemetry(
//                telemetry, FtcDashboard.getInstance().getTelemetry());
//        Common.dashTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
//        Common.dashTelemetry.setMsTransmissionInterval(150);
//
//        Follower drivetrain = Constants.createFollower(hardwareMap);
//        drivetrain.setPose(startPose()); // before Turret: it reads the pose at construction
//        Turret turret = new Turret(hardwareMap, drivetrain); // Follower::getPose = real odometry
//        TurretFeedforwardTest.loadSavedCalibration(hardwareMap.appContext);
//        turret.stow();
//        turret.setAutoTracking(true);
//
//        LimelightEx limelight = null;
//        try {
//            limelight = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
//            limelight.setPipeline(LimelightEx.PIPELINE_APRILTAG); // relocalize needs this pipeline
//        } catch (IllegalArgumentException ignored) {
//            relocalizeStatus = "no limelight configured";
//        }
//
//        try {
//            while (opModeInInit()) {
//                if (gamepad1.dpad_up) turret.setAlliance(true);
//                if (gamepad1.dpad_down) turret.setAlliance(false);
//                drivetrain.setPose(startPose()); // pick up Dashboard edits before Start
//                turret.run();
//                show(drivetrain, turret, "Point turret FORWARD, set start pose, START");
//                idle();
//            }
//            waitForStart();
//            if (isStopRequested()) return;
//            drivetrain.setPose(startPose());
//            turret.zeroLeftForward(); // physically point forward before Start (also re-picks aim half)
//            drivetrain.startTeleOpDrive(true);
//            long lastLoop = System.nanoTime();
//            lastA = gamepad1.a;
//            lastLeftBumper = gamepad1.left_bumper;
//
//            while (opModeIsActive()) {
//                long now = System.nanoTime();
//                double loopMs = (now - lastLoop) / 1e6;
//                lastLoop = now;
//
//                if (gamepad1.a && !lastA) {
//                    worstLag = maxLoopMs = 0;
//                    turret.resumeTracking(); // also clears a latched fault
//                }
//                lastA = gamepad1.a;
//                if (gamepad1.b) turret.stow(); // stop wins over A
//                if (gamepad1.dpad_up) turret.setAlliance(true);
//                if (gamepad1.dpad_down) turret.setAlliance(false);
//
//                drivetrain.setTeleOpDrive(-gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x, -gamepad1.right_stick_x * TURN_SCALE);
//                drivetrain.update(); // pose MUST be fresh before turret.run()
//
//                if (gamepad1.left_bumper && !lastLeftBumper && limelight != null) {
//                    relocalizeStatus = limelight.relocalize(drivetrain)
//                            ? "applied" : "no fresh fix or jump too large";
//                }
//                lastLeftBumper = gamepad1.left_bumper;
//
//                turret.setPowerRampPerSecond(POWER_RAMP_PER_SECOND, BRAKE_RAMP_PER_SECOND);
//                turret.setFullRotation(FULL_ROTATION);
//                turret.run(); // throttled internally to Turret.MIN_RUN_PERIOD_MS
//
//                boolean tracking = turret.get() == Turret.TurretStates.ODOM_TRACKING
//                        && !turret.isLeftControlTripped();
//                double lag = Math.abs(turret.getLeftTrackingError());
//                if (tracking && !turret.isAligning() && Double.isFinite(lag)) {
//                    worstLag = Math.max(worstLag, lag);
//                    maxLoopMs = Math.max(maxLoopMs, loopMs);
//                }
//                show(drivetrain, turret, null);
//            }
//        } finally {
//            turret.stow();
//            drivetrain.setTeleOpDrive(0, 0, 0);
//            drivetrain.update();
//            if (limelight != null) limelight.stop();
//        }
//    }
//
//    private static Pose startPose() {
//        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEGREES));
//    }
//
//    private void show(Follower drivetrain, Turret turret, String initLine) {
//        Common.dashTelemetry.clearAll();
//        String status;
//        if (initLine != null) status = initLine;
//        else if (turret.get() == Turret.TurretStates.IDLE) status = "STOPPED - A resumes";
//        else if (!Double.isFinite(turret.getLeftAngle())) status = "ENCODER ERROR";
//        else if (turret.isLeftControlTripped()) status = turret.getLeftFaultReason() + " - A resets";
//        else if (!turret.isLeftTargetReachable()) status = "LIMITED - aim is outside turret travel";
//        else if (turret.isAligning()) status = "ALIGNING";
//        else status = turret.isReadyToShoot() ? "ON TARGET" : "TRACKING";
//        Pose p = drivetrain.getPose();
//        Common.dashTelemetry.addData("Status ", status);
//        Common.dashTelemetry.addLine("");
//        Common.dashTelemetry.addData("Robot X/Y", "%6.1f, %6.1f in", p.getX(), p.getY());
//        Common.dashTelemetry.addData("Heading ", "%6.1f deg", Math.toDegrees(p.getHeading()));
//        Common.dashTelemetry.addData("Team    ", Common.isRed ? "RED" : "BLUE");
//        Common.dashTelemetry.addData("Distance", "%6.1f in", turret.getDistance());
//        Common.dashTelemetry.addLine(" ");
//        Common.dashTelemetry.addData("Actual  ", "%6.1f deg", turret.getLeftAngle());
//        Common.dashTelemetry.addData("Target  ", "%6.1f deg", turret.getLeftCommandedAngle());
//        Common.dashTelemetry.addData("Lag     ", "%6.1f deg", Math.abs(turret.getLeftTrackingError()));
//        Common.dashTelemetry.addData("Worst   ", "%6.1f deg", worstLag);
//        Common.dashTelemetry.addData("Max loop", "%6.0f ms", maxLoopMs);
//        Common.dashTelemetry.addData("Power   ", "%6.2f", turret.getLeftPower());
//        Common.dashTelemetry.addData("Speed   ", "%+7.1f deg/s", turret.getLeftSpeed());
//        Common.dashTelemetry.addData("Aim spd ", "%+7.1f deg/s", turret.getLeftTargetSpeed());
//        Common.dashTelemetry.addData("Relocalize", relocalizeStatus);
//        Common.dashTelemetry.addLine("  ");
//        Common.dashTelemetry.addLine("A = track/resume   B = STOP   dpad up/down = red/blue");
//        Common.dashTelemetry.addLine("LB = relocalize from Limelight");
//        Common.dashTelemetry.update();
//    }
//}
