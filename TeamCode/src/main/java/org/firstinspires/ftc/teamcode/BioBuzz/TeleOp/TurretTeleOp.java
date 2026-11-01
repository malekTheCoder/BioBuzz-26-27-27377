//package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
//
///**
// * Drive + turret auto-aim on the real Pedro pose, using Robot (robot.run() already does
// * bulk read -> drivetrain.update() -> turret.run(), which is the order the turret needs).
// * Add your shooter / intake / gate controls where marked.
// */
//@Config
//@TeleOp(name = "Turret TeleOp", group = "Main")
//public class TurretTeleOp extends LinearOpMode {
//    // Used only when Auto did not run (Common.AUTO_END_POSE still 0,0). Inches / degrees.
//    public static double DEFAULT_START_X = 72.0, DEFAULT_START_Y = 72.0;
//    public static double DEFAULT_START_HEADING_DEGREES = 0.0;
//    // Try a Limelight position fix at Start when Auto did not run.
//    public static boolean USE_LIMELIGHT_START = true;
//    public static long LIMELIGHT_START_TIMEOUT_MS = 1500;
//    // Same ramps the bench test was tuned with. 0 disables.
//    public static double POWER_RAMP_PER_SECOND = 1.0, BRAKE_RAMP_PER_SECOND = 4.0;
//    // Keep false on the mounted turret so the travel limits in Turret apply.
//    public static boolean FULL_ROTATION = false;
//    // Limits driver spin so the turret can counter-rotate. Raise once tracking holds up.
//    public static double TURN_SCALE = 0.5;
//
//    private boolean lastA, lastLeftBumper;
//    private String startSource = "none", relocalizeStatus = "not tried";
//
//    @Override public void runOpMode() {
//        Common.dashTelemetry = new MultipleTelemetry(
//                telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Robot robot = new Robot(hardwareMap); // turret starts stowed
//        Turret turret = robot.turret;
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
//                if (gamepad1.dpad_up) turret.setAlliance(true);   // red
//                if (gamepad1.dpad_down) turret.setAlliance(false); // blue
//                Common.dashTelemetry.addLine("Point turret FORWARD (0 deg), pick alliance, START");
//                Common.dashTelemetry.addData("Alliance", Common.isRed ? "RED" : "BLUE");
//                Common.dashTelemetry.addData("Auto pose",
//                        autoRan() ? "from Auto" : "none - using default start");
//                Common.dashTelemetry.update();
//                idle();
//            }
//            waitForStart();
//            if (isStopRequested()) return;
//
//            // ---- starting pose -------------------------------------------------------
//            if (autoRan()) {
//                robot.drivetrain.setPose(Common.AUTO_END_POSE);
//                startSource = "Auto end pose";
//            } else {
//                double heading = Math.toRadians(DEFAULT_START_HEADING_DEGREES);
//                robot.drivetrain.setPose(new Pose(DEFAULT_START_X, DEFAULT_START_Y, heading));
//                startSource = "default start pose";
//                if (limelight != null && USE_LIMELIGHT_START) {
//                    long t0 = System.currentTimeMillis();
//                    while (opModeIsActive()
//                            && System.currentTimeMillis() - t0 < LIMELIGHT_START_TIMEOUT_MS) {
//                        if (limelight.setStartingPose(robot.drivetrain, heading)) {
//                            startSource = "Limelight fix";
//                            break;
//                        }
//                        sleep(20);
//                    }
//                }
//            }
//            robot.drivetrain.update();
//            robot.drivetrain.startTeleOpDrive(true);
//
//            // Turret MUST be physically forward right now (encoder has no absolute multi-turn).
//            turret.zeroLeftForward();
//            turret.setAutoTracking(true);
//            turret.resumeTracking();
//            lastA = gamepad1.a;
//            lastLeftBumper = gamepad1.left_bumper;
//
//            while (opModeIsActive()) {
//                // ---- turret controls -------------------------------------------------
//                if (gamepad1.a && !lastA) turret.resumeTracking(); // also clears a latched fault
//                lastA = gamepad1.a;
//                if (gamepad1.b) turret.stow(); // stop wins over A
//                if (gamepad1.dpad_up) turret.setAlliance(true);
//                if (gamepad1.dpad_down) turret.setAlliance(false);
//                turret.setPowerRampPerSecond(POWER_RAMP_PER_SECOND, BRAKE_RAMP_PER_SECOND);
//                turret.setFullRotation(FULL_ROTATION);
//
//                // ---- drive -----------------------------------------------------------
//                robot.drivetrain.setTeleOpDrive(-gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x, -gamepad1.right_stick_x * TURN_SCALE);
//
//                // ---- TODO: shooter / intake / gate controls go here ------------------
//
//                robot.run(); // bulk read, drivetrain.update(), turret.run(), shooter.update()...
//
//                if (gamepad1.left_bumper && !lastLeftBumper && limelight != null) {
//                    relocalizeStatus = limelight.relocalize(robot.drivetrain)
//                            ? "applied" : "no fresh fix or jump too large";
//                }
//                lastLeftBumper = gamepad1.left_bumper;
//
//                Pose p = robot.drivetrain.getPose();
//                Common.dashTelemetry.addData("Start from", startSource);
//                Common.dashTelemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
//                        p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
//                Common.dashTelemetry.addData("Relocalize", relocalizeStatus);
//                Common.dashTelemetry.addData("Turret lag", "%.1f deg",
//                        Math.abs(turret.getLeftTrackingError()));
//                if (turret.isLeftControlTripped())
//                    Common.dashTelemetry.addLine("TURRET STOPPED: " + turret.getLeftFaultReason()
//                            + " - press A to resume");
//                robot.printTelemetry();
//                Common.dashTelemetry.update();
//            }
//        } finally {
//            turret.stow();
//            robot.drivetrain.setTeleOpDrive(0, 0, 0);
//            robot.drivetrain.update();
//            if (limelight != null) limelight.stop();
//        }
//    }
//
//    private static boolean autoRan() {
//        return Common.AUTO_END_POSE.getX() != 0 || Common.AUTO_END_POSE.getY() != 0;
//    }
//}
