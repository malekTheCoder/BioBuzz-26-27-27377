package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Axon MINI MK2 Tracking Tuner", group = "Test")
public class trackerruner extends LinearOpMode {
    public static double START_X = 55.5, START_Y = 8.5, START_HEADING_DEGREES = 90;
    public static double TURRET_FORWARD_OFFSET = 5, TURRET_LEFT_OFFSET = -4;
    private static final double TRIAL_FRACTION_OF_SERVO_RANGE = 0.20;
    private static final double MAX_AIM_TRIM_DEGREES = 60;

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEGREES));
    }

    @Override public void runOpMode() {
        if (!Turret.hasSavedCalibration(hardwareMap.appContext)) {
            telemetry.addLine("Save Axon MINI MK2 calibration first.");
            telemetry.update();
            return;
        }
        Follower drivetrain = Constants.createFollower(hardwareMap);
        drivetrain.setPose(startPose());
        Turret.LEFT_OFFSET_X = TURRET_FORWARD_OFFSET;
        Turret.LEFT_OFFSET_Y = TURRET_LEFT_OFFSET;
        Turret turret = new Turret(hardwareMap, drivetrain);
        ServoImplEx output = hardwareMap.get(ServoImplEx.class, Turret.LEFT_SERVO_NAME);
        output.setPwmDisable();
        turret.stow();
        turret.setAlliance(true);
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences(
                Turret.CALIBRATION_PREFS, Context.MODE_PRIVATE);

        telemetry.addLine("Set REAL start pose above. Start centers the left turret.");
        telemetry.addLine("Clear turret and wires. This has no drive or shooter controls.");
        telemetry.addLine("Measure physical trial angle from forward before saving scale.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        drivetrain.setPose(startPose());
        turret.centerLeftForward();
        output.setPwmEnable();
        turret.centerLeftForward();

        boolean armed = true;
        double trialAngle = Double.NaN;
        double measuredAngle = 0;
        String status = "Centered. D-pad left/right selects a safe scale trial.";
        boolean lastX = false, lastLeft = false, lastRight = false;
        boolean lastLb = false, lastRb = false, lastY = false, lastStickButton = false;
        boolean lastRightStickButton = false;
        boolean lastG2Left = false, lastG2Right = false;
        boolean lastG2X = false, lastG2Y = false;
        try {
            while (opModeIsActive()) {
                boolean x = gamepad1.x;
                boolean left = gamepad1.dpad_left;
                boolean right = gamepad1.dpad_right;
                boolean lb = gamepad1.left_bumper;
                boolean rb = gamepad1.right_bumper;
                boolean y = gamepad1.y;
                boolean stickButton = gamepad1.left_stick_button;
                boolean rightStickButton = gamepad1.right_stick_button;
                boolean g2Left = gamepad2.dpad_left;
                boolean g2Right = gamepad2.dpad_right;
                boolean g2X = gamepad2.x;
                boolean g2Y = gamepad2.y;

                if (gamepad1.b || gamepad2.b) {
                    turret.stow();
                    output.setPwmDisable();
                    armed = false;
                    trialAngle = Double.NaN;
                    status = "PWM OFF. Press X to center and re-enable.";
                } else if (x && !lastX) {
                    turret.centerLeftForward();
                    output.setPwmEnable();
                    turret.centerLeftForward();
                    armed = true;
                    trialAngle = Double.NaN;
                    measuredAngle = 0;
                    status = "Centered. Select a scale trial or press A to track.";
                } else if (armed) {
                    if ((left && !lastLeft) || (right && !lastRight)) {
                        boolean ccw = left && !lastLeft;
                        double safeLimit = ccw ? Turret.LEFT_MAX_COUNTERCLOCKWISE_DEGREES
                                : Turret.LEFT_MAX_CLOCKWISE_DEGREES;
                        double magnitude = Math.min(TRIAL_FRACTION_OF_SERVO_RANGE
                                * Turret.SERVO_TRAVEL_DEGREES, 0.8 * safeLimit);
                        trialAngle = (ccw ? 1 : -1) * magnitude;
                        measuredAngle = 0;
                        turret.setManualAngles(trialAngle, 0);
                        status = "Wait for motion to stop; enter measured physical angle with bumpers.";
                    }
                    if (turret.get() == Turret.TurretStates.ODOM_TRACKING) {
                        if (rightStickButton && !lastRightStickButton) {
                            Turret.LEFT_AIM_TRIM_DEGREES = 0;
                            status = "Aim trim reset to zero. Press Y to save.";
                        }
                        if (lb && !lastLb)
                            Turret.LEFT_AIM_TRIM_DEGREES = Math.max(-MAX_AIM_TRIM_DEGREES,
                                    Turret.LEFT_AIM_TRIM_DEGREES - 1);
                        if (rb && !lastRb)
                            Turret.LEFT_AIM_TRIM_DEGREES = Math.min(MAX_AIM_TRIM_DEGREES,
                                    Turret.LEFT_AIM_TRIM_DEGREES + 1);
                    } else if (Double.isFinite(trialAngle)) {
                        if (lb && !lastLb) measuredAngle = Math.max(0, measuredAngle - 1);
                        if (rb && !lastRb) measuredAngle = Math.min(180, measuredAngle + 1);
                    }
                    if (y && !lastY && turret.get() == Turret.TurretStates.ODOM_TRACKING) {
                        saveAim(prefs);
                        status = "SAVED tracking trim and direction.";
                    } else if (y && !lastY) {
                        if (Double.isFinite(trialAngle) && Math.abs(trialAngle) >= 1
                                && measuredAngle >= 2
                                && Math.abs(turret.getLeftCommandedAngle() - trialAngle) <= 0.5) {
                            double oldTravel = Turret.SERVO_TRAVEL_DEGREES;
                            double newTravel = measuredAngle * oldTravel / Math.abs(trialAngle);
                            double scale = newTravel / oldTravel;
                            if (Double.isFinite(newTravel) && newTravel >= 20 && newTravel <= 720) {
                                double cw = Turret.LEFT_MAX_CLOCKWISE_DEGREES * scale;
                                double ccw = Turret.LEFT_MAX_COUNTERCLOCKWISE_DEGREES * scale;
                                prefs.edit().putFloat("travel", (float) newTravel)
                                        .putFloat("clockwise", (float) cw)
                                        .putFloat("counterclockwise", (float) ccw)
                                        .putBoolean("gear_scale_calibrated", true)
                                        .putInt("gear_scale_revision", Turret.GEAR_SCALE_REVISION).apply();
                                Turret.SERVO_TRAVEL_DEGREES = newTravel;
                                Turret.LEFT_MAX_CLOCKWISE_DEGREES = cw;
                                Turret.LEFT_MAX_COUNTERCLOCKWISE_DEGREES = ccw;
                                turret.centerLeftForward();
                                trialAngle = Double.NaN;
                                measuredAngle = 0;
                                status = "SAVED turret scale and rescaled both safe ends. Verify with tracking.";
                            } else status = "Scale outside 20..720 deg/full command; remeasure trial.";
                        } else status = "Wait for trial to finish; enter at least 2 physical degrees.";
                    }
                    if (gamepad1.dpad_up) turret.setAlliance(true);
                    if (gamepad1.dpad_down) turret.setAlliance(false);
                    if (gamepad1.a && turret.get() != Turret.TurretStates.ODOM_TRACKING) {
                        trialAngle = Double.NaN;
                        turret.resumeTracking();
                        status = "Tracking. Turn robot slowly and compare physical aim to goal.";
                    }
                    if (g2Left && !lastG2Left)
                        Turret.LEFT_AIM_TRIM_DEGREES = Math.max(-MAX_AIM_TRIM_DEGREES,
                                Turret.LEFT_AIM_TRIM_DEGREES - 1);
                    if (g2Right && !lastG2Right)
                        Turret.LEFT_AIM_TRIM_DEGREES = Math.min(MAX_AIM_TRIM_DEGREES,
                                Turret.LEFT_AIM_TRIM_DEGREES + 1);
                    if ((g2X && !lastG2X) || (stickButton && !lastStickButton))
                        Turret.LEFT_AIM_REVERSED = !Turret.LEFT_AIM_REVERSED;
                    if (g2Y && !lastG2Y) {
                        saveAim(prefs);
                        status = "SAVED tracking trim and direction.";
                    }
                }

                drivetrain.update();
                turret.run();
                Pose pose = drivetrain.getPose();
                telemetry.addData("Status", status);
                telemetry.addData("State / alliance", "%s / %s", turret.get(),
                        Common.isRed ? "RED" : "BLUE");
                telemetry.addData("Robot X/Y/heading", "%.1f / %.1f / %.1f deg",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                telemetry.addData("Turret offset forward/left", "%.2f / %.2f in",
                        Turret.LEFT_OFFSET_X, Turret.LEFT_OFFSET_Y);
                telemetry.addData("Target / commanded estimate", "%.1f / %.1f deg",
                        turret.getLeftTargetAngle(), turret.getLeftCommandedAngle());
                telemetry.addData("Servo command", "%.3f", turret.getLeftPower());
                telemetry.addData("Trial / measured from forward", "%.1f / %.1f deg",
                        trialAngle, measuredAngle);
                telemetry.addData("Effective travel / CW / CCW", "%.1f / %.1f / %.1f deg",
                        Turret.SERVO_TRAVEL_DEGREES, Turret.LEFT_MAX_CLOCKWISE_DEGREES,
                        Turret.LEFT_MAX_COUNTERCLOCKWISE_DEGREES);
                telemetry.addData("Aim trim / reverse / reachable", "%.1f / %s / %s",
                        Turret.LEFT_AIM_TRIM_DEGREES, Turret.LEFT_AIM_REVERSED,
                        turret.isLeftTargetReachable());
                String adjustmentLimit = "none";
                if (turret.get() == Turret.TurretStates.ODOM_TRACKING
                        && !turret.isLeftTargetReachable()) {
                    adjustmentLimit = "SAVED TURRET TRAVEL: check pose, offsets and gearing";
                } else if (Math.abs(Turret.LEFT_AIM_TRIM_DEGREES) >= MAX_AIM_TRIM_DEGREES) {
                    adjustmentLimit = "TRIM +/-60: check pose and offsets";
                }
                telemetry.addData("Adjustment limit", adjustmentLimit);
                telemetry.addData("Command ramp", "%.0f deg/s (servo speed set in programmer)",
                        Turret.MAX_COMMAND_SPEED_DEGREES_PER_SECOND);
                telemetry.addLine("G1 X=center A=track B=PWM off Dpad left/right=trial Up/Down=red/blue");
                telemetry.addLine("G1 trial: LB/RB=measured angle -/+1 deg Y=save scale and ends");
                telemetry.addLine("G1 tracking: LB/RB=trim left stick click=reverse right stick click=zero trim Y=save");
                telemetry.addLine("G2 Dpad left/right=trim X=reverse Y=save aim (optional)");
                telemetry.addLine("Angles are command estimates; verify turret motion and real field pose.");
                telemetry.update();

                lastX = x; lastLeft = left; lastRight = right;
                lastLb = lb; lastRb = rb; lastY = y; lastStickButton = stickButton;
                lastRightStickButton = rightStickButton;
                lastG2Left = g2Left; lastG2Right = g2Right;
                lastG2X = g2X; lastG2Y = g2Y;
                idle();
            }
        } finally {
            turret.stow();
            output.setPwmDisable();
        }
    }

    private static void saveAim(SharedPreferences prefs) {
        prefs.edit().putFloat("aim_trim", (float) Turret.LEFT_AIM_TRIM_DEGREES)
                .putBoolean("aim_reversed", Turret.LEFT_AIM_REVERSED).apply();
    }
}
