package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;

@Config
@TeleOp(name = "Turret Fake Pose Test", group = "Test")
public class TurretFakePoseTest extends LinearOpMode {
    // Dashboard can change these live. Inches; 0 deg = field +X, 90 deg = +Y.
    public static double FAKE_X = 24.0, FAKE_Y = 120.0, FAKE_HEADING_DEGREES = 60.0;
    public static double TURN_DEGREES_PER_SECOND = 90.0;
    public static double BENCH_TRAVEL_DEGREES = 120.0;
    public static boolean BENCH_FULL_ROTATION = true;
    public static double REAR_CLOCKWISE_TRAVEL_DEGREES = 185.0;
    public static double BENCH_POWER_RAMP_PER_SECOND = 1.0;
    public static double BENCH_BRAKE_RAMP_PER_SECOND = 4.0;
    public static double MOTION_PERIOD_SECONDS = 16.0;
    public static double MOTION_ACCEL_SECONDS = 2.0;
    // Viewed with +X to the right and +Y up. Every heading is 60 degrees apart.
    private static final String[] POSE_NAMES = {
            "TOP LEFT", "TOP MIDDLE", "TOP RIGHT",
            "BOTTOM LEFT", "BOTTOM MIDDLE", "BOTTOM RIGHT"
    };
    private static final double[][] PRESET_POSES = {
            {24, 120, 60}, {72, 120, 180}, {120, 120, 300},
            {24, 24, 120}, {72, 24, 0}, {120, 24, 240}
    };
    private int presetIndex;
    private boolean motionLoop, aligningLoopStart, rearAim;
    private double motionPhase, motionSpeedScale;
    private double tripPower, tripSpeed, tripError;
    private double worstLoopError, loopTrackingSeconds, maxLoopGapMillis;
    private boolean measuredFeedforward;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastSlower, lastFaster;
    private boolean lastA, lastX, lastY, lastUp, lastDown, lastLeft, lastRight;

    @Override public void runOpMode() {
        setPresetPose(0);
        measuredFeedforward = TurretFeedforwardTest.loadSavedCalibration(hardwareMap.appContext);
        Common.dashTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        Common.dashTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        Common.dashTelemetry.setMsTransmissionInterval(150);
        // Only the turret servo and encoder are initialized; there is no drivetrain.
        Turret turret = new Turret(hardwareMap, () -> new Pose(
                FAKE_X, FAKE_Y, Math.toRadians(normalizeHeading(FAKE_HEADING_DEGREES))));
        turret.stow();
        turret.setTravelLimits(BENCH_TRAVEL_DEGREES, BENCH_TRAVEL_DEGREES);
        turret.setPowerRampPerSecond(BENCH_POWER_RAMP_PER_SECOND, BENCH_BRAKE_RAMP_PER_SECOND);
        turret.setFullRotation(BENCH_FULL_ROTATION);

        try {
            while (opModeInInit()) {
                selectPose(turret);
                turret.run();
                showTelemetry(turret);
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            turret.zeroLeftForward(); // physically point forward before Start
            long lastNanos = System.nanoTime();
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;
            lastSlower = gamepad1.left_trigger > 0.5;
            lastFaster = gamepad1.right_trigger > 0.5;

            while (opModeIsActive()) {
                long now = System.nanoTime();
                double elapsed = (now - lastNanos) / 1e9;
                double dt = Math.min(elapsed, 0.1);
                lastNanos = now;
                boolean selectedPose = selectPose(turret);
                selectLoopSpeed();
                boolean rearPressed = gamepad1.right_bumper && !lastRightBumper;
                if (rearPressed && !selectedPose) {
                    if (rearAim) setPresetPose(presetIndex);
                    else {
                        motionLoop = aligningLoopStart = false;
                        rearAim = true;
                        setRearAimPose();
                    }
                    turret.resumeTracking();
                } else if (gamepad1.left_bumper && !lastLeftBumper && !selectedPose) {
                    if (motionLoop) setPresetPose(presetIndex);
                    else {
                        rearAim = false;
                        motionLoop = aligningLoopStart = true;
                        motionPhase = 0;
                        motionSpeedScale = 0;
                        resetLoopStats();
                        updateMotionPose();
                    }
                    turret.resumeTracking();
                }
                lastLeftBumper = gamepad1.left_bumper;
                lastRightBumper = gamepad1.right_bumper;
                if (gamepad1.a && !lastA) {
                    if (motionLoop && (turret.isLeftControlTripped()
                            || turret.get() == Turret.TurretStates.IDLE)) {
                        // Catch up to the paused pose before restarting fake motion.
                        motionSpeedScale = 0;
                        aligningLoopStart = true;
                        resetLoopStats();
                    }
                    turret.resumeTracking();
                }
                if (selectedPose) turret.resumeTracking();
                if (gamepad1.b) {
                    turret.stow(); // stop wins even when A is pressed
                    if (motionLoop) {
                        motionSpeedScale = 0;
                        aligningLoopStart = true;
                    }
                }
                lastA = gamepad1.a;
                if (motionLoop) {
                    if (turret.get() == Turret.TurretStates.ODOM_TRACKING
                            && !turret.isLeftControlTripped() && Double.isFinite(turret.getLeftAngle())) {
                        // Reach the starting aim before the imaginary robot begins moving.
                        if (aligningLoopStart) {
                            if (turret.isReadyToShoot()) aligningLoopStart = false;
                        } else {
                            double period = Double.isFinite(MOTION_PERIOD_SECONDS)
                                    && MOTION_PERIOD_SECONDS > 0 ? MOTION_PERIOD_SECONDS : 16.0;
                            double ramp = Double.isFinite(MOTION_ACCEL_SECONDS)
                                    && MOTION_ACCEL_SECONDS > 0 ? MOTION_ACCEL_SECONDS : 2.0;
                            double nextScale = Math.min(1, motionSpeedScale + dt / ramp);
                            motionPhase = (motionPhase + 2 * Math.PI * dt / period
                                    * (motionSpeedScale + nextScale) * 0.5) % (2 * Math.PI);
                            motionSpeedScale = nextScale;
                        }
                        updateMotionPose();
                    }
                } else if (!rearAim) {
                    // Right-stick heading control remains available in preset mode.
                    FAKE_HEADING_DEGREES = normalizeHeading(FAKE_HEADING_DEGREES
                            - stick(gamepad1.right_stick_x) * TURN_DEGREES_PER_SECOND * dt);
                }
                turret.setTravelLimits(rearAim
                        ? Math.max(BENCH_TRAVEL_DEGREES, REAR_CLOCKWISE_TRAVEL_DEGREES)
                        : BENCH_TRAVEL_DEGREES, BENCH_TRAVEL_DEGREES);
                turret.setPowerRampPerSecond(BENCH_POWER_RAMP_PER_SECOND, BENCH_BRAKE_RAMP_PER_SECOND);
                turret.setFullRotation(BENCH_FULL_ROTATION);
                turret.setContinuousTracking(motionLoop && !aligningLoopStart);
                boolean wasTripped = turret.isLeftControlTripped();
                double previousPower = turret.getLeftPower();
                turret.run();
                if (!wasTripped && turret.isLeftControlTripped()) {
                    // A latched stop normally displays zero power; retain useful trip evidence.
                    tripPower = previousPower;
                    tripSpeed = Math.abs(turret.getLeftSpeed());
                    tripError = Math.abs(turret.getLeftTrackingError());
                }
                if (motionLoop && !aligningLoopStart && !wasTripped
                        && turret.get() == Turret.TurretStates.ODOM_TRACKING
                        && Double.isFinite(turret.getLeftTrackingError())) {
                    loopTrackingSeconds += elapsed;
                    worstLoopError = Math.max(worstLoopError, Math.abs(turret.getLeftTrackingError()));
                    maxLoopGapMillis = Math.max(maxLoopGapMillis, elapsed * 1000);
                }
                showTelemetry(turret);
                sleep(20); // avoid thousands of tiny encoder/servo corrections per second
            }
        } finally {
            turret.stow();
        }
    }

    private void selectLoopSpeed() {
        boolean slower = gamepad1.left_trigger > 0.5;
        boolean faster = gamepad1.right_trigger > 0.5;
        boolean slowPressed = slower && !lastSlower;
        boolean fastPressed = faster && !lastFaster;
        lastSlower = slower;
        lastFaster = faster;
        if (slowPressed == fastPressed) return;
        double period = MOTION_PERIOD_SECONDS;
        double nextPeriod = slowPressed ? (period < 12 ? 12 : 16)
                : (period > 12 ? 12 : 8);
        if (period == nextPeriod) return;
        MOTION_PERIOD_SECONDS = nextPeriod;
        if (motionLoop) {
            // A speed change realigns and ramps up; it never clears a latched fault.
            motionSpeedScale = 0;
            aligningLoopStart = true;
            resetLoopStats();
        }
    }

    private void resetLoopStats() {
        worstLoopError = loopTrackingSeconds = maxLoopGapMillis = 0;
    }

    private boolean selectPose(Turret turret) {
        if (gamepad1.y && !lastY) {
            turret.setAlliance(!Common.isRed);
            if (rearAim) setRearAimPose();
        }
        boolean up = gamepad1.dpad_up && !lastUp;
        boolean down = gamepad1.dpad_down && !lastDown;
        boolean left = gamepad1.dpad_left && !lastLeft;
        boolean right = gamepad1.dpad_right && !lastRight;
        boolean next = gamepad1.x && !lastX;
        boolean selected = up || down || left || right || next;
        int index = presetIndex;
        if (up) index %= 3;
        else if (down) index = 3 + index % 3;
        if (left) index = (index / 3) * 3 + Math.max(0, index % 3 - 1);
        else if (right) index = (index / 3) * 3 + Math.min(2, index % 3 + 1);
        if (next) index = (index + 1) % PRESET_POSES.length;
        if (selected) setPresetPose(index);
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        FAKE_HEADING_DEGREES = normalizeHeading(FAKE_HEADING_DEGREES);
        return selected;
    }

    private void setPresetPose(int index) {
        motionLoop = aligningLoopStart = rearAim = false;
        presetIndex = index;
        double[] pose = PRESET_POSES[index];
        FAKE_X = pose[0];
        FAKE_Y = pose[1];
        FAKE_HEADING_DEGREES = normalizeHeading(pose[2]);
        // Changing the pretend robot pose must never re-zero the real encoder.
    }

    private void updateMotionPose() {
        // Repeat a figure eight entirely in the bottom half; no goal-switch jump.
        FAKE_X = 72 + 18 * Math.sin(motionPhase);
        FAKE_Y = 24 + 8 * Math.sin(2 * motionPhase);
        FAKE_HEADING_DEGREES = normalizeHeading(90 + (BENCH_FULL_ROTATION
                ? Math.toDegrees(motionPhase) : -60 * Math.sin(motionPhase)));
    }

    private void setRearAimPose() {
        // Put the alliance goal directly behind the imaginary robot; use real tracking math.
        FAKE_X = 72;
        FAKE_Y = 24;
        Pose goal = Common.getGoalForAlliance(Common.isRed, FAKE_Y >= Common.FIELD_MIDLINE_Y);
        FAKE_HEADING_DEGREES = normalizeHeading(Math.toDegrees(Math.atan2(
                goal.getY() - FAKE_Y, goal.getX() - FAKE_X)) + 180);
    }

    private double stick(double value) {
        return Math.abs(value) < 0.08 ? 0 : value;
    }

    private static double normalizeHeading(double degrees) {
        return (degrees % 360 + 360) % 360;
    }

    private void showTelemetry(Turret turret) {
        Common.dashTelemetry.clearAll();
        String status;
        if (opModeInInit()) status = "Point forward, then START";
        else if (turret.get() == Turret.TurretStates.IDLE) status = "STOPPED - A resumes";
        else if (!Double.isFinite(turret.getLeftAngle())) status = "ENCODER ERROR";
        else if (turret.isLeftControlTripped()) status = turret.getLeftFaultReason() + " - STOPPED";
        else if (!turret.isLeftTargetReachable()) status = "LIMITED - requested "
                + Math.round(normalizeHeading(turret.getLeftTargetAngle())) + " deg";
        else if (motionLoop) status = aligningLoopStart ? "ALIGNING BEFORE MOTION"
                : motionSpeedScale < 1 ? "RAMPING UP" : "TRACKING MOVING POSE";
        else if (turret.isReadyToShoot()) status = "ON TARGET";
        else status = "TRACKING";
        Common.dashTelemetry.addData("Status ", status);
        Common.dashTelemetry.addLine("");
        Common.dashTelemetry.addData("Actual ", "%6.1f deg", normalizeHeading(turret.getLeftAngle()));
        Common.dashTelemetry.addData("Target ", "%6.1f deg", normalizeHeading(turret.getLeftCommandedAngle()));
        boolean tripped = turret.isLeftControlTripped();
        Common.dashTelemetry.addData(tripped ? "Trip error" : "Lag    ", "%6.1f deg",
                tripped ? tripError : Math.abs(turret.getLeftTrackingError()));
        if (motionLoop) {
            Common.dashTelemetry.addData("Worst  ", "%6.1f deg", worstLoopError);
            Common.dashTelemetry.addData("Run time", "%6.1f sec", loopTrackingSeconds);
            if (tripped) Common.dashTelemetry.addData("Max gap", "%6.0f ms", maxLoopGapMillis);
        }
        Common.dashTelemetry.addData(tripped ? "Trip power" : "Power  ", "%6.2f",
                tripped ? tripPower : turret.getLeftPower());
        Common.dashTelemetry.addData(tripped ? "Trip speed" : "Speed  ", "%6.1f deg/s",
                tripped ? tripSpeed : Math.abs(turret.getLeftSpeed()));
        Common.dashTelemetry.addLine(" ");
        Common.dashTelemetry.addData("Test   ", rearAim ? "REAR AIM 180 deg"
                : motionLoop ? String.format("LOOP %.0f sec", MOTION_PERIOD_SECONDS) : POSE_NAMES[presetIndex]);
        if (motionLoop) Common.dashTelemetry.addData("Tune   ", measuredFeedforward ? "MEASURED" : "DEFAULT");
        Common.dashTelemetry.addData("Fake X ", "%6.1f in", FAKE_X);
        Common.dashTelemetry.addData("Fake Y ", "%6.1f in", FAKE_Y);
        Common.dashTelemetry.addData("Heading", "%6.1f deg", normalizeHeading(FAKE_HEADING_DEGREES));
        Common.dashTelemetry.addData("Team   ", Common.isRed ? "RED" : "BLUE");
        Common.dashTelemetry.addLine("  ");
        Common.dashTelemetry.addLine("LB = loop   RB = rear 180");
        Common.dashTelemetry.addLine("LT = slower    RT = faster");
        Common.dashTelemetry.addLine("X/D-pad = presets");
        Common.dashTelemetry.addLine("A = resume     B = STOP    Y = team");
        Common.dashTelemetry.update();
    }
}
