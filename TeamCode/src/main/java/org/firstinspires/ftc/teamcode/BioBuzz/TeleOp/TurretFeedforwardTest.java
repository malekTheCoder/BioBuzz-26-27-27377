package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TurretFeedforwardCalibration;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TurretFeedforwardCalibration.Fit;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TurretFeedforwardCalibration.Point;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TurretPositionControl;

@TeleOp(name = "Turret Feedforward Test", group = "Test")
public class TurretFeedforwardTest extends LinearOpMode {
    private static final String PREFS = "turret_bench_feedforward_v1";

    @Override public void runOpMode() {
        CRServo servo = hardwareMap.get(CRServo.class, Turret.LEFT_SERVO_NAME);
        servo.setPower(0);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, Turret.LEFT_ENCODER_NAME);
        TurretPositionControl feedback = new TurretPositionControl(encoder);
        TurretFeedforwardCalibration test = new TurretFeedforwardCalibration();
        MultipleTelemetry screen = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        screen.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        screen.setMsTransmissionInterval(150);
        boolean lastA, lastX, lastUp, lastY, saved = false, showForward = true;
        double ratio = Turret.GEAR_RATIO;
        boolean encoderReversed = Turret.LEFT_ENCODER_REVERSED;
        boolean servoReversed = Turret.LEFT_SERVO_REVERSED;
        try {
            screen.addLine("Point forward, then START.");
            screen.addLine("A = auto calibration (~65 sec)");
            screen.addLine("D-pad UP = short check (~12 sec)");
            screen.addLine("Starts at 0.42, steps DOWN to 0.12.");
            screen.addLine("B = STOP at any time");
            screen.update();
            waitForStart();
            if (isStopRequested()) return;
            feedback.zeroForward();
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastUp = gamepad1.dpad_up;
            lastY = gamepad1.y;
            while (opModeIsActive()) {
                feedback.update();
                double now = System.nanoTime() / 1e9;
                boolean configMatches = ratio == Turret.GEAR_RATIO
                        && encoderReversed == Turret.LEFT_ENCODER_REVERSED
                        && servoReversed == Turret.LEFT_SERVO_REVERSED;
                if (gamepad1.a && !lastA && !gamepad1.b) {
                    saved = false;
                    showForward = true;
                    test.start(now, feedback.getAngle());
                } else if (gamepad1.dpad_up && !lastUp && !gamepad1.b) {
                    saved = false;
                    showForward = true;
                    test.startQuickCheck(now, feedback.getAngle());
                }
                if (gamepad1.y && !lastY && test.getState() != TurretFeedforwardCalibration.State.RUNNING)
                    showForward = !showForward;
                if (!configMatches) {
                    saved = false;
                    test.fail("CONFIG CHANGED - restart OpMode");
                }
                boolean wasRunning = test.getState() == TurretFeedforwardCalibration.State.RUNNING;
                double power = test.update(now, feedback.getAngle());
                if (wasRunning && test.getState() == TurretFeedforwardCalibration.State.FAILED
                        && test.getForward() != null && test.getReverse() != null)
                    showForward = !test.getForward().valid;
                if (!Double.isFinite(Turret.MAX_POWER) || Math.abs(power) > Math.max(0, Turret.MAX_POWER)) {
                    power = test.fail("POWER LIMIT TOO LOW - check tuning");
                }
                if (gamepad1.b) { test.stop(); power = 0; }
                // No position controller or stall boost: measure the actual commanded power.
                servo.setPower(servoReversed ? -power : power);
                if (gamepad1.x && !lastX && !gamepad1.b && configMatches
                        && test.getState() == TurretFeedforwardCalibration.State.DONE) {
                    saveCalibration(hardwareMap.appContext, test.getForward(), test.getReverse());
                    saved = true;
                }
                lastA = gamepad1.a;
                lastX = gamepad1.x;
                lastUp = gamepad1.dpad_up;
                lastY = gamepad1.y;
                screen.clearAll();
                screen.addData("Status", saved ? "SAVED - open Fake Pose Test" : test.getMessage());
                screen.addLine("");
                Point point = test.getHighPowerPoint(showForward);
                if (test.getState() != TurretFeedforwardCalibration.State.RUNNING
                        && test.getState() != TurretFeedforwardCalibration.State.DONE && point != null) {
                    screen.addData("Direction", showForward ? "FORWARD" : "REVERSE");
                    screen.addData("Power", "%.2f", point.power);
                    screen.addData("Step travel", "%.1f deg", Math.abs(point.stepTravel));
                    screen.addData("Measured travel", "%.1f deg", Math.abs(point.measuredTravel));
                    screen.addData("Mean speed", "%.1f deg/s", Math.abs(point.meanSpeed));
                    screen.addData("Speed spread", "%.1f deg/s", point.speedSpread);
                    screen.addData("Steady spread", "%.1f deg/s", point.steadySpread);
                    screen.addData("Result", point.result);
                    Fit directionFit = showForward ? test.getForward() : test.getReverse();
                    if (directionFit != null) screen.addData("Usable powers", "%d / 6", directionFit.count);
                    screen.addLine("Y = view other direction");
                } else if (test.getForward() != null && test.getReverse() != null) {
                    showFit(screen, "Forward", test.getForward(), test, true);
                    screen.addLine(" ");
                    showFit(screen, "Reverse", test.getReverse(), test, false);
                } else {
                    screen.addData("Step", "%d / %d", test.getStep(), test.getStepCount());
                    screen.addData("Direction", test.direction() > 0 ? "FORWARD (+)" : "REVERSE (-)");
                    screen.addData("Power", "%.2f", Math.abs(power));
                    screen.addData("Speed", "%.1f deg/s", Math.abs(feedback.getSpeed()));
                    if (Double.isFinite(test.getLastMeasuredSpeed())) {
                        screen.addData("Last point", test.getLastPointResult());
                        screen.addData("Last mean", "%.1f deg/s", test.getLastMeasuredSpeed());
                    }
                }
                screen.addLine(" ");
                screen.addLine("A = run again    B = STOP");
                if (test.getState() != TurretFeedforwardCalibration.State.RUNNING)
                    screen.addLine("D-pad UP = short check");
                if (test.getState() == TurretFeedforwardCalibration.State.DONE)
                    screen.addLine("X = save for Fake Pose Test");
                screen.update();
                sleep(20);
            }
        } finally {
            servo.setPower(0);
        }
    }

    private static void showFit(MultipleTelemetry screen, String name, Fit fit,
                                TurretFeedforwardCalibration test, boolean forward) {
        screen.addData(name + " points", "%d / 6 steady", fit.count);
        if (fit.valid) {
            screen.addData(name + " kS", "%.5f", fit.kS);
            screen.addData(name + " kV", "%.7f", fit.kV);
        } else {
            screen.addData(name + " too slow", test.getSlowPoints(forward));
            screen.addData(name + " uneven", test.getUnevenPoints(forward));
        }
    }

    private static void saveCalibration(Context context, Fit forward, Fit reverse) {
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
                .putFloat("forwardS", (float) forward.kS).putFloat("forwardV", (float) forward.kV)
                .putFloat("reverseS", (float) reverse.kS).putFloat("reverseV", (float) reverse.kV)
                .putFloat("ratio", (float) Turret.GEAR_RATIO)
                .putBoolean("encoderReversed", Turret.LEFT_ENCODER_REVERSED)
                .putBoolean("servoReversed", Turret.LEFT_SERVO_REVERSED).apply();
        applyCalibration(forward.kS, forward.kV, reverse.kS, reverse.kV);
    }

    /** Only the fake-pose bench mode loads these saved settings. */
    static boolean loadSavedCalibration(Context context) {
        // Do not carry a previously loaded fit across a gearing/direction change.
        Turret.CONTINUOUS_KS_FORWARD = Turret.CONTINUOUS_KS_REVERSE = 0;
        Turret.CONTINUOUS_KV_FORWARD = Turret.CONTINUOUS_KV_REVERSE = -1;
        SharedPreferences prefs = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
        double savedRatio = prefs.getFloat("ratio", -1);
        if (!Double.isFinite(savedRatio) || !Double.isFinite(Turret.GEAR_RATIO)
                || Math.abs(savedRatio - Turret.GEAR_RATIO) > 0.00001
                || prefs.getBoolean("encoderReversed", !Turret.LEFT_ENCODER_REVERSED) != Turret.LEFT_ENCODER_REVERSED
                || prefs.getBoolean("servoReversed", !Turret.LEFT_SERVO_REVERSED) != Turret.LEFT_SERVO_REVERSED) return false;
        double fs = prefs.getFloat("forwardS", -1), fv = prefs.getFloat("forwardV", -1);
        double rs = prefs.getFloat("reverseS", -1), rv = prefs.getFloat("reverseV", -1);
        if (!validGains(fs, fv) || !validGains(rs, rv)) return false;
        applyCalibration(fs, fv, rs, rv);
        return true;
    }

    private static boolean validGains(double ks, double kv) {
        return Double.isFinite(ks) && ks >= 0 && ks < 0.50 && Double.isFinite(kv) && kv > 0;
    }

    private static void applyCalibration(double fs, double fv, double rs, double rv) {
        Turret.CONTINUOUS_KS_FORWARD = fs; Turret.CONTINUOUS_KV_FORWARD = fv;
        Turret.CONTINUOUS_KS_REVERSE = rs; Turret.CONTINUOUS_KV_REVERSE = rv;
    }
}
