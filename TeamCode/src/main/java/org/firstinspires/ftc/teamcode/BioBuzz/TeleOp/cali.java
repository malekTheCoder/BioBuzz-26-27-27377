package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;

@TeleOp(name = "Axon MINI MK2 Calibrate", group = "Test")
public class cali extends LinearOpMode {
    @Override public void runOpMode() {
        Turret.loadCalibration(hardwareMap.appContext);
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences(
                Turret.CALIBRATION_PREFS, Context.MODE_PRIVATE);
        Servo servo = hardwareMap.get(Servo.class, Turret.LEFT_SERVO_NAME);
        ServoImplEx output = hardwareMap.get(ServoImplEx.class, Turret.LEFT_SERVO_NAME);
        output.setPwmDisable();
        Turret.configureAxonPwm(servo);

        double travel = prefs.getFloat("travel", (float) Turret.SERVO_TRAVEL_DEGREES);
        double center = prefs.getFloat("center", (float) Turret.SERVO_FORWARD_POSITION);
        boolean reversed = prefs.getBoolean("reversed", Turret.LEFT_SERVO_REVERSED);
        double position = center;
        double clockwise = Double.NaN, counterclockwise = Double.NaN;
        boolean armed = false;
        boolean lastA = false, lastLb = false, lastX = false, lastB = false;
        boolean lastY = false, lastRb = false, lastUp = false, lastDown = false;
        boolean lastDpadLeft = false, lastDpadRight = false;
        String status = "Remove horn before A. Servo will not move on Init.";
        telemetry.addLine(status);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = Math.min((now - lastTime) / 1e9, 0.1);
            lastTime = now;
            if (gamepad1.a && !lastA) {
                position = center;
                servo.setPosition(position);
                output.setPwmEnable();
                armed = true;
                status = "Centered. Fit horn with turret pointing forward.";
            }
            if (armed) {
                double jog = gamepad1.left_stick_x;
                if (Math.abs(jog) > 0.15) {
                    position = Range.clip(position + (reversed ? 1 : -1) * jog * dt * 0.05,
                            0.05, 0.95);
                    servo.setPosition(position);
                }
                if (gamepad1.left_bumper && !lastLb) {
                    center = position;
                    clockwise = counterclockwise = Double.NaN;
                    status = "Forward marked. Mark both safe ends again.";
                }
                if (gamepad1.x && !lastX) {
                    reversed = !reversed;
                    clockwise = counterclockwise = Double.NaN;
                    status = "Stick direction flipped. Re-mark both safe ends.";
                }
                if (gamepad1.b && !lastB) {
                    clockwise = position;
                    status = "Clockwise safe end marked.";
                }
                if (gamepad1.y && !lastY) {
                    counterclockwise = position;
                    status = "Counterclockwise safe end marked.";
                }
                if (gamepad1.dpad_up && !lastUp) travel = Math.min(360, travel + 5);
                if (gamepad1.dpad_down && !lastDown) travel = Math.max(30, travel - 5);
                if (gamepad1.dpad_left && !lastDpadLeft) travel = Math.max(30, travel - 30);
                if (gamepad1.dpad_right && !lastDpadRight) travel = Math.min(360, travel + 30);
                boolean valid = Double.isFinite(clockwise) && Double.isFinite(counterclockwise)
                        && (reversed ? clockwise > center && counterclockwise < center
                        : clockwise < center && counterclockwise > center);
                if (gamepad1.right_bumper && !lastRb) {
                    if (valid) {
                        prefs.edit().putBoolean("saved", true)
                                .putFloat("travel", (float) travel)
                                .putFloat("center", (float) center)
                                .putBoolean("reversed", reversed)
                                .putFloat("clockwise", (float) (Math.abs(clockwise - center) * travel))
                                .putFloat("counterclockwise", (float) (Math.abs(counterclockwise - center) * travel))
                                .putInt("gear_scale_revision", Turret.GEAR_SCALE_REVISION)
                                .apply();
                        status = "SAVED. Restart the bench test to load settings.";
                    } else status = "Cannot save: ends must be on opposite sides of forward.";
                }
                telemetry.addData("Live offset from forward", "%.1f software deg",
                        (position - center) * travel * (reversed ? -1 : 1));
                telemetry.addData("CW safe (mark with B)", Double.isFinite(clockwise)
                        ? Math.abs(clockwise - center) * travel + " software deg" : "not marked");
                telemetry.addData("CCW safe (mark with Y)", Double.isFinite(counterclockwise)
                        ? Math.abs(counterclockwise - center) * travel + " software deg" : "not marked");
            }
            telemetry.addLine("A=center (horn off first) | left stick=slow jog");
            telemetry.addLine("LB=mark forward | X=flip direction");
            telemetry.addLine("B=mark CW end | Y=mark CCW end | RB=save");
            telemetry.addLine("Dpad up/down=travel +/-5 | left/right=travel -/+30");
            telemetry.addData("Servo position / forward", "%.3f / %.3f", position, center);
            telemetry.addData("Travel / reversed", "%.0f deg / %s", travel, reversed);
            telemetry.addData("Status", status);
            telemetry.update();
            lastA = gamepad1.a; lastLb = gamepad1.left_bumper;
            lastX = gamepad1.x; lastB = gamepad1.b; lastY = gamepad1.y;
            lastRb = gamepad1.right_bumper;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left; lastDpadRight = gamepad1.dpad_right;
            idle();
        }
        output.setPwmDisable();
    }
}
