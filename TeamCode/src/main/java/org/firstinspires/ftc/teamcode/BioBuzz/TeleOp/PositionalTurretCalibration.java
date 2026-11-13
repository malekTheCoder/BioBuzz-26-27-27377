//package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;
//
//import android.content.Context;
//import android.content.SharedPreferences;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
//
//@TeleOp(name = "Positional Turret Calibration", group = "Test")
//public class PositionalTurretCalibration extends LinearOpMode {
//    @Override public void runOpMode() {
//        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences(
//                Turret.CALIBRATION_PREFS, Context.MODE_PRIVATE);
//        Servo servo = hardwareMap.get(Servo.class, Turret.LEFT_SERVO_NAME);
//        double travel = prefs.getFloat("travel", (float) Turret.SERVO_TRAVEL_DEGREES);
//        double center = prefs.getFloat("center", (float) Turret.SERVO_FORWARD_POSITION);
//        boolean reversed = prefs.getBoolean("reversed", Turret.LEFT_SERVO_REVERSED);
//        double clockwisePosition = Double.NaN, counterclockwisePosition = Double.NaN;
//        double position = center;
//        boolean armed = false;
//        String status = "Remove horn first. A sends servo to center.";
//        telemetry.addLine(status);
//        telemetry.update();
//        waitForStart();
//        if (isStopRequested()) return;
//        long last = System.nanoTime();
//        boolean lastA = false, lastB = false, lastX = false, lastY = false;
//        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
//        boolean lastSave = false;
//        while (opModeIsActive()) {
//            long now = System.nanoTime();
//            double dt = Math.min((now - last) / 1e9, 0.1);
//            last = now;
//            if (gamepad1.a && !lastA) {
//                position = center;
//                servo.setPosition(position);
//                armed = true;
//                status = "At center. Attach horn with turret forward.";
//            }
//            if (armed) {
//                double stick = gamepad1.left_stick_x;
//                if (Math.abs(stick) > 0.15) {
//                    position = Range.clip(position + stick * dt * 0.08, 0.05, 0.95);
//                    servo.setPosition(position);
//                }
//                if (gamepad1.left_bumper && !lastLeft) {
//                    center = position;
//                    clockwisePosition = Double.NaN;
//                    counterclockwisePosition = Double.NaN;
//                    status = "Forward center marked. Re-mark both safe ends.";
//                }
//                if (gamepad1.x && !lastX) {
//                    reversed = !reversed;
//                    status = "Direction flipped. Re-mark both safe ends.";
//                    clockwisePosition = Double.NaN;
//                    counterclockwisePosition = Double.NaN;
//                }
//                if (gamepad1.b && !lastB) {
//                    clockwisePosition = position;
//                    status = "Clockwise safe end marked.";
//                }
//                if (gamepad1.y && !lastY) {
//                    counterclockwisePosition = position;
//                    status = "Counterclockwise safe end marked.";
//                }
//                if (gamepad1.dpad_up && !lastUp) travel = Math.min(360, travel + 5);
//                if (gamepad1.dpad_down && !lastDown) travel = Math.max(30, travel - 5);
//                boolean valid = Double.isFinite(clockwisePosition)
//                        && Double.isFinite(counterclockwisePosition)
//                        && (reversed ? clockwisePosition > center && counterclockwisePosition < center
//                        : clockwisePosition < center && counterclockwisePosition > center);
//                if (gamepad1.right_bumper && !lastSave) {
//                    if (valid) {
//                        double cw = Math.abs(clockwisePosition - center) * travel;
//                        double ccw = Math.abs(counterclockwisePosition - center) * travel;
//                        prefs.edit().putBoolean("saved", true)
//                                .putFloat("travel", (float) travel)
//                                .putFloat("center", (float) center)
//                                .putBoolean("reversed", reversed)
//                                .putFloat("clockwise", (float) cw)
//                                .putFloat("counterclockwise", (float) ccw).apply();
//                        status = "SAVED. Reopen normal turret TeleOp to load settings.";
//                    } else status = "Cannot save: mark safe ends on opposite sides of forward.";
//                }
//                telemetry.addData("Clockwise limit", Double.isFinite(clockwisePosition)
//                        ? String.format("%.1f deg", Math.abs(clockwisePosition - center) * travel) : "not marked");
//                telemetry.addData("Counterclockwise limit", Double.isFinite(counterclockwisePosition)
//                        ? String.format("%.1f deg", Math.abs(counterclockwisePosition - center) * travel) : "not marked");
//            }
//            telemetry.addLine("A=center (horn off first)  Left stick=slow jog");
//            telemetry.addLine("Left bumper=mark forward  X=flip direction");
//            telemetry.addLine("B=mark clockwise safe end  Y=mark counterclockwise safe end");
//            telemetry.addLine("Dpad up/down=servo travel degrees +/-5  Right bumper=save");
//            telemetry.addData("Servo position / forward", "%.3f / %.3f", position, center);
//            telemetry.addData("Servo travel / reversed", "%.0f deg / %s", travel, reversed);
//            telemetry.addData("Status", status);
//            telemetry.update();
//            lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
//            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
//            lastLeft = gamepad1.left_bumper; lastRight = gamepad1.right_bumper;
//            lastSave = gamepad1.right_bumper;
//        }
//    }
//}
