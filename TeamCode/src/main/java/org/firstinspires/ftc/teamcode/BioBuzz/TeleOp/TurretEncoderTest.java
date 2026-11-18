package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TurretPositionControl;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {
    @Override public void runOpMode() {
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, Turret.LEFT_ENCODER_NAME);
        CRServo servo = hardwareMap.get(CRServo.class, Turret.LEFT_SERVO_NAME);
        servo.setPower(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TurretPositionControl control = new TurretPositionControl(encoder);
        Double target = null;
        boolean lastA = false;
        try {
            telemetry.addLine("LEFT ONLY: point turret FORWARD before Start (0 deg).");
            telemetry.addLine("Dpad: up=180, right=90, left=270, down=0. B=stop. A=zero + stop.");
            telemetry.addLine("Tuning lives in Dashboard > Turret; same controller as tracking.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            control.zeroForward();

            while (opModeIsActive()) {
                control.update();
                if (gamepad1.a && !lastA) {
                    control.zeroForward();
                    target = null; // don't send it to an old target after re-zeroing
                }
                lastA = gamepad1.a;
                if (gamepad1.dpad_up) target = 180.0;
                else if (gamepad1.dpad_right) target = 90.0;
                else if (gamepad1.dpad_left) target = 270.0;
                else if (gamepad1.dpad_down) target = 0.0;
                if (gamepad1.b) target = null;

                double power = 0;
                if (target == null) control.stop();
                else power = control.powerTo(target);
                servo.setPower(Turret.LEFT_SERVO_REVERSED ? -power : power);

                telemetry.addLine("LEFT ONLY - position test can command 270 deg; check wire clearance.");
                telemetry.addLine("Dpad up=180 right=90 left=270 down=0; B=stop A=zero + stop");
                if (control.isTripped()) telemetry.addLine("STOPPED: error grew. Check direction; B resets guard.");
                telemetry.addData("Voltage", "%.3f V", encoder.getVoltage());
                telemetry.addData("Encoder revs / ratio", "%.3f / %.3f", control.getEncoderRevs(), Turret.GEAR_RATIO);
                telemetry.addData("Turret angle", "%.1f deg", control.getAngle());
                telemetry.addData("Target", target == null ? "none" : target);
                telemetry.addData("Error", "%.1f deg", target == null ? 0 : target - control.getAngle());
                telemetry.addData("Speed / boost / power", "%.1f / %.3f / %.2f",
                        control.getSpeed(), control.getBoost(), power);
                telemetry.update();
                idle();
            }
        } finally { servo.setPower(0); }
    }
}
