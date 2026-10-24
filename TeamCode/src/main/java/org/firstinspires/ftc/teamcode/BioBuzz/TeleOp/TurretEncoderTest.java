package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Only read the encoder; this test doesn't initialize any motors or servos.
        AnalogInput left = hardwareMap.get(AnalogInput.class, Turret.LEFT_ENCODER_NAME);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double leftZero = Turret.LEFT_ZERO_VOLTAGE;
        boolean lastA = false;

        telemetry.addLine("Encoder switch = A (analog). Press Start to read encoder.");
        telemetry.addLine("Point the turret forward, then gamepad 1 A = temporary zero.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double leftVolts = left.getVoltage();

            // Save forward for this test only; copy the voltage into Turret.java later.
            if (gamepad1.a && !lastA) {
                leftZero = leftVolts;
            }
            lastA = gamepad1.a;

            telemetry.addLine("Move the encoder slowly by hand with servo drive disconnected.");
            telemetry.addLine("Gamepad 1 A = current position is forward (temporary).");
            showEncoder("LEFT", leftVolts, leftZero, Turret.LEFT_ENCODER_REVERSED);
            telemetry.addLine("Copy forward-zero voltage into Turret.java to keep it:");
            telemetry.addData("LEFT_ZERO_VOLTAGE", "%.4f", leftZero);
            telemetry.addLine("Changing voltage proves a response; a constant value needs checking.");
            telemetry.update();
            idle();
        }
    }

    private void showEncoder(String side, double volts, double zero, boolean reversed) {
        double fullTurn = Turret.ENCODER_FULL_TURN_VOLTS;
        telemetry.addData(side + " voltage", "%.4f V", volts);
        if (!Double.isFinite(fullTurn) || fullTurn <= 0 || !Double.isFinite(volts)
                || volts < 0 || volts > fullTurn || !Double.isFinite(zero)) {
            telemetry.addLine(side + ": invalid voltage or full-turn/zero setting");
            return;
        }
        // Raw angle wraps at one full turn. Forward-relative angle wraps at +/-180.
        telemetry.addData(side + " raw shaft angle", "%.1f deg", volts * 360.0 / fullTurn);
        double angle = (volts - zero) * 360.0 / fullTurn;
        telemetry.addData(side + " angle from forward", "%.1f deg",
                Turret.normalizeDegrees(reversed ? -angle : angle));
        telemetry.addData(side + " encoder reversed", reversed);
    }
}
