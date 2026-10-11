package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;

@TeleOp(name = "Encoder Revs Counter", group = "Test")
public class EncoderRevsCounter extends LinearOpMode {
    private static final double TICKS_PER_REV = 4000.0;

    @Override
    public void runOpMode() {
        AnalogInput enc = hardwareMap.get(AnalogInput.class, Turret.LEFT_ENCODER_NAME);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Point turret forward, press Start.");
        telemetry.addLine("A = reset to 0. Then turn the turret one full rotation by hand.");
        telemetry.update();
        waitForStart();

        double fullTurn = enc.getMaxVoltage();
        double lastRaw = rawTicks(enc, fullTurn);
        double accumulated = 0;
        boolean lastA = false;

        while (opModeIsActive()) {
            double raw = rawTicks(enc, fullTurn);
            accumulated += wrapTicks(raw - lastRaw); // handles the 4000 -> 0 wrap
            lastRaw = raw;

            if (gamepad1.a && !lastA) accumulated = 0;
            lastA = gamepad1.a;

            telemetry.addData("Voltage", "%.3f V", enc.getVoltage());
            telemetry.addData("Encoder revs", "%.3f", accumulated / TICKS_PER_REV);
            telemetry.addData("Gear ratio (abs)", "%.3f", Math.abs(accumulated) / TICKS_PER_REV);
            telemetry.update();
            idle();
        }
    }

    private static double rawTicks(AnalogInput enc, double fullTurn) {
        double v = Range.clip(enc.getVoltage(), 0, fullTurn);
        return v / fullTurn * TICKS_PER_REV;
    }

    private static double wrapTicks(double ticks) {
        double half = TICKS_PER_REV / 2.0;
        return (((ticks + half) % TICKS_PER_REV) + TICKS_PER_REV) % TICKS_PER_REV - half;
    }
}