package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "MaxVelocityTest", group = "Main")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx pollenShooter;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        pollenShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                pollenShooter.setPower(1);
                currentVelocity = pollenShooter.getVelocity();

                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }
            }
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Maximum Velocity", maxVelocity);
            telemetry.update();
        }
    }
}