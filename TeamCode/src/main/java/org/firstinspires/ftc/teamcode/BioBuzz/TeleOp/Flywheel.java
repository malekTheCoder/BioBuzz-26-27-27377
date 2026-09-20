package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel extends OpMode {

    DcMotorEx launcher;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(155, 0, 0, 15.9)
        );
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            launcher.setVelocity(1300);
        }
    }
}
