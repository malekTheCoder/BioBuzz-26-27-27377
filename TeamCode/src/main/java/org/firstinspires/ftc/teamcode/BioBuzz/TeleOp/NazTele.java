package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.NazShooter;

@TeleOp (name = "NazTele")
public class NazTele extends LinearOpMode {
    @Override

    public void runOpMode () throws InterruptedException{

        NazShooter shooter = new NazShooter(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                shooter.spinMotor();
            } else if (gamepad1.a){
                shooter.stopMotor();
            }

            telemetry.addData("Pollen Velocity", shooter.getPollenVelo());
            telemetry.addData("Nectar Velocity", shooter.getNectarVelo());
            telemetry.addData("Speed", shooter.atSpeed());
            telemetry.update();

        }
    }
}
