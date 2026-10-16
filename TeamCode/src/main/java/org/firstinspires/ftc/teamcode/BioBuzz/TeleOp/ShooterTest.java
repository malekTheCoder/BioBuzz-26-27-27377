package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;

@TeleOp(name = "shooterTest")
public class ShooterTest extends OpMode {
    //test

    int targetVelo = 0;

    DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMode();
        //shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        //if(gamepad1.a) {
        //    targetVelo = 5;
        //}
//        } else if (gamepad1.b) {
//            targetVelo -= 5;
//        } else if (gamepad1.x) {
//            targetVelo = 0;
//
//        }

       //shooter.setVelocity(360, AngleUnit.DEGREES);
        shooter.setVelocity(5);

        //telemetry.addData("TargatVelo", targetVelo);
        telemetry.addData("Velo", shooter.getVelocity());
        telemetry.addData("Pos", shooter.getCurrentPosition());
    }
}