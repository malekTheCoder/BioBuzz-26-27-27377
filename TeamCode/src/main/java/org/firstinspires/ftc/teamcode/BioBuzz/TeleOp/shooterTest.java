package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "shooterTest")
public class shooterTest extends OpMode {

    double targetVelo = 0;

    DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetVelo = 360;
        } else if (gamepad1.x) {
            targetVelo = 0;
        }

        shooter.setVelocity(targetVelo, AngleUnit.DEGREES);

        telemetry.addData("TargetVelo (deg/s)", targetVelo);
        telemetry.addData("Velo (deg/s)", shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Power", shooter.getPower());
        telemetry.update();
    }
}