package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "v1tele", group = "Main")
public class v1tele extends LinearOpMode {

    private IMU imu;
    private DcMotorEx shooter1, shooter2;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        shooter1 = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        shooter2 = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        waitForStart();

        while (opModeIsActive()) {
            // --- mecanum Drive Control ---
            double y = -gamepad1.left_stick_y;    // forward/backward
            double x = gamepad1.left_stick_x;     // left/right strafe
            double rx = gamepad1.right_stick_x;   // rotation

            if (gamepad1.a) {
                intake.setPower(1);
            }

            if (gamepad1.b) {
                intake.setPower(0);
            }

            if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                setShooterPower(1.0);
            } else {
                setShooterPower(0);
            }

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.update();
        }
    }

    private void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
}
