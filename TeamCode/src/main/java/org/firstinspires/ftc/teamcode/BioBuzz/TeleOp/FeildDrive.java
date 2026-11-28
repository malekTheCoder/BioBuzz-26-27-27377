package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Drive Test", group = "Test")
public class FeildDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        boolean fieldCentric = true;

        telemetry.addLine("Point robot away from driver, then press START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.optionsWasPressed()) imu.resetYaw();
            if (gamepad1.shareWasPressed()) fieldCentric = !fieldCentric;

            // ---- Drive inputs ----
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double driveX = x;
            double driveY = y;
            if (fieldCentric) {
                driveX = x * Math.cos(-heading) - y * Math.sin(-heading);
                driveY = x * Math.sin(-heading) + y * Math.cos(-heading);
            }

            double fl = driveY + driveX + rx;
            double bl = driveY - driveX + rx;
            double fr = driveY - driveX - rx;
            double br = driveY + driveX - rx;

            // ---- Wheel test: hold a button to spin one wheel ----
            boolean testing = gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b;
            if (testing) {
                fl = gamepad1.x ? 0.3 : 0;
                fr = gamepad1.y ? 0.3 : 0;
                bl = gamepad1.a ? 0.3 : 0;
                br = gamepad1.b ? 0.3 : 0;
            }

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(fl), Math.abs(bl)),
                            Math.max(Math.abs(fr), Math.abs(br))
                    )
            );

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // ---- Telemetry ----
            telemetry.addData("Mode", testing ? "WHEEL TEST"
                    : fieldCentric ? "Field centric" : "Robot centric");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("FL / FR", "%.2f / %.2f", fl / max, fr / max);
            telemetry.addData("BL / BR", "%.2f / %.2f", bl / max, br / max);
            telemetry.addLine("X=FL Y=FR A=BL B=BR");
            telemetry.addLine("Options=reset heading  Share=toggle mode");
            telemetry.update();
        }
    }
}