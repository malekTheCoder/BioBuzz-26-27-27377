package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotV1Class extends OpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    private DcMotorEx shooterMotor, intakeMotor;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (forward >= 0.05 || forward <= -0.05) {
            frontLeftPower = forward + strafe + turn;
            backLeftPower = forward - strafe + turn;
            frontRightPower = forward - strafe - turn;
            backRightPower = forward + strafe - turn;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        if (gamepad1.right_trigger >= 0.2) {
            shooterMotor.setVelocity(1500);
        }
    }
}