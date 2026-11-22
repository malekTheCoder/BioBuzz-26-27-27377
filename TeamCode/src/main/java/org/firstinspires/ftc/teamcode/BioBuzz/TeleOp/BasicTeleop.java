package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Transfer;

@TeleOp
public class BasicTeleop extends OpMode {

    private DcMotor rearLeft, frontLeft, rearRight, frontRight;
    private DcMotor intake;
    private Transfer transfer;
    private Gate gate;
    private IMU imu;

    // Toggle states
    private boolean intakeOn = false;
    private boolean gateOpen = false;

    // Last loop's button states (so a press only toggles once)
    private boolean lastRightBumper = false;
    private boolean lastY = false;

    @Override
    public void init() {
        rearLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        rearRight = hardwareMap.get(DcMotor.class, "back_right");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        // same directions as pedroPathing/Constants.java (right side reversed)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE); // same as Intake.java
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, // matches Drivetrain.java + v1tele
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        gate.gateMovement(false); // start closed
    }

    @Override
    public void loop() {

        // ---------- Field-centric drive ----------
        double y = -gamepad1.left_stick_y; // left stick Y: forward / backward
        double x = gamepad1.left_stick_x;  // left stick X: strafe left / right
        double rx = gamepad1.right_stick_x; // right stick X: turning

        // Options button: reset heading (point robot away from you, then press)
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the stick direction by the robot's heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // fix imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(backRightPower);

        if (gamepad1.right_bumper && !lastRightBumper) {
            intakeOn = !intakeOn;
        }
        lastRightBumper = gamepad1.right_bumper;

        if (intakeOn) {
            intake.setPower(1.0);
            transfer.loadElements(1.0);
        } else {
            intake.setPower(0);
            transfer.stop();
        }

        if (gamepad1.y && !lastY) {
            gateOpen = !gateOpen;
            gate.gateMovement(gateOpen);
        }
        lastY = gamepad1.y;

        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
        telemetry.addData("Intake + Transfer", intakeOn ? "ON" : "OFF");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}
