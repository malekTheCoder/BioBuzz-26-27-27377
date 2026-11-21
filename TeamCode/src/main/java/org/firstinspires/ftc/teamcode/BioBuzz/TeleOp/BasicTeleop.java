package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Transfer;

@TeleOp
public class BasicTeleop extends OpMode {

    private DcMotor rearLeft, frontLeft, rearRight, frontRight;
    private DcMotor intake;
    private Transfer transfer;
    private Gate gate;

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

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);

        gate.gateMovement(false); // start closed
    }

    @Override
    public void loop() {

        // ---------- Drive ----------
        double forward = -gamepad1.left_stick_y; // left stick Y: forward / backward
        double turn = gamepad1.right_stick_x;    // right stick X: turning

        double leftPower = forward + turn;
        double rightPower = forward - turn;

        double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= max;
        rightPower /= max;

        frontLeft.setPower(leftPower);
        rearLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        rearRight.setPower(rightPower);

        // ---------- Right bumper: toggle intake + transfer ----------
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

        // ---------- Y: toggle gate open / closed ----------
        if (gamepad1.y && !lastY) {
            gateOpen = !gateOpen;
            gate.gateMovement(gateOpen);
        }
        lastY = gamepad1.y;

        telemetry.addData("Intake + Transfer", intakeOn ? "ON" : "OFF");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}
