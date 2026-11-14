
package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "v1tele", group = "Main")
public class v1tele extends LinearOpMode {

    // ---- Flywheel speeds in ticks/sec ----
    public static double POLLEN_TARGET = 1600, NECTAR_TARGET = 1850;
    public static double TOLERANCE_TPS = 70;
    public static double MAX_TPS = 2000;

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor loader = hardwareMap.dcMotor.get("transfer");

        // Flywheel motors
        loader.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx pollenShooter = hardwareMap.get(
                DcMotorEx.class, "pollenShooter"
        );

        DcMotorEx nectarShooter = hardwareMap.get(
                DcMotorEx.class, "nectarShooter"
        );

        pollenShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pollenShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nectarShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        // Gate servos
        Servo pollenGate = hardwareMap.get(Servo.class, "pollen_gate");
        Servo nectarGate = hardwareMap.get(Servo.class, "nectar_gate");

        pollenGate.setDirection(Servo.Direction.FORWARD);
        nectarGate.setDirection(Servo.Direction.FORWARD);

        final double GATE_OPEN = 0.4;
        final double GATE_CLOSED = 0.2;

        pollenGate.setPosition(GATE_CLOSED);
        nectarGate.setPosition(GATE_CLOSED);

        boolean lastY = false;

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastLeft = false;
        boolean lastRight = false;

        boolean editPollen = true;
        boolean velocityMode = false;

        double lastPower = Double.NaN;

        waitForStart();

        while (opModeIsActive()) {

            // ---- Drive inputs ----

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // ---- Intake and loader ----

            if (gamepad1.a) {
                intake.setPower(1);
                loader.setPower(1);
            } else {
                intake.setPower(0);
                loader.setPower(0);
            }

            // ---- Flywheel target adjustments ----

            // D-pad left = Pollen
            // D-pad right = Nectar
            // D-pad up/down = +/- 100
            // Hold left trigger = +/- 25

            if (gamepad1.dpad_left && !lastLeft) {
                editPollen = true;
            }

            if (gamepad1.dpad_right && !lastRight) {
                editPollen = false;
            }

            double step = gamepad1.left_trigger > 0.5 ? 25 : 100;

            double delta = 0;

            if (gamepad1.dpad_up && !lastUp) {
                delta = step;
            }

            if (gamepad1.dpad_down && !lastDown) {
                delta = -step;
            }

            if (editPollen) {
                POLLEN_TARGET = Math.max(
                        0,
                        Math.min(MAX_TPS, POLLEN_TARGET + delta)
                );
            } else {
                NECTAR_TARGET = Math.max(
                        0,
                        Math.min(MAX_TPS, NECTAR_TARGET + delta)
                );
            }

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            // ---- Read flywheel velocities ----

            double pollenVelo = pollenShooter.getVelocity();
            double nectarVelo = nectarShooter.getVelocity();

            boolean atSpeed = false;

            // Pollen motor reads negative, so its target is negative too.
            double pollenVelocityCommand = -POLLEN_TARGET;

            // ---- Shooting ----

            // Hold B = velocity control
            if (gamepad1.b) {

                if (!velocityMode) {

                    pollenShooter.setMode(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

                    nectarShooter.setMode(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

                    velocityMode = true;
                    lastPower = Double.NaN;
                }

                // Pollen gets a NEGATIVE target.
                // Nectar keeps its original POSITIVE target.
                pollenShooter.setVelocity(pollenVelocityCommand);
                nectarShooter.setVelocity(NECTAR_TARGET);

                // Check both flywheels against their signed targets.
                atSpeed = POLLEN_TARGET > 0
                        && NECTAR_TARGET > 0
                        && Math.abs(
                        pollenVelo - pollenVelocityCommand
                ) <= TOLERANCE_TPS
                        && Math.abs(
                        nectarVelo - NECTAR_TARGET
                ) <= TOLERANCE_TPS;

                // Gates are still manually controlled with Y.

            } else {

                if (velocityMode) {

                    pollenShooter.setMode(
                            DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    );

                    nectarShooter.setMode(
                            DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    );

                    velocityMode = false;
                    lastPower = Double.NaN;

                    pollenGate.setPosition(GATE_CLOSED);
                    nectarGate.setPosition(GATE_CLOSED);
                }

                // Right trigger = raw power mode
                double power = gamepad1.right_trigger > 0.05
                        ? gamepad1.right_trigger
                        : 0;

                if (power != lastPower) {
                    pollenShooter.setPower(power);
                    nectarShooter.setPower(power);

                    lastPower = power;
                }
            }

            // ---- Manual gate control ----

            // Hold Y to open both gates.
            // Release Y to close both gates.

            if (!gamepad1.b) {

                if (gamepad1.y) {

                    pollenGate.setPosition(GATE_OPEN);
                    nectarGate.setPosition(GATE_OPEN);

                } else if (lastY) {

                    pollenGate.setPosition(GATE_CLOSED);
                    nectarGate.setPosition(GATE_CLOSED);
                }
            }

            lastY = gamepad1.y;

            // ---- Mecanum drive ----

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

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

            telemetry.addData("rx", rx);
            telemetry.addData("y", y);

            telemetry.addData("Velocity mode (B)", velocityMode);
            telemetry.addData("Trigger power", gamepad1.right_trigger);

            telemetry.addData(
                    "D-pad adjusting",
                    editPollen ? "POLLEN" : "NECTAR"
            );

            telemetry.addData("Pollen target", POLLEN_TARGET);
            telemetry.addData(
                    "Pollen velocity command",
                    pollenVelocityCommand
            );
            telemetry.addData("Pollen velocity", pollenVelo);

            telemetry.addData("Nectar target", NECTAR_TARGET);
            telemetry.addData("Nectar velocity", nectarVelo);

            telemetry.addData("At speed", atSpeed);

            telemetry.addData(
                    "Gate pollen / nectar pos",
                    "%.2f / %.2f",
                    pollenGate.getPosition(),
                    nectarGate.getPosition()
            );

            telemetry.update();
        }
    }
}
