
package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
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

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;

@Config
@TeleOp(name = "v1tele", group = "Main")
public class v1tele extends LinearOpMode {

    // ---- Flywheel speeds in ticks/sec ----
    public static double POLLEN_TARGET = 1600, NECTAR_TARGET = 1850;
    public static double TOLERANCE_TPS = 50;
    public static double MAX_TPS = 2000;

    public static double POLLEN_KF = 1.0 / 2600;
    public static double POLLEN_KP = 0.001;
    private IMU imu;

    public double target = 0;

    public double offset = 0;


    private boolean gatePosition = false;


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

        pollenShooter.setDirection(DcMotorSimple.Direction.REVERSE);
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

        final double pollenGATE_OPEN = 0;
        final double pollenGATE_CLOSED = .15;
        final double nectarGATE_OPEN = .4;
        final double nectarGATE_CLOSED = .2;



        pollenGate.setPosition(pollenGATE_CLOSED);
        nectarGate.setPosition(nectarGATE_CLOSED);

        boolean lastY = false;

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastLeft = false;
        boolean lastRight = false;

        boolean editPollen = true;
        boolean velocityMode = false;

        double lastPower = Double.NaN;

        robot = new Robot(hardwareMap);

        robot.drivetrain.setStartingPose(Common.AUTO_END_POSE);

        waitForStart();

        while (opModeIsActive()) {
            robot.bulkReader.bulkRead();
            robot.shooter.updatePIDF();
            robot.drivetrain.update();

            // ---- Drive inputs ----

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            double pollenVelo = pollenShooter.getVelocity();
            double nectarVelo = nectarShooter.getVelocity();

            boolean atSpeed = false;



            double distance = 3;

//            if (gamepad1.rightBumperWasPressed()) {
//                distance += 2;
//            } else if (gamepad1.leftBumperWasPressed()) {
//                distance -= 2;
//            }

            // hi



            // Hold B = velocity control
            // Hold B = velocity control
            if (gamepad1.left_bumper) {
                target = Shooter.getIdealVelocity(distance);
                robot.shooter.setPollenVelocity(target + offset);

                if (Math.abs(target - pollenVelo) < TOLERANCE_TPS) {
                    pollenGate.setPosition(pollenGATE_OPEN);
                    nectarGate.setPosition(nectarGATE_OPEN);
                    intake.setPower(1);
                    loader.setPower(.7);
                }
            } else {
                robot.shooter.setPollenVelocity(0);
                pollenGate.setPosition(pollenGATE_CLOSED);
                nectarGate.setPosition(nectarGATE_CLOSED);
                intake.setPower(0);
                loader.setPower(0);
            }


            if (gamepad1.dpadUpWasPressed()){
                offset += 50;
            }

            if (gamepad1.dpadDownWasPressed()){
                offset -= 50;
            }

            if (gamepad1.right_bumper) {
                intake.setPower(1);
                loader.setPower(1);
            }




            // ---- Manual gate control ----

            // Hold Y to open both gates.
            // Release Y to close both gates.

//            if (gamepad1.xWasPressed()) {
//                if (!gatePosition) {
//                    gatePosition = true;
//                } else if (gatePosition) {
//                    gatePosition = false;
//
//            }
//
//            if (gatePosition = true) {
//                pollenGate.setPosition(GATE_OPEN);
//                nectarGate.setPosition(GATE_OPEN);
//            } else if (gatePosition = false) {
//                pollenGate.setPosition(GATE_CLOSED);
//                nectarGate.setPosition(GATE_CLOSED);
//                // ---- Mecanum drive ----
//            }



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
            telemetry.addData(" total target", target + offset);
            telemetry.addData("Pollen velocity", pollenShooter.getVelocity());
            telemetry.addData("At speed", atSpeed);
            telemetry.addData("status of gate", gatePosition);
            telemetry.addData("gate position", nectarGate.getPosition());
            telemetry.addData("distance in inches", distance * .0254);
            Pose pose = robot.drivetrain.getPose();
            telemetry.addData("Robot X", "%.1f", pose.getX());
            telemetry.addData("Robot Y", "%.1f", pose.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));

            telemetry.update();
            }
        }

    }
