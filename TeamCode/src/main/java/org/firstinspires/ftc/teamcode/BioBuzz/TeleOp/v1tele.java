package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;

@TeleOp(name = "v1tele", group = "Main")
public class v1tele extends LinearOpMode {

    // --- Auto-intake tuning ---
    private static final double TURN_KP = 0.02;           // make negative if it turns away from the ball
    private static final double MAX_TURN = 0.4;
    private static final double CLOSE_AREA = 8.0;         // set from "ball ta" telemetry at pickup distance
    private static final double APPROACH_SPEED = 0.35;
    private static final double APPROACH_DIRECTION = -1;  // 1 = forward, -1 = reverse
    private static final double ALIGN_RANGE_DEG = 15.0;   // ball must be within this many degrees to drive at it

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor pollenShooter = hardwareMap.dcMotor.get("pollenShooter");
        imu = hardwareMap.get(IMU.class, "imu");

        LimelightEx ll = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
        ll.setPipeline(LimelightEx.PIPELINE_APRILTAG);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            // --- mecanum drive inputs ---
            double y = -gamepad1.left_stick_y;    // forward/backward
            double x = gamepad1.left_stick_x;     // left/right strafe
            double rx = gamepad1.right_stick_x;   // rotation

            // --- auto ball tracking ---
            // Left bumper = pollen, right bumper = blue nectar, X = red nectar
            int ballPipeline = -1;
            String trackingName = "none";
            if (gamepad1.left_bumper) {
                ballPipeline = LimelightEx.PIPELINE_POLLEN;
                trackingName = "POLLEN";
            } else if (gamepad1.right_bumper) {
                ballPipeline = LimelightEx.PIPELINE_NECTAR_BLUE;
                trackingName = "BLUE NECTAR";
            } else if (gamepad1.x) {
                ballPipeline = LimelightEx.PIPELINE_NECTAR_RED;
                trackingName = "RED NECTAR";
            }

            boolean autoIntake = false;

            if (ballPipeline != -1) {
                ll.setPipeline(ballPipeline);
                ll.update();
                LLResultTypes.ColorResult ball = ll.getBestBall(ballPipeline);

                if (ball != null) {
                    double tx = ball.getTargetXDegrees();   // + = ball is to the right
                    double ta = ball.getTargetArea();

                    // turn toward the ball
                    rx = Range.clip(TURN_KP * tx, -MAX_TURN, MAX_TURN);
                    x = 0;

                    // drive at the ball only as it gets centered, so turn and drive don't cancel out
                    double alignment = Math.max(0.0, 1.0 - Math.abs(tx) / ALIGN_RANGE_DEG);
                    y = (ta < CLOSE_AREA) ? APPROACH_SPEED * APPROACH_DIRECTION * alignment : 0.0;

                    autoIntake = true;

                    telemetry.addData("ball tx", tx);
                    telemetry.addData("ball ta", ta);
                }
                // no ball seen: driver keeps manual control
            } else {
                ll.setPipeline(LimelightEx.PIPELINE_APRILTAG);
            }

            // --- intake: runs while A is held or auto-tracking a ball ---
            intake.setPower((gamepad1.a || autoIntake) ? 1.0 : 0.0);

            // --- shooter: power follows right trigger ---
            pollenShooter.setPower(gamepad1.right_trigger);

            // --- mecanum math, normalized so no wheel exceeds 1.0 ---
            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))));
            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // --- telemetry ---
            telemetry.addData("Tracking", trackingName);
            telemetry.addData("Target locked", autoIntake);
            telemetry.addData("Balls seen", ll.getBallCount());
            telemetry.addData("Pipeline", ll.getResult() != null ? ll.getResult().getPipelineIndex() : -1);
            telemetry.addData("rx", rx);
            telemetry.addData("y", y);
            telemetry.addData("Shooter power", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}