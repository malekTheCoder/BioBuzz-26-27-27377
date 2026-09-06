//package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "ahmedteleop", group = "Main")
//public class simpletele extends LinearOpMode {
//
//    public double opengate = 0.96;
//    public double closegate = 0;
//    private IMU imu;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
//        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
//        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
//        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
//        DcMotorEx shooter1,shooter2;
//        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
//        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");
//
//        Servo gateServo = hardwareMap.servo.get("gateServo");
//        //Servo gateServo2 = hardwareMap.servo.get("gateServo2");
//
//
//        shooter1 = hardwareMap.get(DcMotorEx.class,"rightShooterMotor");
//        shooter2 = hardwareMap.get(DcMotorEx.class,"leftShooterMotor");
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        Limelight3A limelight;
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();
//        gateServo.setPosition(0);
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//
//
//            // --- mecanum Drive Control ---
//            double y = -gamepad1.left_stick_y;    // forward/backward
//            double x = gamepad1.left_stick_x; // left/right strafe with multiplier
//            double rx = gamepad1.right_stick_x;   // rotation
//            double tx = 0, ty = 0;
////            limelight.getTx();
////            limelight.getTy();
//            double currentVelocity = 0;
//            double gatePosition = 0;
//            if(gamepad1.left_trigger > 0.1) {
//
//                int target = calcVelo(ty);
//                gatePosition = gateServo.getPosition();
//                currentVelocity = shooter1.getVelocity();
//                if(currentVelocity < target){
//                    shooter1.setPower(1);//ff
//                    shooter2.setPower(1);
//                }
//                else{
//                    shooter1.setPower(0);
//                    shooter2.setPower(0);
//                }
//                if(Math.abs(currentVelocity - target) < 30) {
//                    //gateServo2.setPosition(opengate);
//                    gateServo.setPosition(opengate);
//                }
//                if(gamepad1.right_trigger > 0.1){
//                    double tolerance = 1;
//                    if(Math.abs(tx) > tolerance){
//                        double kp = 0.01;
//                        double kf = 0.05;
//                        rx = kp * tx + kf * Math.signum(tx);
//                    }
//                    else{
//                        rx = 0;
//                    }
//                    intake.setPower(1);
//                    loader.setPower(1);
//                }
//            }
//            else {
//
//                shooter1.setPower(0);
//                shooter2.setPower(0);
//                //gateServo2.setPosition(closegate);
//                gateServo.setPosition(closegate);
//                if (gamepad1.right_bumper) {
//                    loader.setPower(1);
//                    intake.setPower(1);
//                } else if (gamepad1.left_bumper) {
//                    intake.setPower(-1);
//                    loader.setPower(-1);
//                } else {
//                    loader.setPower(0);
//                    intake.setPower(0);
//                }
//            }
//            double fl = y + x + rx;
//            double bl = y - x + rx;
//            double fr = y - x - rx;
//            double br = y + x - rx;
//            frontLeft.setPower(fl);
//            frontRight.setPower(fr);
//            backLeft.setPower(bl);
//            backRight.setPower(br);
//            telemetry.addData("currentVelocity",currentVelocity);
//            telemetry.addData("gatePosition",gatePosition);
//            telemetry.addData("tyvalue", ty);
//            telemetry.update();
//        }
//
//    }
//    int calcVelo(double ty){
//        if(ty < 1)
//            return 1900;
//        if(ty < 5.5)
//            return 1600;
//        if(ty < 10)
//            return 1500;
//        return 1400;
//    }
//
//
//}
