package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;

@TeleOp
public class RehanTeleOp extends OpMode {

    private Limelight3A limelightDevice;
    LimelightEx limelightClass;

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;

    private int goalRPM = 1300;

    private GoBildaPinpointDriver odometry;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelightDevice = hardwareMap.get(Limelight3A.class, "limelight");
        limelightDevice.pipelineSwitch(9);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odometry.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();

        odometry.setPosition(new Pose2D(DistanceUnit.MM, GlobalPoseStorage.globalX, GlobalPoseStorage.globalY, AngleUnit.RADIANS, GlobalPoseStorage.globalHeading));

        limelightClass = new LimelightEx(limelightDevice);

        telemetry.addLine("Initialized");
    }

    @Override
    public void start() {
        odometry.update();
    }

    @Override
    public void loop() {

        odometry.update();

        Pose2D pose = odometry.getPosition();

        double x = odometry.getPosition().getX(DistanceUnit.INCH);
        double y = odometry.getPosition().getY(DistanceUnit.INCH);
        double heading = odometry.getPosition().getHeading(AngleUnit.DEGREES);

        boolean hasTarget = limelightDevice.getLatestResult().isValid();

        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(turn), 1.0);

        double frontLeftPower  = (leftStickY + leftStickX + turn) / denominator;
        double backLeftPower   = (leftStickY - leftStickX + turn) / denominator;
        double frontRightPower = (leftStickY - leftStickX - turn) / denominator;
        double backRightPower  = (leftStickY + leftStickX - turn) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);


        if (gamepad1.left_trigger >= 0.2) {
            intakeMotor.setVelocity(500);
        } else {
            intakeMotor.setVelocity(0);
        }

        if (shooterMotor.getVelocity() < goalRPM) {
            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0);
        }

        if (limelightClass.hasResult()) {
            limelightClass.getPoseEstimate(heading);
        }

        telemetry.addData("X", "%.2f in", x);
        telemetry.addData("Y", "%.2f in", y);
        telemetry.addData("Heading", "%.2f°", heading);
        telemetry.addData("Limelight", hasTarget ? "HAS TARGET" : "NO TARGET");
        telemetry.addData("Shooter RPM", "%.0f / %d", shooterMotor.getVelocity(), goalRPM);
    }
}