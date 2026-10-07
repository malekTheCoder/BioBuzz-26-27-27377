package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public final class Robot {

    public final Follower drivetrain;
    public final Shooter shooter;

    public final Intake intake;
    public final Gate gate;
    public final Turret turret;
    public final LimelightEx limelight;


    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;

    public Robot(HardwareMap hardwareMap) {


        bulkReader = new BulkReader(hardwareMap);
        drivetrain = Constants.createFollower(hardwareMap);
        actionScheduler = new ActionScheduler();

        gate = new Gate(hardwareMap);

        Limelight3A limelightDevice = hardwareMap.get(Limelight3A.class, "limelight");
        limelightDevice.pipelineSwitch(9);
        limelight = new LimelightEx(limelightDevice);

        intake = new Intake(hardwareMap); // 2 intake wheel

        shooter = new Shooter(hardwareMap); // shooter wheel (2 motors)

        turret = new Turret(hardwareMap); // CR servo + axon encoder
    }

    // Hardware updates only. Used by auto, where runBlocking() already runs the actions
    public void updateHardware() {
        bulkReader.bulkRead();
        limelight.update();
        drivetrain.update();
        turret.update();
    }

    // Hardware updates + one step of the scheduled actions (for tele-op)
    public void run() {
        updateHardware();
        actionScheduler.run();
    }

    public void printTelemetry() {
        Common.dashTelemetry.addData("Sees goal?", limelight.hasResult());
        if (limelight.hasResult()) {
            Common.dashTelemetry.addData("tx", "%.1f", limelight.getResult().getTx());
        }
        double goalDistance = RobotActions.getLimelightDistanceToGoal();
        Common.dashTelemetry.addData("Goal distance (in)", "%.1f", goalDistance);
        Common.dashTelemetry.addData("Auto RPM target", "%.0f", RobotActions.distanceToShooterVelocity(goalDistance));
        Common.dashTelemetry.addData("Turret angle", "%.1f", turret.getAngle());
        Common.dashTelemetry.addData("Turret target", "%.1f", turret.getTargetAngle());
        Common.dashTelemetry.update();
    }
}