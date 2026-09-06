//package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//public final class Robot {
//
//    public final Follower drivetrain;
//    public final Loader loader;
//    public final Shooter shooter;
//    public final Intake intake;
//    public final HoodServo hoodServo;
//    public final GateServo gateServo;
//    public final LimelightEx limelight;
//
//    public final BulkReader bulkReader;
//    public final ActionScheduler actionScheduler;
//
//    public Robot(HardwareMap hardwareMap) {
//
//        bulkReader = new BulkReader(hardwareMap);
//        drivetrain = Constants.createFollower(hardwareMap);
//        actionScheduler = new ActionScheduler();
//
//        loader = new Loader(hardwareMap);
//        intake = new Intake(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//
//        hoodServo = new HoodServo();
//        hoodServo.init(hardwareMap);
//
//        gateServo = new GateServo();
//        gateServo.init(hardwareMap);
//
//        limelight = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
//    }
//
//    public void run() {
//        bulkReader.bulkRead();
//        drivetrain.update();
//       // shooter.run();        // re-commands target velocity every loop — fixes instability
//        actionScheduler.run();
//    }
//
//    public void printTelemetry() {
//        Common.dashTelemetry.addData("Shooter Velocity", shooter.getVelo());
//      //  Common.dashTelemetry.addData("Shooter Target", shooter.getTargetVelocity());
//        Common.dashTelemetry.update();
//    }
//}