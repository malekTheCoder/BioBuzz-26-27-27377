package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class Robot {

    public final Follower drivetrain;
    public final Shooter shooter;
    public final Intake intake;

    public final Turret turret;

    public final Gate gateServo;

    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;

    public Robot(HardwareMap hardwareMap) {


        bulkReader = new BulkReader(hardwareMap);
        drivetrain = Constants.createFollower(hardwareMap);
        actionScheduler = new ActionScheduler();


        intake = new Intake(hardwareMap); // 2 intake wheel

        shooter = new Shooter(hardwareMap); // gotta change this for shooter thing

        gateServo = new Gate(hardwareMap);
        turret = new Turret(hardwareMap, drivetrain);
        turret.stow();
    }

    public void run() {
        bulkReader.bulkRead();
        drivetrain.update();
        turret.run();
        shooter.update();
        actionScheduler.run();
    }

    public void printTelemetry() {
        turret.printTelemetry();
    }


}
