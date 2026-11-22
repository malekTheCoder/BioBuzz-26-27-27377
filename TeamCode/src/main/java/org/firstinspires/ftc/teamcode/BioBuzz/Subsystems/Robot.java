package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class Robot {

    public final Follower drivetrain;
    public final Shooter shooter;

    public final Gate pollenServo;
    public final Gate nectarServo;

    public final Intake intake;
    public final Transfer transfer;
    public final Turret turret;

    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;
    private boolean turretCommandInitialized;

    public Robot(HardwareMap hardwareMap) {


        bulkReader = new BulkReader(hardwareMap);
        drivetrain = Constants.createFollower(hardwareMap);
        actionScheduler = new ActionScheduler();

        transfer = new Transfer(hardwareMap);

        intake = new Intake(hardwareMap); // 2 intake wheel

        pollenServo = new Gate(hardwareMap);
        nectarServo = new Gate(hardwareMap);

        shooter = new Shooter(hardwareMap); // shooter wheel (2 motors)
        turret = new Turret(hardwareMap, drivetrain);
    }

    public void centerTurret() {
        turret.centerLeftForward();
        turretCommandInitialized = true;
    }

    public void startTurretTracking() {
        if (!turretCommandInitialized) centerTurret();
        turret.resumeTracking();
    }

    public void stowTurret() {
        turret.stow();
    }

    public void disableTurret() {
        turret.disablePwm();
        turretCommandInitialized = false;
    }

    public void run() {
        bulkReader.bulkRead();
        drivetrain.update();
        turret.run();
        actionScheduler.run();
        //shooter.update(); // enabled distance tracking must run every loop
    }

    public void printTelemetry() {
        turret.printTelemetry();
    }

}

