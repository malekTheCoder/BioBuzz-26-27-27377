package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class NazShooter {

    // declarations can live in the class body
    // instructions that DO something must be inside a method or constructor
    private DcMotorEx pollenShooter;
    private DcMotorEx nectarShooter;

    // must be public static for dashboard to edit it live
    public static double TARGET_TPS = 1800;  // units are ticks PER SECOND (speed) not a distance.
    public static double SPEED_TOLERANCE = 100; // how far off the target (in ticks/sec) still counts as ready


    public NazShooter(HardwareMap hardwareMap) { // type is HardwareMap (capital), name is hardwareMap (lowercase)
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarShooter = hardwareMap.get(DcMotorEx.class, "nectarShooter");
        nectarShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void spinMotor() {   // the motor holds this velocity until a new command replaces it
        pollenShooter.setVelocity(TARGET_TPS);
        nectarShooter.setVelocity(TARGET_TPS);
    }


    public void stopMotor() {   // must be called explicitly, releasing a button doesn't stop the motor
        pollenShooter.setVelocity(0);
        nectarShooter.setVelocity(0);
    }

    public double getPollenVelo() {
        return pollenShooter.getVelocity();
    }
    public double getNectarVelo() {
        return nectarShooter.getVelocity();
    }

    public boolean atSpeed (){
        return
                (Math.abs(getPollenVelo()-TARGET_TPS) <= SPEED_TOLERANCE) &&
                        (Math.abs(getNectarVelo()-TARGET_TPS) <=SPEED_TOLERANCE);
    }
}