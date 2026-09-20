package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {

    // one flywheel motor for each turret
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;

    // pid values for later when we start velocity control
    public static double kP = 50.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;

    public Shooter(HardwareMap hardwareMap) {
        // names have to match robot config
        leftFlywheel = new MotorEx(hardwareMap, "leftShooterMotor", Motor.GoBILDA.BARE);
        rightFlywheel = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);

        // flip one of these later if its wheel spins the wrong way
        leftFlywheel.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // float lets the flywheels coast down instead of braking hard
        leftFlywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftFlywheel.resetEncoder();
        rightFlywheel.resetEncoder();

        // velocity controller is saved for later right now we use power
//        updatePIDF();
    }

    // velocity setup for later this is not called right now
    public void updatePIDF() {
        leftFlywheel.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        rightFlywheel.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    // raw power controls used right now
    public void setPower(double power) {
        // same raw power for both flywheels
        setPower(power, power);
    }

    // each turret has its own flywheel
    public void setPower(double leftPower, double rightPower) {
        // separate raw power for each turret flywheel
        leftFlywheel.set(leftPower);
        rightFlywheel.set(rightPower);
    }

    public void setLeftPower(double power) {
        leftFlywheel.set(power);
    }

    public void setRightPower(double power) {
        rightFlywheel.set(power);
    }

    public void shootArtifacts() {
        setPower(1);
    }

    // velocity control for later this is not used by the test
    public void setVelocity(double velocity) {
        leftFlywheel.setVelocity(velocity);
        rightFlywheel.setVelocity(velocity);
    }

    public double getVelocity() {
        // average velocity is only for telemetry right now
        return (leftFlywheel.getVelocity() + rightFlywheel.getVelocity()) / 2.0;
    }

    public double getLeftVelocity() {
        // velocity is telemetry only it does not control the motor yet
        return leftFlywheel.getVelocity();
    }

    public double getRightVelocity() {
        return rightFlywheel.getVelocity();
    }

    public void stop() {
        // shut off both motors
        setPower(0);
    }
}
