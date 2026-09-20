package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Shooter {
    private MotorEx leftShooter;
    private MotorEx rightShooter;
    // Tune these in FTC Dashboard
    public static double kP = 50.0; // tune
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;

    public static double LAUNCH_ANGLE = 45; // check this
    public static double LAUNCH_HEIGHT = 0.254;
    public static double GOAL_HEIGHT = 1.372;
    public static double WHEEL_RADIUS = 0.048; // check this
    public static double TRANSFER_RATIO = 0.45; // calculate this when testing
    public static double TICKS_PER_REV = 28;
    public Shooter(HardwareMap hardwareMap) {
        leftShooter = new MotorEx(hardwareMap, "leftShooterMotor", Motor.GoBILDA.BARE);
        rightShooter = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);

        leftShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftShooter.resetEncoder();
        rightShooter.resetEncoder();
// Set velocity PIDF on both motors
        leftShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updatePIDF();
    }
    // Called from TeleOp every loop so dashboard changes apply live
    public void updatePIDF() {
        rightShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        leftShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
    }
    // Raw power — used in auto
// Variable velocity — used in TeleOp with distance-based control
    public void setRightVelocity(double velo) {
        rightShooter.setVelocity(velo);
    }
    public void setLeftVelocity(double velo) {
        leftShooter.setVelocity(velo);
    }
    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }
    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }
    public static double calcVelocity(double d) { // d= horizantal distance from goal
        double theta = Math.toRadians(LAUNCH_ANGLE);
        double deltaH = GOAL_HEIGHT - LAUNCH_HEIGHT;
        double cos = Math.cos(theta);

        double denom = 2 * cos * cos * (d * Math.tan(theta) - deltaH);
        if (denom <= 0) return -1;                    // too close to reach

        double velo = d * Math.sqrt(9.81 / denom);    // ball exit speed, m/s
        double surface = velo / TRANSFER_RATIO;                  // wheel surface speed, m/s
        double rps = surface / (2 * Math.PI * WHEEL_RADIUS);
        return rps * TICKS_PER_REV;                   // ticks/sec
    }
    public void stop() {
        rightShooter.set(0);
        leftShooter.set(0);
    }
}