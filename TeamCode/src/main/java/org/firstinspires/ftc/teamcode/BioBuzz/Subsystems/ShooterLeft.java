package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterLeft {

    private final MotorEx leftShooter;

    public static double kP = 50.0; // tune
    public static double kI = 5.0;
    public static double kD = 0.2;
    public static double kF = 12.0;

    public static double LAUNCH_ANGLE = 45; // check this
    public static double LAUNCH_HEIGHT = 0.254;
    public static double GOAL_HEIGHT = 1.372;
    public static double WHEEL_RADIUS = 0.048; // check this
    public static double TRANSFER_RATIO = 0.45; // calculate this when testing
    public static double TICKS_PER_REV = 28;

    public ShooterLeft(HardwareMap hardwareMap) {
        leftShooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);

        leftShooter.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftShooter.resetEncoder();

        leftShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();
    }

    public void updatePIDF() {
        leftShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
}

    public void setLeftVelocity(double velo) {
        leftShooter.motorEx.setVelocity(velo);
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public static double calcVelocity(double distance) { // d= horizantal distance from goal
        double theta = Math.toRadians(LAUNCH_ANGLE);
        double deltaH = GOAL_HEIGHT - LAUNCH_HEIGHT;
        double cos = Math.cos(theta);

        double denom = 2 * cos * cos * (distance * Math.tan(theta) - deltaH);
        if (denom <= 0) return -1;                    // too close to reach

        double velo = distance * Math.sqrt(9.81 / denom);    // ball exit speed, m/s
        double surface = velo / TRANSFER_RATIO;                  // wheel surface speed, m/s
        double rps = surface / (2 * Math.PI * WHEEL_RADIUS);
        return rps * TICKS_PER_REV;                   // ticks/sec
    }
    public void setVelo() {
        // Use the same alliance and field-half goal as the turrets.
        double distance = robot.turret.getDistance() * 0.0254;
        double ticks = calcVelocity(distance);
        if (!Double.isFinite(ticks) || ticks < 0) {
            stop();
            return;
        }

        lastTarget = ticks; // atSpeed needs the speed we're actually asking for
        setLeftVelocity(ticks);
    }

    private double lastTarget = -1; // to check if it's on
    public static double SPEED_TOLERANCE_RPM = 50; // can change

    public boolean atSpeed() {
        if (lastTarget <= 0) return false;
        double tol = SPEED_TOLERANCE_RPM * TICKS_PER_REV / 60.0;   // RPM → ticks/sec
        return Math.abs(getLeftVelocity()  - lastTarget) <= tol;
    }
    public boolean enabled = false;

    public void update() {
        if (enabled) {
            updatePIDF();
            setVelo();
        } else {
            stop();
        }
    }

    public void stop() {
        enabled = false;
        lastTarget = -1;
        leftShooter.set(0);
    }

    // Independent trigger power for the hardware test; no distance calculation.
    public void setPower(double left) {
        enabled = false;
        lastTarget = -1;
        leftShooter.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.motorEx.setPower(left);
    }
}
