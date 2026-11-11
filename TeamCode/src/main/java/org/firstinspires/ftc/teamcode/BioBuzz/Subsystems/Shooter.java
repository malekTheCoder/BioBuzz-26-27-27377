package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;
import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Shooter {
    private MotorEx nectarShooter;
    private MotorEx pollenShooter;
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
        nectarShooter = new MotorEx(hardwareMap, "leftShooterMotor", Motor.GoBILDA.BARE);
        pollenShooter = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);

        nectarShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (pollenShooter != null) pollenShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        nectarShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        pollenShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);


        nectarShooter.resetEncoder();
        pollenShooter.resetEncoder();
// Set velocity PIDF on both motors
        nectarShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updatePIDF();
    }
    // Called from TeleOp every loop so dashboard changes apply live
    private double lastP = Double.NaN, lastI, lastD, lastF;
    // CHANGE: remembers the PIDF values last sent to the motors.

    // Called from TeleOp every loop so dashboard changes apply live
    public void updatePIDF() {
        if (kP == lastP && kI == lastI && kD == lastD && kF == lastF) return;
        // CHANGE: skips the slow hardware write when nothing changed. It still sends on the
        // first call and whenever you edit kP/kI/kD/kF in Dashboard.
        nectarShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        pollenShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        // CHANGE: removed the if-check on pollenShooter, and put nectar first then pollen, matching the constructor.
        lastP = kP; lastI = kI; lastD = kD; lastF = kF;
        // CHANGE: records what was just sent.
    }

    // Raw power — used in auto
// Variable velocity — used in TeleOp with distance-based control
    public void setPollenVelocity(double velo) {
        pollenShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.motorEx.setVelocity(velo);
    }
    public void setNectarVelocity(double velo) {
        nectarShooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarShooter.motorEx.setVelocity(velo);
    }
    public double getNectarVelocity() {
        return nectarShooter.getVelocity();
    }
    public double getPollenVelocity() {
        return pollenShooter.getVelocity();
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
        if (!Double.isFinite(ticks) || ticks < 0) { stop(); return; }

        lastTarget = ticks; // atSpeed needs the speed we're actually asking for
        setNectarVelocity(ticks);
        setPollenVelocity(ticks);
    }


    private double lastTarget = -1; // to check if its on
    public static double SPEED_TOLERANCE_RPM = 100; // can change

    public boolean atSpeed() {
        if (lastTarget <= 0) return false;
        double tol = SPEED_TOLERANCE_RPM * TICKS_PER_REV / 60.0;   // RPM → ticks/sec
        return Math.abs(getNectarVelocity()  - lastTarget) <= tol
                && (pollenShooter == null || Math.abs(getPollenVelocity() - lastTarget) <= tol);
    }
    public boolean enabled = false;

    public void update() {
        if (enabled) {
            updatePIDF();
            setVelo();
        }
    }

    public void stop() {
        enabled = false;
        lastTarget = -1;
        nectarShooter.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenShooter.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenShooter.set(0);
        nectarShooter.set(0);
    }

    // Independent trigger power for the hardware test; no distance calculation.
//    public void setPower(double nectar, double pollen) {
//        enabled = false;
//        lastTarget = -1;
//        nectarShooter.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pollenShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        nectarShooter.motorEx.setPower(nectar);
//        pollenShooter.motorEx.setPower(nectar);
//    }
}