package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotActions {

    // ── Goal position for distance calculations ────────────────────────────
    private static Pose goal = new Pose(136, 136);

    public static void setGoal(Pose goalPose) {
        goal = goalPose;
    }

    // ── Distance / velocity constants ──────────────────────────────────────
    public static double MIN_DISTANCE           = 20.0;
    public static double MID_DISTANCE           = 60.0;
    public static double MAX_DISTANCE           = 180.0;
    public static double SHOOTER_VELOCITY_CLOSE = 1900;
    public static double SHOOTER_VELOCITY_MID   = 2400;
    public static double SHOOTER_VELOCITY_FAR   = 2600;
    public static double targetVelo = 1500;

    public static double MIN_DISTANCEFAR          = 20.0;
    public static double MID_DISTANCEFAR          = 60.0;
    public static double MAX_DISTANCEFAR          = 180.0;
    public static double SHOOTER_VELOCITY_CLOSEFAR = 2150;
    public static double SHOOTER_VELOCITY_MIDFAR   = 2200;
    public static double SHOOTER_VELOCITY_FARFAR   = 2300;

    // ── Helper: current distance to goal (drivetrain pose) ─────────────────
    public static double getDistanceToGoal() {
        double robotX = robot.drivetrain.getPose().getX();
        double robotY = robot.drivetrain.getPose().getY();
        double dx = goal.getX() - robotX;
        double dy = goal.getY() - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Velocity mapping ────────────────────────────────────────────────────
    public static double distanceToShooterVelocity(double distance) {
        if (distance <= MID_DISTANCE) {
            double t = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_CLOSE + t * (SHOOTER_VELOCITY_MID - SHOOTER_VELOCITY_CLOSE);
        } else {
            double t = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_MID + t * (SHOOTER_VELOCITY_FAR - SHOOTER_VELOCITY_MID);
        }
    }

    public static double distanceToShooterVelocityFar(double distance) {
        if (distance <= MID_DISTANCEFAR) {
            double t = (distance - MIN_DISTANCEFAR) / (MID_DISTANCEFAR - MIN_DISTANCEFAR);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_CLOSEFAR + t * (SHOOTER_VELOCITY_MIDFAR - SHOOTER_VELOCITY_CLOSEFAR);
        } else {
            double t = (distance - MID_DISTANCEFAR) / (MAX_DISTANCEFAR - MID_DISTANCEFAR);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_MIDFAR + t * (SHOOTER_VELOCITY_FARFAR - SHOOTER_VELOCITY_MIDFAR);
        }
    }

    // ── INTAKE ────────────────────────────────────────────────────────────
    public static Action intakeAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.intakeArtifacts(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }

    // ── SHOOTER ───────────────────────────────────────────────────────────
    /**
     * Spin up shooter to close-range target velocity, running intake once at speed.
     * Blocking loop for up to 3 seconds.
     */
    public static Action startShootertype(double timeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        int target = 1540;
        while (timer.seconds() < 3) {
            robot.bulkReader.bulkRead();
            double currentVelo = Math.abs(robot.shooter.getVelocity());

            if (currentVelo < target) {
                robot.shooter.setVelocity(target);
            } else {
                robot.shooter.stop();
            }

            Common.dashTelemetry.addData("currentVelo", currentVelo);
            Common.dashTelemetry.update();

            if (Math.abs(currentVelo - target) < 30) {
                robot.intake.intakeArtifacts(1);
            }
        }
        robot.shooter.stop();
        return new SequentialAction();
    }

    /**
     * Far-range version — higher target velocity, longer spin-up window.
     */
    public static Action startShooterFar(double timeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        int target = 2300;
        while (timer.seconds() < 4) {
            robot.bulkReader.bulkRead();
            double currentVelo = Math.abs(robot.shooter.getVelocity());

            if (currentVelo < target) {
                robot.shooter.setVelocity(target);
            } else {
                robot.shooter.stop();
            }

            Common.dashTelemetry.addData("currentVelo", currentVelo);
            Common.dashTelemetry.update();

            if (Math.abs(currentVelo - target) < 30) {
                robot.intake.intakeArtifacts(1);
            }
        }
        robot.shooter.stop();
        return new SequentialAction();
    }

    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.shooter.stop();
        });
    }
}