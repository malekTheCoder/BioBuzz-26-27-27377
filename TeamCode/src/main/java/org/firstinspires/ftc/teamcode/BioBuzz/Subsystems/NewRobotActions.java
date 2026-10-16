package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;

import java.util.concurrent.Callable;

public class NewRobotActions {

    // ── Goal position for distance calculations ────────────────────────────
    private static Pose goal = new Pose(136, 136);

    public static void setGoal(Pose goalPose) {
        goal = goalPose;
    }



    // ── Helper: current distance to goal (drivetrain pose) ─────────────────
    public static double getDistanceToGoal() {
        double robotX = robot.drivetrain.getPose().getX();
        double robotY = robot.drivetrain.getPose().getY();
        double dx = goal.getX() - robotX;
        double dy = goal.getY() - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Helper: wait until something is true, but give up after timeoutSeconds
    // so auto never gets stuck if a sensor never says "ready"
    public static Action waitUntil(Callable<Boolean> done, double timeoutSeconds) {
        return new Action() {
            private long startNanos = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startNanos == 0) startNanos = System.nanoTime();
                if ((System.nanoTime() - startNanos) / 1e9 >= timeoutSeconds) return false;
                try {
                    return !done.call(); // true = keep waiting
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
        };
    }



//───── INTAKE ────────────────
    public static Action startIntake(double power) {
        return new InstantAction(() -> robot.intake.intakeElements(power));
    }

    public static Action stopIntake() {
        return new InstantAction(() -> robot.intake.stop());
    }

    // runs the intake for a set time then stops it
    public static Action intakeFor(double power, double seconds) {
        return new SequentialAction(
                startIntake(power),
                new SleepAction(seconds),
                stopIntake()
        );
    }



//───── GATE ────────────────
    public static Action openGate() {
        return new InstantAction(() -> robot.gateServo.gateMovement(true));
    }

    public static Action closeGate() {
        return new InstantAction(() -> robot.gateServo.gateMovement(false));
    }

    // opens the gate, waits, then closes it
    public static Action openGateFor(double seconds) {
        return new SequentialAction(
                openGate(),
                new SleepAction(seconds),
                closeGate()
        );
    }



//───── TURRETTRACKING ────────────────
    public static Action startTurretTracking() {
        return new InstantAction(() -> robot.turret.resumeTracking());
    }

    public static Action stowTurret() {
        return new InstantAction(() -> robot.turret.stow());
    }

    public static Action waitForTurret(double timeoutSeconds) {
        return waitUntil(() -> robot.turret.isReadyToShoot(), timeoutSeconds);
    }



//───── SHOOTER ────────────────
    // shooter speed is set from distance to goal every loop while enabled
    public static Action enableShooter() {
        return new InstantAction(() -> robot.shooter.enabled = true);
    }

    public static Action stopShooter() {
        return new InstantAction(() -> robot.shooter.stop()); // also cuts motor power
    }

    public static Action waitForShooter(double timeoutSeconds) {
        return waitUntil(() -> robot.shooter.atSpeed(), timeoutSeconds);
    }



//───── SHOOT ────────────────
    // waits for flywheels + turret (max 3s), then opens the gate to fire
    public static Action shoot(double gateSeconds) {
        return new SequentialAction(
                new ParallelAction(
                        waitForShooter(3),
                        waitForTurret(3)
                ),
                openGateFor(gateSeconds)
        );
    }



    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.shooter.stop();
            robot.gateServo.gateMovement(false);
            robot.turret.stow();
        });
    }
}
