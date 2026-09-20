package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;

public class RobotActions {

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



    // ── INTAKE ────────────────────────────────────────────────────────────
    public static Action intakeAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.intakeArtifacts(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }


    public static Action startTurretTracking() {
        return new InstantAction(() -> robot.turret.resumeTracking());
    }

    public static Action stowTurret() {
        return new InstantAction(() -> robot.turret.stow());
    }
//
//    public static Action setTurretManual(double leftPosition, double rightPosition) {
//        return new InstantAction(() -> robot.turret.setManual(leftPosition, rightPosition));
//    }
//
//    public static Action waitForTurret() {
//        return new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot());
//    }

    // ── SHOOTER ───────────────────────────────────────────────────────────
    // starts both flywheels with raw power
    public static Action startShooter(double power) {
        return new InstantAction(() -> robot.shooter.setPower(power));
    }

    /**
     * Far-range version — higher target velocity, longer spin-up window.
     */
    public static Action startShooterFar(double power) {
        return startShooter(power);
    }

    // separate power for each turret flywheel
    public static Action setFlywheelPower(double leftPower, double rightPower) {
        // lets auto give each turret flywheel a different raw power
        return new InstantAction(() -> robot.shooter.setPower(leftPower, rightPower));
    }

    public static Action runFlywheels(
            double leftPower,
            double rightPower,
            double timeSeconds
    ) {
        // turns both on waits then shuts both off
        return new SequentialAction(
                setFlywheelPower(leftPower, rightPower),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.shooter.stop())
        );
    }

    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.shooter.stop();
        });
    }
}
