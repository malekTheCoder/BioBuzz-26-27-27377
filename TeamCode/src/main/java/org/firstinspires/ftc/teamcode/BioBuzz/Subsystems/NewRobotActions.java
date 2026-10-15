package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;

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



//───── TURRETTRACKING ────────────────
    public static Action startTurretTracking() {
        return new InstantAction(() -> robot.turret.resumeTracking());
    }

    public static Action stowTurret() {
        return new InstantAction(() -> robot.turret.stow());
    }



    public static Action enableShooter(double power){

        return new InstantAction(() -> robot.shooter.enabled = true);

    }

    public static Action stopShooter(){

        return new InstantAction(() -> robot.shooter.enabled = false);
    }


    public static Action waitForShooter() {
        return new Actions.RunnableAction(() -> !robot.shooter.atSpeed());
    }

    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.shooter.stop();
            robot.turret.stow();
        });
    }
}
