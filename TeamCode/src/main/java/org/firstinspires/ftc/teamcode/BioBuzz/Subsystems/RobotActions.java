package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;

public class RobotActions {

    // ── Goal position for distance calculations ────────────────────────────

    private static Pose goalOverride = null;

    // ── Helper: current distance to goal (drivetrain pose) ─────────────────
    public static double getDistanceToGoal() {
        Pose robotPose = robot.drivetrain.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // same goal the turret uses: your alliance + whichever half of the field you're on
        Pose goal = (goalOverride != null)
                ? goalOverride
                : Common.getGoalForAlliance(Common.isRed, robotY >= Common.FIELD_MIDLINE_Y);

        double dx = goal.getX() - robotX;
        double dy = goal.getY() - robotY;
        return Math.sqrt(dx * dx + dy * dy);   // inches
    }



    // ── INTAKE ────────────────────────────────────────────────────────────
    public static Action intakeAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.intakeElements(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }

    // ── GATE ──────────────────────────────────────────────────────────────
    // ── GATE ──────────────────────────────────────────────────────────────
    public static Action openGate() {return new InstantAction(() -> robot.gateServo.gateMovement(true));
    }
    public static Action openGateFor(double seconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.gateServo.gateMovement(true)),    // open
                new SleepAction(seconds),                                       // wait (non-blocking)
                new InstantAction(() -> robot.gateServo.gateMovement(false))    // close
        );
    }
    public static Action closeGate() {
        return new InstantAction(() -> robot.gateServo.gateMovement(false));
    }
    public static Action startTurretTracking() {
        return new InstantAction(() -> robot.turret.resumeTracking());
    }

    public static Action stowTurret() {
        return new InstantAction(() -> robot.turret.stow());
    }

    public static Action setTurretManual(double leftPosition, double rightPosition) {
        return new InstantAction(() -> robot.turret.setManual(leftPosition, rightPosition));
    }


    public static Action waitForTurret() {
        return new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot());
    }
    public static Action waitForShooter() {
        return new Actions.RunnableAction(() -> !robot.shooter.atSpeed());
    }

    // ── SHOOTER ───────────────────────────────────────────────────────────
    // starts both flywheels with raw power
    public static Action startShooter() {
        return new InstantAction(() -> robot.shooter.spinForCurrentDistance());
    }
    public static Action enableShooter() {
        return new InstantAction(() -> robot.shooter.enabled = true);
    }

    public static Action disableShooter() {
        return new InstantAction(() -> {
            robot.shooter.enabled = false;
            robot.shooter.stop();
        });
    }
    /**
     * Far-range version — higher target velocity, longer spin-up window.
     */

    // separate power for each turret flywheel
//    public static Action setFlywheelPower(double leftPower, double rightPower) {
//        // lets auto give each turret flywheel a different raw power
//        return new InstantAction(() -> robot.shooter.setPower(leftPower, rightPower));
//    }
//
//    public static Action runFlywheels(
//            double leftPower,
//            double rightPower,
//            double timeSeconds
//    ) {
//        // turns both on waits then shuts both off
//        return new SequentialAction(
//                setFlywheelPower(leftPower, rightPower),
//                new SleepAction(timeSeconds),
//                new InstantAction(() -> robot.shooter.stop())
//        );
//    }

    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.shooter.stop();
        });
    }
}
