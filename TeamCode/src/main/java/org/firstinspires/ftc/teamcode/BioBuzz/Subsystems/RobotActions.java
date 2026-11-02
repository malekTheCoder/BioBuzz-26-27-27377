package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import java.util.function.BooleanSupplier;

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
                new InstantAction(() -> robot.intake.intakeElements(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }
    public static TrackBallsAction trackBalls(LimelightEx camera, boolean redAlliance,
                                               double timeoutSeconds, BooleanSupplier active) {
        return new TrackBallsAction(camera, robot.drivetrain, robot.intake,
                redAlliance, timeoutSeconds, active);
    }
    // --- GATE

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
        return new InstantAction(() -> robot.shooter.setVelo());
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

    public static Action waitForShooter() {
        return new Actions.RunnableAction(() -> !robot.shooter.atSpeed());
    }

    /**
     * Far-range version — higher target velocity, longer spin-up window.
     */
    public static Action startShooterFar(double power) {
        return startShooter(power);
    }

    // separate power for each turret flywheel
//    public static Action setFlywheelPower(double leftPower, double rightPower) {
//        // lets auto give each turret flywheel a different raw power
//        return new InstantAction(() -> robot.shooter.setPower(leftPower, rightPower));
//    }

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
            robot.turret.stow();
        });
    }
    // ── BALL PICKUP (Limelight) ─────────────────────────────────────────
    public static double LL_HEIGHT = 10;  // lens height minus ball-center height (in) ← MEASURE
    public static double LL_ANGLE  = 20;  // how far the LL tilts down (deg)          ← MEASURE
    private static Pose ballPose = null;  // shared between the ball actions

    private static Pose findBall(LimelightEx camera, int pipeline) {
        camera.update();
        LLResultTypes.ColorResult seen = camera.getBestBall(pipeline);
        if (seen == null) return null;
        double downAngle = Math.toRadians(LL_ANGLE - seen.getTargetYDegrees());
        if (downAngle <= 0) return null;
        Pose r = robot.drivetrain.getPose();
        double dist = LL_HEIGHT / Math.tan(downAngle);
        double ang  = r.getHeading() - Math.toRadians(seen.getTargetXDegrees());
        return new Pose(r.getX() + dist * Math.cos(ang),
                r.getY() + dist * Math.sin(ang),
                ang + Math.PI);
    }

    public static Action scanForBall(LimelightEx camera, int pipeline, double timeoutSec) {
        return new Action() {
            ElapsedTime timer = null;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (timer == null) {
                    timer = new ElapsedTime();
                    ballPose = null;
                    camera.setPipeline(pipeline);
                }
                ballPose = findBall(camera, pipeline);
                return ballPose == null && timer.seconds() < timeoutSec;
            }
        };
    }

    public static Action driveToBall(LimelightEx camera, int pipeline) {
        return new Action() {
            Pose target = null;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (target == null) {
                    if (ballPose == null) return false;
                    target = ballPose;
                    follow(target);
                }
                Pose seen = findBall(camera, pipeline);
                if (seen != null && Math.hypot(seen.getX() - target.getX(),
                        seen.getY() - target.getY()) > 2) {
                    target = seen;
                    ballPose = seen;
                    follow(target);
                }
                return robot.drivetrain.isBusy();
            }

            private void follow(Pose t) {
                Follower f = robot.drivetrain;
                f.followPath(f.pathBuilder()
                        .addPath(new BezierLine(f.getPose(), t))
                        .setLinearHeadingInterpolation(f.getHeading(), t.getHeading())
                        .build(), true);
            }
        };
    }

    public static Action driveToShoot(LimelightEx camera, Pose shootPose) {
        return new Action() {
            FollowPathAction path = null;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (path == null) {
                    camera.setPipeline(LimelightEx.PIPELINE_APRILTAG);
                    Follower f = robot.drivetrain;
                    path = new FollowPathAction(f, f.pathBuilder()
                            .addPath(new BezierLine(f.getPose(), shootPose))
                            .setLinearHeadingInterpolation(f.getHeading(), shootPose.getHeading())
                            .build(), true);
                }
                return path.run(packet);
            }
        };
    }
}
