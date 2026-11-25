package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class RobotActions {
    public static double SHOOTER_TOLERANCE_TPS = 50;
    public static double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3;
    public static double SHOOTING_INTAKE_POWER = 1;
    public static double SHOOTING_TRANSFER_POWER = 0.7;

    // Intake and transfer

    public static Action startIntake(double power) {
        return startIntake(power, power);
    }

    public static Action startIntake(double intakePower, double transferPower) {
        return new InstantAction(() -> {
            robot.intake.intakeElements(intakePower);
            robot.transfer.loadElements(transferPower);
        });
    }

    public static Action stopIntake() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.transfer.stop();
        });
    }

    public static Action intakeAction(double power, double timeSeconds) {
        return intakeAction(power, power, timeSeconds);
    }

    public static Action intakeAction(double intakePower, double transferPower, double timeSeconds) {
        return new SequentialAction(
                startIntake(intakePower, transferPower),
                new SleepAction(timeSeconds),
                stopIntake()
        );
    }

    public static Action intakeAction(double intakePower, double intakeSeconds,
                                      double transferPower, double transferSeconds) {
        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> robot.intake.intakeElements(intakePower)),
                        new SleepAction(intakeSeconds),
                        new InstantAction(() -> robot.intake.stop())
                ),
                loaderAction(transferPower, transferSeconds)
        );
    }

    public static Action startLoader(double power) {
        return new InstantAction(() -> robot.transfer.loadElements(power));
    }

    public static Action stopLoader() {
        return new InstantAction(() -> robot.transfer.stop());
    }

    public static Action loaderAction(double power, double timeSeconds) {
        return new SequentialAction(
                startLoader(power),
                new SleepAction(timeSeconds),
                stopLoader()
        );
    }

    // Shooter

    public static Action startShooter(double targetVelocity) {
        return new InstantAction(() -> robot.shooter.setPollenVelocity(targetVelocity));
    }

    public static boolean isReadyToShoot() {
        return isReadyToShoot(Shooter.getIdealVelocity(3));
    }

    public static boolean isReadyToShoot(double targetVelocity) {
        return targetVelocity > 0
                && Math.abs(targetVelocity - robot.shooter.getPollenVelocity())
                < SHOOTER_TOLERANCE_TPS;
    }

    public static Action waitForShooter(double targetVelocity) {
        return waitForShooter(targetVelocity, SHOOTER_SPINUP_TIMEOUT_SECONDS);
    }

    public static Action waitForShooter(double targetVelocity, double timeoutSeconds) {
        return new Action() {
            private ElapsedTime timer;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (timer == null) timer = new ElapsedTime();
                packet.put("Shooter target TPS", targetVelocity);
                packet.put("Shooter actual TPS", robot.shooter.getPollenVelocity());
                return !isReadyToShoot(targetVelocity) && timer.seconds() < timeoutSeconds;
            }
        };
    }

    public static Action stopShooter() {
        return new InstantAction(() -> {
            robot.gate.gateMovement(false);
            robot.intake.stop();
            robot.transfer.stop();
            robot.shooter.setPollenVelocity(0);
        });
    }

    private static double shotVelocity = 0;

    public static Action shootAction(double feedSeconds) {
        return new SequentialAction(
                new InstantAction(() -> {
                    shotVelocity = Shooter.getIdealVelocity(3);
                    robot.shooter.setPollenVelocity(shotVelocity);
                }),
                new Actions.RunnableAction(() ->
                        Math.abs(shotVelocity - robot.shooter.getPollenVelocity()) > 100),
                new InstantAction(() -> {
                    robot.pollenServo.pollenGate(true);
                    robot.intake.intakeElements(1);
                    robot.transfer.loadElements(1);
                }),
                new SleepAction(feedSeconds),
                // 4. shut everything off
                new InstantAction(() -> {
                    robot.pollenServo.pollenGate(false);
                    robot.intake.stop();
                    robot.shooter.setPollenVelocity(0);
                })
        );
    }

//    public static Action shootAction(double feedSeconds) {
//        return shootAction(Shooter.getIdealVelocity(3), feedSeconds);
//    }


    public static Action shootAction(double targetVelocity, double feedSeconds) {
        return new Action() {
            private ElapsedTime timer;
            private boolean feeding;
            private boolean finished;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (finished) return false;
                if (timer == null) {
                    timer = new ElapsedTime();
                    robot.gate.gateMovement(false);
                    robot.intake.stop();
                    robot.transfer.stop();
                    robot.shooter.setPollenVelocity(targetVelocity);
                }
                packet.put("Shooter target TPS", targetVelocity);
                packet.put("Shooter actual TPS", robot.shooter.getPollenVelocity());
                packet.put("Shooter ready", isReadyToShoot(targetVelocity));
                packet.put("Gate command", feeding ? "OPEN" : "CLOSED");
                if (!feeding) {
                    if (isReadyToShoot(targetVelocity)) {
                        robot.gate.gateMovement(true);
                        robot.intake.intakeElements(SHOOTING_INTAKE_POWER);
                        robot.transfer.loadElements(SHOOTING_TRANSFER_POWER);
                        packet.put("Gate command", "OPEN");
                        timer.reset();
                        feeding = true;
                    } else if (timer.seconds() >= SHOOTER_SPINUP_TIMEOUT_SECONDS) {
                        stop();
                        return false;
                    }
                    return true;
                }
                if (timer.seconds() >= feedSeconds) {
                    stop();
                    return false;
                }
                return true;
            }

            private void stop() {
                robot.gate.gateMovement(false);
                robot.intake.stop();
                robot.transfer.stop();
                robot.shooter.setPollenVelocity(0);
                finished = true;
            }
        };
    }

    // Gates

    public static Action openGate() {
        return new InstantAction(() -> robot.gate.gateMovement(true));
    }

    public static Action closeGate() {
        return new InstantAction(() -> robot.gate.gateMovement(false));
    }

    public static Action openGateFor(double seconds) {
        return new SequentialAction(openGate(), new SleepAction(seconds), closeGate());
    }

    public static Action openPollenGate() {
        return new InstantAction(() -> robot.gate.pollenGate(true));
    }

    public static Action closePollenGate() {
        return new InstantAction(() -> robot.gate.pollenGate(false));
    }

    public static Action openNectarGate() {
        return new InstantAction(() -> robot.gate.nectarGate(true));
    }

    public static Action closeNectarGate() {
        return new InstantAction(() -> robot.gate.nectarGate(false));
    }

    // Turret

    public static Action centerTurret() {
        return new InstantAction(() -> robot.centerTurret());
    }

    public static Action startTurretTracking() {
        return new InstantAction(() -> robot.startTurretTracking());
    }

    public static Action stowTurret() {
        return new InstantAction(() -> robot.stowTurret());
    }

    public static Action disableTurret() {
        return new InstantAction(() -> robot.disableTurret());
    }

    public static Action waitForTurret() {
        return waitForTurret(2.0);
    }

    public static Action waitForTurret(double timeoutSeconds) {
        return new Action() {
            private ElapsedTime timer;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (timer == null) timer = new ElapsedTime();
                return !robot.turret.isReadyToShoot() && timer.seconds() < timeoutSeconds;
            }
        };
    }

    // Optional Limelight ball actions

    public static double LL_HEIGHT = 10;
    public static double LL_ANGLE = 20;
    private static Pose ballPose;

    public static TrackBallsAction trackBalls(LimelightEx camera, boolean redAlliance,
                                              double timeoutSeconds, BooleanSupplier active) {
        return new TrackBallsAction(camera, robot.drivetrain, robot.intake,
                redAlliance, timeoutSeconds, active);
    }

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