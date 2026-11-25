package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;

@Autonomous(name = "SimplePathAuto")
public class SimplePathAuto extends AbstractAuto {
    private static final double SHOT_VELOCITY = Shooter.getIdealVelocity(3);
    private Follower f;
    private Paths path;

    @Override
    protected Pose getStartPose() {
        return Paths.P_SIMPLE_START.withHeading(Paths.H_90);
    }

    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new Paths(f);

        if (Common.isRed != Paths.isPathRed) {
            Paths.isPathRed = !Paths.isPathRed;
            path.mirrorAll();
        }

        path.simpleBuild();
    }

    @Override
    protected void onRun() {
        shootPreload();
        intake();
        shootsecond();

    }
    private void shootPreload() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        RobotActions.startTurretTracking(),
                        RobotActions.startShooter(SHOT_VELOCITY),
                        //new FollowPathAction(f,path.simplePath1,true),
                        new ParallelAction(
                                RobotActions.waitForTurret(),
                                RobotActions.waitForShooter(SHOT_VELOCITY)
                        ),
                        RobotActions.shootAction(SHOT_VELOCITY,1),
                        RobotActions.stopShooter()
                )


        );
        robot.actionScheduler.runBlocking();
    }

    private void intake() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                    new ParallelAction(
                            new InstantAction(()-> f.setMaxPower(0.7)),
                            new Actions.CallbackAction(RobotActions.intakeAction(1,1,4), path.simplePath2,01, 0, f, "intake" ),
                            new FollowPathAction(f,path.simplePath2,true)

)
                )

        );
        robot.actionScheduler.runBlocking();
    }
    private void shootsecond() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        RobotActions.startTurretTracking(),
                        RobotActions.startShooter(SHOT_VELOCITY),
                        new InstantAction(()-> f.setMaxPower(1)),
                        //new FollowPathAction(f,path.simplePath3,true),
                        new ParallelAction(
                                RobotActions.waitForTurret(),
                                RobotActions.waitForShooter(SHOT_VELOCITY)
                        ),
                        RobotActions.shootAction(SHOT_VELOCITY,3),
                        RobotActions.stopShooter()

                )

        );
        robot.actionScheduler.runBlocking();
    }
}

/*


    private void followPaths() {
        f.setMaxPower(0.7);
//        LimelightEx camera = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
//        camera.setPipeline(LimelightEx.PIPELINE_APRILTAG);
        //TrackBallsAction tracking = RobotActions.trackBalls(camera, Common.isRed,
                //6.0, this::opModeIsActive);
        try {
            robot.actionScheduler.addAction(
                new SequentialAction(
                        new FollowPathAction(f, path.simplePath1, true),
                        new SleepAction(3.0),
                        new FollowPathAction(f, path.simplePath2, true),
                        new FollowPathAction(f, path.simplePath3, true),
                        //tracking,
                        returnToPoint3()
                )
            );
            robot.actionScheduler.runBlocking();
        } finally {
            //tracking.stop();
            //camera.stop();
            f.startTeleopDrive(true);
            f.setTeleOpDrive(0, 0, 0, true);
        }
    }



    private Action returnToPoint3() {
        return new Action() {
            private FollowPathAction returnPath;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!opModeIsActive()) return false;
                if (returnPath == null) {
                    Pose current = f.getPose();
                    Pose destination = Paths.P_SIMPLE_POINT3;
                    if (Math.hypot(current.getX() - destination.getX(),
                            current.getY() - destination.getY()) < 1.0) return false;

                    returnPath = new FollowPathAction(f, f.pathBuilder()
                            .addPath(new BezierLine(current, destination))
                            .setLinearHeadingInterpolation(current.getHeading(), Paths.H_270)
                            .build(), true);
                }
                return returnPath.run(packet);
            }
        };
    }
}

 */
