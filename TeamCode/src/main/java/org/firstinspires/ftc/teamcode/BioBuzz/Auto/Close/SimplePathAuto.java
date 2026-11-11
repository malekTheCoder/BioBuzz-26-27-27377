package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.TrackBallsAction;

@Autonomous(name = "SimplePathAuto")
public class SimplePathAuto extends AbstractAuto {
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
        followPaths();
    }

    private void followPaths() {
        f.setMaxPower(0.7);
        LimelightEx camera = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));
        camera.setPipeline(LimelightEx.PIPELINE_APRILTAG);
        TrackBallsAction tracking = RobotActions.trackBalls(camera, Common.isRed,
                6.0, this::opModeIsActive);
        try {
            robot.actionScheduler.addAction(
                new SequentialAction(
                        new FollowPathAction(f, path.simplePath1, true),
                        new SleepAction(3.0),
                        new FollowPathAction(f, path.simplePath2, true),
                        new FollowPathAction(f, path.simplePath3, true),
                        tracking,
                        returnToPoint3()
                )
            );
            robot.actionScheduler.runBlocking();
        } finally {
            tracking.stop();
            camera.stop();
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
