package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;

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
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new FollowPathAction(f, path.simplePath1, true),
                        new SleepAction(3.0),
                        new FollowPathAction(f, path.simplePath2, true),
                        new FollowPathAction(f, path.simplePath3, true)
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
