package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;

public class Top10autoOat{
    private final Follower f;
    private final PathNaz paths;

    public Top10autoOat(Follower follower, PathNaz paths) {
        this.f = follower;
        this.paths = paths;
    }
    public void collectAndShoot() {

        Common.robot.actionScheduler.addAction(
                new SequentialAction(

                        new FollowPathAction(f, paths.collect, true),

                        new ParallelAction(
                                new InstantAction(
                                        () -> f.setMaxPower(0.6)
                                ),
                                RobotActions.intakeAction(1, 3),
                                new FollowPathAction(f, paths.openIntake, false)
                        ),

                        new ParallelAction(
                                new InstantAction(
                                        () -> f.setMaxPower(0.9)
                                ),
                                RobotActions.enableShooter(),
                                new FollowPathAction(f, paths.backToShoot, true)
                        ),

                        RobotActions.waitForShooter(),
                        RobotActions.openGateFor(1.0)
                )
        );
    }

}