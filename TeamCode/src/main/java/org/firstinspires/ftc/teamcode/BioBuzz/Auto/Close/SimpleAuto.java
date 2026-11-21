package org.firstinspires.ftc.teamcode.BioBuzz.Auto;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close.NewPaths;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;
@Autonomous(name = "Simple Auto", group = "Homework")
public class SimpleAuto extends AbstractAuto {

    private NewPaths paths; // poses + paths live in NewPaths.java

    private Follower f;
    private NewPaths path;
    @Override
    protected Pose getStartPose() {
        return NewPaths.getStartPose();
    }


    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new NewPaths(f);

        if (Common.isRed != NewPaths.isPathRed) {
            NewPaths.isPathRed = !NewPaths.isPathRed;
            path.mirrorAll();
        }

        path.autoNewPaths();
    }

    @Override
    protected void onRun() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        // path 1: drive to shoot spot while spinning up shooter + aiming turret
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path1, true),
                                RobotActions.shootAction(3.0),
                                RobotActions.startTurretTracking()
                        ),
                        RobotActions.shootAction(3.0),
                        new FollowPathAction(robot.drivetrain, paths.path2, true),
                        // path 3: intake on, half speed so it can grab all 4 balls
                        new InstantAction(() -> robot.drivetrain.setMaxPower(0.5)),
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path3, true),
                                RobotActions.intakeAction(1.0, 3.0)
                        ),
                        new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path4, true),
                                RobotActions.intakeAction(1, 1.0),
                                RobotActions.shootAction(3.0),
                                RobotActions.startTurretTracking()
                        ),
                        RobotActions.shootAction(3.0),
                        new FollowPathAction(robot.drivetrain, paths.path5, true),
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path6, true),
                                RobotActions.intakeAction(1.0, 3.0),
                                RobotActions.shootAction(3.0)
//                                RobotActions.startTurretTracking()
                        )
                )
        );
        robot.actionScheduler.runBlocking();
    }
}