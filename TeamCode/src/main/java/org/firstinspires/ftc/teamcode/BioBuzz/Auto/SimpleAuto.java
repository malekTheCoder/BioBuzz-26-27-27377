package org.firstinspires.ftc.teamcode.BioBuzz.Auto;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close.NewPaths;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.NewRobotActions;
@Autonomous(name = "Simple Auto", group = "Homework")
public class SimpleAuto extends AbstractAuto {

    private NewPaths paths; // poses + paths live in NewPaths.java

    @Override
    protected Pose getStartPose() {
        return NewPaths.START;
    }

    @Override
    protected void onInit() {
        super.onInit();

        paths = new NewPaths(robot.drivetrain);
        paths.build();
    }

    @Override
    protected void onRun() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        // path 1: drive to shoot spot while spinning up shooter + aiming turret
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path1, true),
                                NewRobotActions.enableShooter(),
                                NewRobotActions.startTurretTracking()
                        ),
                        NewRobotActions.shoot(1.0),
                        NewRobotActions.stopShooter(),
                        // paths 2-6
                        new FollowPathAction(robot.drivetrain, paths.path2, true),
                        new FollowPathAction(robot.drivetrain, paths.path3, true),
                        new FollowPathAction(robot.drivetrain, paths.path4, true),
                        new FollowPathAction(robot.drivetrain, paths.path5, true),
                        // end of path 5 (top): intake on, keep it on through path 6
                        NewRobotActions.startIntake(1),
                        new FollowPathAction(robot.drivetrain, paths.path6, true),
                        NewRobotActions.stopIntake(),
                        NewRobotActions.stopAll()
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
