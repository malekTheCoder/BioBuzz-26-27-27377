package org.firstinspires.ftc.teamcode.BioBuzz.Auto;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close.NewPaths;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.NewRobotActions;
@Autonomous(name = "Simple Auto", group = "Homework")
public class SimpleAuto extends AbstractAuto {

    private NewPaths paths; // poses + paths live in NewPaths.java

    @Override
    protected Pose getStartPose() {
        return NewPaths.getStartPose();
    }

    @Override
    protected void onInit() {
        paths = new NewPaths(robot.drivetrain);

        // flip poses + headings if the paths are on the other alliance
        if (Common.isRed != NewPaths.isPathRed) {
            NewPaths.isPathRed = !NewPaths.isPathRed;
            paths.mirrorAll();
        }

        super.onInit(); // sets start pose (after mirroring)
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
                        new FollowPathAction(robot.drivetrain, paths.path2, true),
                        // path 3: intake on, half speed so it can grab all 4 balls
                        new InstantAction(() -> robot.drivetrain.setMaxPower(0.5)),
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path3, true),
                                NewRobotActions.startIntake(1)
                        ),
                        new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                        // path 4: intake keeps going 1 s so the last ball gets in;
                        // spin up shooter + aim turret while driving, then shoot
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path4, true),
                                NewRobotActions.intakeFor(1, 1.0),
                                NewRobotActions.enableShooter(),
                                NewRobotActions.startTurretTracking()
                        ),
                        NewRobotActions.shoot(1.0),
                        NewRobotActions.stopShooter(),
                        new FollowPathAction(robot.drivetrain, paths.path5, true),
                        // path 6: intake on + shooter spinning up, then intake off and shoot
                        new ParallelAction(
                                new FollowPathAction(robot.drivetrain, paths.path6, true),
                                NewRobotActions.startIntake(1),
                                NewRobotActions.enableShooter(),
                                NewRobotActions.startTurretTracking()
                        ),
                        NewRobotActions.stopIntake(),
                        NewRobotActions.shoot(1.0),
                        NewRobotActions.stopAll()
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
