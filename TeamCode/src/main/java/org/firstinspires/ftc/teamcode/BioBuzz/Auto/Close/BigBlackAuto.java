package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close.NewPaths;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Shooter;

@Autonomous (name = "BigBlackAuto")
public class BigBlackAuto extends AbstractAuto {
    private Follower f;
    private NewPaths path;
    @Override
    protected Pose getStartPose() {
        return NewPaths.P_START;
    }


    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new NewPaths(f);

        if (Common.isRed != NewPaths.isPathRed) {
            NewPaths.isPathRed = !NewPaths.isPathRed;
            path.mirrorAll();
        }

        path.autoNewPaths();   // builds firstShot, gardenCollect, gardenIntake, gardenReturn
    }

    @Override
    protected void onRun() {
        preload();
        restOfIt();
    }

    private void preload (){
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(1)),
                                new InstantAction(() -> robot.pollenServo.pollenGate(false)),
                                RobotActions.shootAction(3.0),
                                RobotActions.startTurretTracking(),

                                new FollowPathAction(f, path.path1, true)
                        ),
                        new ParallelAction(
                                new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot()),
                                RobotActions.shootAction(3.0)
                        )
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void restOfIt (){
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new FollowPathAction(f, path.path2, true),
                                new InstantAction(() -> f.setMaxPower(1)),
                                RobotActions.startTurretTracking()
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path3, true),
                                new InstantAction(() -> f.setMaxPower(0.5)),
                                RobotActions.intakeAction(1, 4)
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path4, true),
                                new InstantAction(() -> f.setMaxPower(0.8))
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path5, true),
                                new InstantAction(() -> f.setMaxPower(0.7)),
                                RobotActions.shootAction(3.0)
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path6, true),
                                new InstantAction(() -> f.setMaxPower(0.7)),
                                RobotActions.intakeAction(1, 4)
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path7),
                                new InstantAction(() -> f.setMaxPower(0.6)),
                                RobotActions.shootAction(3.0)
                        )
                )
        );
    }
}