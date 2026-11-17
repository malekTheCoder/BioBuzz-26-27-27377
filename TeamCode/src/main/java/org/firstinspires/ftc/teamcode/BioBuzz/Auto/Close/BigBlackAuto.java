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

@Autonomous (name = "BioV1Auto")
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
    }

    private void preload (){
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new InstantAction(()-> robot.gateServo.gateMovement(false)),
                                RobotActions.enableShooter(),
                                RobotActions.startTurretTracking(),

                                new FollowPathAction(f, path.path1, true)
                        ),
                        new ParallelAction(
                                new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot()),
                                RobotActions.waitForShooter()
                        ),
                        RobotActions.openGateFor(2.0)

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
                                RobotActions.disableShooter(),
                                RobotActions.startTurretTracking()
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path3, true),
                                new InstantAction(() -> f.setMaxPower(0.5)),
                                RobotActions.intakeAction(1, 4)
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.path4, true),
                                new InstantAction(() -> f.setMaxPower(0.8)),
                                RobotActions.enableShooter()
                        ),
                        )
        );
    }
}