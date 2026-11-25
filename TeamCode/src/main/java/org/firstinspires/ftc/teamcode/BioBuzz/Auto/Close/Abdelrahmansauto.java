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

@Autonomous (name = "AbdelAuto")
public class Abdelrahmansauto extends AbstractAuto {
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
        robot.gate.gateMovement(false);
    }

    @Override
    protected void onRun() {
        preload();
        intake();
        shoot2();

    }

    private static double shotVelocity = 0;

    private void preload() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                    new SequentialAction(
                        RobotActions.startTurretTracking(),
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(.5)),
                                new FollowPathAction(f, path.preload, true)
                    ),
                    new SequentialAction(
                        new InstantAction(() -> {
                            robot.shooter.setPollenVelocity(1660);
                        }),
                        new Actions.RunnableAction(() ->
                                Math.abs(shotVelocity - robot.shooter.getPollenVelocity()) > 100),
                        new InstantAction(() -> {
                            robot.pollenServo.pollenGate(true);
                            robot.intake.intakeElements(1);
                            robot.transfer.loadElements(1);
                        }),
                        new SleepAction(3),
                        // 4. shut everything off
                        new InstantAction(() -> {
                            robot.pollenServo.pollenGate(false);
                            robot.intake.stop();
                            robot.transfer.stop();
                            robot.shooter.setPollenVelocity(0);
                        })
                    )
                 )
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void intake() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new FollowPathAction(f, path.intake, true),
                                new InstantAction(() -> f.setMaxPower(.7))
                        ),
                        RobotActions.intakeAction(1, 4)
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void shoot2() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new FollowPathAction(f, path.shoot2, true),
                                new InstantAction(() -> f.setMaxPower(.8)),
                                RobotActions.startTurretTracking()
                        ),
                        RobotActions.shootAction(3.0)
                )
        );
        robot.actionScheduler.runBlocking();
    }

}
//    private void sixthOne(){
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new FollowPathAction(f, path.path6, true),
//                                new InstantAction(() -> f.setMaxPower(0.7))
//                                RobotActions.intakeAction(1, 4)
//                        )
//
//                )
//       );
//    }

//        private void seventhOne(){
//            robot.actionScheduler.addAction(
//                    new SequentialAction(
//                            new ParallelAction(
//                                new FollowPathAction(f, path.path7),
//                                new InstantAction(() -> f.setMaxPower(0.6))
//                                RobotActions.shootAction(3.0)
//                            )
//                    )
//            );
//        }

