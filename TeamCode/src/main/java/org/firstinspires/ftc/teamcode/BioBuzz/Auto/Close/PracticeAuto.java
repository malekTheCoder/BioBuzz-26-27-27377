package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;

@Autonomous(name = "PRACTICE_AUTO")
public class PracticeAuto extends AbstractAuto {

    private Follower follower;
    private PracticeAutoPath paths;

    // -----------------------------
    // START POSE
    // -----------------------------
    @Override
    protected Pose getStartPose() {
        return PracticeAutoPath.START_POS;
    }

    // -----------------------------
    // INIT
    // -----------------------------
    @Override
    protected void onInit() {

        follower = robot.drivetrain;
        paths = new PracticeAutoPath(follower);

        // ---- alliance mirroring ----
        if (Common.isRed != PracticeAutoPath.isPathRed) {
            PracticeAutoPath.isPathRed = !PracticeAutoPath.isPathRed;
            paths.mirrorAll();
        }

        paths.goal3Build();
        robot.drivetrain.setPose(getStartPose());
    }

    // -----------------------------
    // RUN
    // -----------------------------
    @Override
    protected void onRun() {

        runShootOne();
        runIntake();
        runShootTwo();
        runPark();
    }

    // -----------------------------
    // SHOOT PRELOAD
    // -----------------------------
    private void runShootOne() {

        robot.actionScheduler.addAction(
                new SequentialAction(

                        // PATH + SHOOTER AT SAME TIME
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(0),
                                        paths.shootOne,
                                        0.2,          // t trigger
                                        0,            // path index
                                        follower,
                                        "Shoot One"
                                ),
                                new FollowPathAction(follower, paths.shootOne, true)
                        )
                )
        );

        robot.actionScheduler.runBlocking();
    }

    // -----------------------------
    // INTAKE
    // -----------------------------
    private void runIntake() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new FollowPathAction(follower, paths.preIntake),

                        // Intake while driving
                        new ParallelAction(
                                RobotActions.intakeAction(1, 2),
                                new FollowPathAction(follower, paths.intake)
                        )
                )
        );

        robot.actionScheduler.runBlocking();
    }

    // -----------------------------
    // SHOOT TWO
    // -----------------------------
    private void runShootTwo() {

        robot.actionScheduler.addAction(
                new SequentialAction(

                        // PATH + SHOOTER AT SAME TIME
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(0),
                                        paths.shootTwo,
                                        0.2,          // t trigger
                                        0,            // path index
                                        follower,
                                        "Shoot Two"
                                ),
                                new FollowPathAction(follower, paths.shootTwo, true)
                        )
                )
        );

        robot.actionScheduler.runBlocking();
    }

    // -----------------------------
    // PARK
    // -----------------------------
    private void runPark() {

        robot.actionScheduler.addAction(
                new ParallelAction(
                        RobotActions.stopAll(),
                        new FollowPathAction(follower, paths.park)
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
