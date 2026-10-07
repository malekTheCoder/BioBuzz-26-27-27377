package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;

@Autonomous(name = "PRACTICE_AUTO")
public class PracticeAuto extends AbstractAuto {
    private Follower follower;
    private PracticeAutoPath paths;
    private boolean turretAiming = true;

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

        robot.limelight.getLimelight().pipelineSwitch(Common.isRed ? 9 : 8);

        follower = robot.drivetrain;
        paths = new PracticeAutoPath(follower);

        // ---- alliance mirroring ----
        if (Common.isRed != PracticeAutoPath.isPathRed) {
            PracticeAutoPath.isPathRed = !PracticeAutoPath.isPathRed;
            paths.mirrorAll();
        }

        paths.goal3Build();
        robot.drivetrain.setPose(getStartPose());

        // ---- turret auto aim ----
        RobotActions.setGoal(Common.isRed ? new Pose(136, 136) : new Pose(136, 136).mirror());
        robot.actionScheduler.setUpdate(() -> {
            if (turretAiming) {
                robot.turret.autoAim(robot.limelight, robot.drivetrain.getPose(), RobotActions.getGoal());
            }
            update();
        });
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
                        new ParallelAction(
                                    new SequentialAction(
                                            new InstantAction(() -> robot.gate.openGate()),
                                            new SleepAction(4),
                                            new InstantAction(() -> robot.gate.closeGate())
                                    ),
                                    new Actions.CallbackAction(RobotActions.autoShooter(3), paths.shootOne, 0.2, 0, follower, "Shoot One"),
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
                                new SequentialAction(
                                        new InstantAction(() -> robot.gate.openGate()),
                                        new SleepAction(4),
                                        new InstantAction(() -> robot.gate.closeGate())
                                ),
                                new Actions.CallbackAction(RobotActions.autoShooter(3), paths.shootTwo, 0.2, 0, follower, "Shoot Two"),
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
        ElapsedTime centerTimer = new ElapsedTime();

        robot.actionScheduler.addAction(
                new ParallelAction(
                        RobotActions.stopAll(),
                        new FollowPathAction(follower, paths.park),

                        // Center the turret so the next opmode's 0 degrees is robot forward
                        new SequentialAction(
                                new InstantAction(() -> {
                                    turretAiming = false;
                                    robot.turret.setTargetAngle(0);
                                    centerTimer.reset();
                                }),
                                new Actions.RunnableAction(() -> !robot.turret.atTarget() && centerTimer.seconds() < 2)
                        )
                )
        );
        robot.actionScheduler.runBlocking();
    }
}