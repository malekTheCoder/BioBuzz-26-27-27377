package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.LimelightEx;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;

@Autonomous (name = "BioV1Auto")
public class Biobuzzauto extends AbstractAuto {
    private Follower f;
    private Paths path;
    private LimelightEx ll;

    @Override
    protected Pose getStartPose() {
        return Paths.P_START.withHeading(Paths.H_90);
    }


    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new Paths(f);

        if (Common.isRed != Paths.isPathRed) {
            Paths.isPathRed = !Paths.isPathRed;
            path.mirrorAll();
        }
        // in onInit():
        ll = new LimelightEx(hardwareMap.get(Limelight3A.class, "limelight"));

        path.realAuto();   // builds firstShot, gardenCollect, gardenIntake, gardenReturn
    }

    @Override
    protected void onRun() {
        shootPreload();
        cycle4();
        //cluster1();
    }

    private void shootPreload () {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.3)),
                                //new InstantAction(() -> robot.gateServo.gateMovement(false)),
                                RobotActions.enableShooter(),
                                RobotActions.startTurretTracking(),

                                new FollowPathAction(f, path.firstShot, true)
                        ),
                        new ParallelAction(
                                new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot()),
                                RobotActions.waitForShooter()
                        )
                        //RobotActions.openGateFor(1.0)
                        // change this to however long we want to keep gate open
                )
        );
        robot.actionScheduler.runBlocking();
    }



    private void cycle4() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new FollowPathAction(f, path.gardenCollect, true),
                                new InstantAction(() -> f.setMaxPower(0.3)),
                                RobotActions.disableShooter(),
                                RobotActions.startTurretTracking()
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.gardenIntake, false),
                                new InstantAction(() -> f.setMaxPower(.3)),
                                RobotActions.intakeAction(1,5)
                        ),
                        new ParallelAction(
                                new FollowPathAction(f, path.gardenShot, true),
                                new InstantAction(() -> f.setMaxPower(.3)),
                                RobotActions.enableShooter()
                                //RobotActions.startTurretTracking()
                        ),
                        new ParallelAction(
                                new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot()),
                                RobotActions.waitForShooter()
                        )
                        //RobotActions.openGateFor(1.0)

                )
        );
        robot.actionScheduler.runBlocking();
    }
    private void cluster1() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        // 1) Set up: gate closed, flywheel spinning, turret aiming
                        new ParallelAction(
                               // RobotActions.closeGate(),
                                RobotActions.enableShooter(),
                                RobotActions.startTurretTracking(),
                                new InstantAction(() -> f.setMaxPower(0.3))   // slower near balls
                        ),

                        // 2) Look for a ball (gives up after 1.5 s)
                        RobotActions.scanForBall(ll, LimelightEx.PIPELINE_POLLEN, 1.5),

                        // 3) Turn the intake on and drive to the ball
                        new InstantAction(() -> robot.intake.intakeElements(1.0)),
                        RobotActions.driveToBall(ll, LimelightEx.PIPELINE_POLLEN),

                        // 4) Keep intaking 0.5 s more after arriving, then stop it
                        RobotActions.intakeAction(1.0, 0.5),

                        // 5) Camera back to AprilTags (for relocalizing later)
                        new InstantAction(() -> ll.setPipeline(LimelightEx.PIPELINE_APRILTAG)),

                        // 6) Wait until turret is aimed AND flywheel is at speed
                        new ParallelAction(
                                new Actions.RunnableAction(() -> !robot.turret.isReadyToShoot()),
                                RobotActions.waitForShooter()
                        ),

                        // 7) Shoot from right here
                        //RobotActions.openGateFor(1.0),
                        new InstantAction(() -> f.setMaxPower(1))
                )
        );
        robot.actionScheduler.runBlocking();
    }
    //   private void shootPreload() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new InstantAction(() -> robot.gateServo.open()),
//                        new InstantAction(()-> f.setMaxPower(1)),
//                        new FollowPathAction(f,path.shootPreload,true),
//                        new InstantAction(() -> RobotActions.startShooter(0))
//                        // new InstantAction(() -> RobotActions.intakeLoaderAction(1,1.5)),
//                )
//
//                //    new InstantAction(() -> RobotActions.intakeLoaderAction(1,1.5)),
//                //  new InstantAction(()-> f.setMaxPower(1))
//
//
//
//        );
//        robot.actionScheduler.runBlocking();
//    }

//        private void cycle6() {
//        path.shoot6.getPath(0).setBrakingStart(0.8);// fl path  ppr gate open and shooter
//        path.shoot6.getPath(0).setBrakingStrength(0.8);
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(()-> f.setMaxPower(0.58)),
//                                new InstantAction(() -> robot.gateServo.close()),
//
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeLoaderAction(1,4),
//                                        path.intake6, 0.01, 0, f, "intake6"
//                                ),
//
//                                new FollowPathAction(f, path.intake6)
//                        ),
//                        new ParallelAction(
//                                new InstantAction(() -> robot.gateServo.open()),
//                                new InstantAction(()-> f.setMaxPower(1)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(0.8), path.shoot6, 0.01, 0, f, "Shoot6"
//                                ),
//                                new FollowPathAction(f, path.shoot6)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeLoaderAction(1,1.5)
//
//                        ),
//                        new InstantAction(() -> robot.shooter.stop()),
//                        new InstantAction(() -> robot.gateServo.close())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void cycle3() {
//        path.shoot3.getPath(0).setBrakingStart(0.9);
//        path.shoot3.getPath(0).setBrakingStrength(0.7);
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(()-> f.setMaxPower(0.7)),
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeLoaderAction(1,3),
//                                        path.intake3,0.01,0,f,"Intake3"
//                                ),
//                                new FollowPathAction(f,path.intake3)
//                        ),
//                        new ParallelAction(
//                                new InstantAction(() -> robot.gateServo.open()),
//                                new InstantAction(()-> f.setMaxPower(0.9)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1.35),
//                                        path.shoot3,0.3,0,f,"Shoot3"
//                                ),
//                                new FollowPathAction(f,path.shoot3)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeLoaderAction(1,1.5)
//
//                        ),
//                        new InstantAction(()-> robot.shooter.stop()),
//                        new InstantAction(() -> robot.gateServo.close()),
//                        new InstantAction(()-> f.setMaxPower(1))
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void unloadRamp() {
//        robot.actionScheduler.addAction(
//                new FollowPathAction(f, path.unloadRamp)
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void cycle9() {
//        path.shoot9.getPath(0).setBrakingStart(0.8);
//        path.shoot9.getPath(0).setBrakingStrength(0.8);
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> robot.gateServo.close()),
//                                new InstantAction(()-> f.setMaxPower(0.7)),
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeLoaderAction(1,4)
//                                        ,path.intake9,0.1,0,f,"Intake9"
//                                ),
//                                new FollowPathAction(f,path.intake9)
//                        ),
//                        new ParallelAction(
//                                new InstantAction(() -> robot.gateServo.open()),
//                                new InstantAction(()-> f.setMaxPower(0.9)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(0.87),path.shoot9,0.001,0,f,"Shoot9"
//                                ),
//                                new FollowPathAction(f,path.shoot9)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeLoaderAction(1,1.5)
//
//                        ),
//                        new InstantAction(()-> robot.shooter.stop()),
//                        new InstantAction(() -> robot.gateServo.close())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void Human() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(()-> f.setMaxPower(0.7)),
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1,4),path.intakeHuman,0.1,0,f,"IntakeHuman"
//                                ),
//                                new FollowPathAction(f,path.intakeHuman)
//                        ),
//                        new ParallelAction(
//                                new InstantAction(()-> f.setMaxPower(0.8)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1.5),path.shootHuman,0.001,0,f,"ShootHuman"
//                                ),
//                                new FollowPathAction(f,path.shootHuman)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1,1.5),
//                                RobotActions.loaderAction(1,1.5)
//                        ),
//                        new InstantAction(()-> robot.shooter.stop())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void extraOne() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1,2),path.intakeExtra1,0.1,0,f,"IntakeExtra1"
//                                ),
//                                new FollowPathAction(f,path.intakeExtra1)
//                        ),
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1),path.shootExtra1,0.5,0,f,"ShootExtra1"
//                                ),
//                                new FollowPathAction(f,path.shootExtra1)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1,1),
//                                RobotActions.loaderAction(1,1)
//                        ),
//                        new InstantAction(()-> robot.shooter.stop())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void extraTwo() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1,2),path.intakeExtra2,0.1,0,f,"IntakeExtra2"
//                                ),
//                                new FollowPathAction(f,path.intakeExtra2)
//                        ),
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1),path.shootExtra2,0.5,0,f,"ShootExtra2"
//                                ),
//                                new FollowPathAction(f,path.shootExtra2)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1,0.7),
//                                RobotActions.loaderAction(1,0.7)
//                        )
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void extraThree() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1,2),path.intakeExtra3,0.1,0,f,"IntakeExtra3"
//                                ),
//                                new FollowPathAction(f,path.intakeExtra3)
//                        ),
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1),path.shootExtra3,0.5,0,f,"ShootExtra3"
//                                ),
//                                new FollowPathAction(f,path.shootExtra3)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1,0.7),
//                                RobotActions.loaderAction(1,0.7)
//                        ),
//                        new FollowPathAction(f,path.leave)
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
}