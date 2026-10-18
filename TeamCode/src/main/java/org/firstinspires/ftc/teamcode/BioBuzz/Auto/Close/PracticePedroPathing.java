package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.NewRobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;

@Autonomous
    public class PracticePedroPathing extends OpMode{

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Action currentAction; // robot action running alongside the paths
    private boolean lastA = false; // for the red/blue toggle
    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_SHOOTPS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_PATH2,
        DRIVE_PATH3,
        DRIVE_PATH4,
        SHOOT_PATH4,
        DRIVE_PATH5,
        DRIVE_PATH6,
        SHOOT_PATH6,
        DONE
    }

    PathState pathState;

    private NewPaths paths; // poses + paths live in NewPaths.java

    public void setPathStateUpdate(){
        switch (pathState){
            case DRIVE_SHOOTPS_SHOOT_POS:
                follower.followPath(paths.path1, true);
                // spin up shooter + aim turret while driving
                startAction(new ParallelAction(
                        NewRobotActions.enableShooter(),
                        NewRobotActions.startTurretTracking()
                ));
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //Check is followers done its path?
                if(!follower.isBusy() && currentAction == null) {
                    startAction(new SequentialAction(
                            NewRobotActions.shoot(1.0),
                            NewRobotActions.stopShooter()
                    ));
                    setPathState(PathState.DRIVE_PATH2);
                }
                break;
            // each state waits for the last path + action to finish, then starts the next path
            case DRIVE_PATH2:
                followWhenReady(paths.path2, PathState.DRIVE_PATH3);
                break;
            case DRIVE_PATH3:
                // intake on through path 3, half speed so it can grab all 4 balls
                if (!follower.isBusy() && currentAction == null) {
                    startAction(NewRobotActions.startIntake(1));
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.path3, true);
                    setPathState(PathState.DRIVE_PATH4);
                }
                break;
            case DRIVE_PATH4:
                // end of path 3: full speed again, intake keeps going 1 s so the last ball gets in;
                // spin up shooter + aim turret while driving path 4
                if (!follower.isBusy() && currentAction == null) {
                    follower.setMaxPower(1);
                    startAction(new ParallelAction(
                            NewRobotActions.intakeFor(1, 1.0),
                            NewRobotActions.enableShooter(),
                            NewRobotActions.startTurretTracking()
                    ));
                    follower.followPath(paths.path4, true);
                    setPathState(PathState.SHOOT_PATH4);
                }
                break;
            case SHOOT_PATH4:
                // end of path 4: shoot
                if (!follower.isBusy() && currentAction == null) {
                    startAction(new SequentialAction(
                            NewRobotActions.shoot(1.0),
                            NewRobotActions.stopShooter()
                    ));
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;
            case DRIVE_PATH5:
                followWhenReady(paths.path5, PathState.DRIVE_PATH6);
                break;
            case DRIVE_PATH6:
                // end of path 5 (top): intake on through path 6, shooter spins up on the way
                if (!follower.isBusy() && currentAction == null) {
                    startAction(new ParallelAction(
                            NewRobotActions.startIntake(1),
                            NewRobotActions.enableShooter(),
                            NewRobotActions.startTurretTracking()
                    ));
                    follower.followPath(paths.path6, true);
                    setPathState(PathState.SHOOT_PATH6);
                }
                break;
            case SHOOT_PATH6:
                // end of path 6: intake off, shoot what it picked up
                if (!follower.isBusy() && currentAction == null) {
                    startAction(new SequentialAction(
                            NewRobotActions.stopIntake(),
                            NewRobotActions.shoot(1.0),
                            NewRobotActions.stopShooter()
                    ));
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void followWhenReady(PathChain path, PathState nextState) {
        if (!follower.isBusy() && currentAction == null) {
            follower.followPath(path, true);
            setPathState(nextState);
        }
    }

    public void startAction(Action action) {
        currentAction = action;
    }

    // runs the current action one step; clears it when it finishes
    public void runAction() {
        if (currentAction != null && !currentAction.run(new TelemetryPacket())) {
            currentAction = null;
        }
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_SHOOTPS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        robot = new Robot(hardwareMap); // NewRobotActions use this
        follower = robot.drivetrain;

        paths = new NewPaths(follower);
    }

    // press A during init to pick red or blue
    @Override
    public void init_loop() {
        if (gamepad1.a && !lastA) Common.isRed = !Common.isRed;
        lastA = gamepad1.a;

        telemetry.addLine("PRESS A TO TOGGLE SIDES");
        telemetry.addData("IS RED: ", Common.isRed);
    }

    @Override
    public void start() {
        // flip poses + headings if the paths are on the other alliance
        if (Common.isRed != NewPaths.isPathRed) {
            NewPaths.isPathRed = !NewPaths.isPathRed;
            paths.mirrorAll();
        }
        paths.build();
        follower.setPose(NewPaths.getStartPose());

        robot.turret.setAlliance();
        NewRobotActions.setGoal(Common.getAllianceGoal());

        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop(){
        robot.run(); // updates follower, turret and shooter
        runAction();
        setPathStateUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
    }

}
