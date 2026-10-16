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

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.NewRobotActions;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;

@Autonomous
    public class PracticePedroPathing extends OpMode{

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Action currentAction; // robot action running alongside the paths
    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_SHOOTPS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_PATH2,
        DRIVE_PATH3,
        DRIVE_PATH4,
        DRIVE_PATH5,
        DRIVE_PATH6,
        STOP_INTAKE,
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
                followWhenReady(paths.path3, PathState.DRIVE_PATH4);
                break;
            case DRIVE_PATH4:
                followWhenReady(paths.path4, PathState.DRIVE_PATH5);
                break;
            case DRIVE_PATH5:
                followWhenReady(paths.path5, PathState.DRIVE_PATH6);
                break;
            case DRIVE_PATH6:
                // end of path 5 (top): intake on, keep it on through path 6
                if (!follower.isBusy() && currentAction == null) {
                    startAction(NewRobotActions.startIntake(1));
                    follower.followPath(paths.path6, true);
                    setPathState(PathState.STOP_INTAKE);
                }
                break;
            case STOP_INTAKE:
                // end of path 6: intake off
                if (!follower.isBusy() && currentAction == null) {
                    startAction(NewRobotActions.stopIntake());
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
        paths.build();
        follower.setPose(NewPaths.START);
    }

    public void start() {
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
