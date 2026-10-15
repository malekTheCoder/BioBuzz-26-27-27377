package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
    public class PracticePedroPathing extends OpMode{

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_SHOOTPS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_INTAKEPOS
    }

    PathState pathState;

    public final Pose startPose = new Pose(43, 8.5, Math.toRadians(90) );
    public final Pose shootingPose = new Pose(58.39366197183098, 23.954937926816275, Math.toRadians(90));
    public final Pose intakePose = new Pose(70.57323943661972, 8.530281690140855, Math.toRadians(90));
    public PathChain  driveStatShoot, driveShootPosIntakePos;
    public void buildPath(){
        //put in coordinates for starting pose > ending pose
        driveStatShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, intakePose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), intakePose.getHeading())
                .build();
    }

    public void setPathStateUpdate(){
        switch (pathState){
            case DRIVE_SHOOTPS_SHOOT_POS:
                follower.followPath(driveStatShoot, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //Check is followers done its path?
                if(!follower.isBusy()) {
                    //TODO add logic to flywheel shooter
                    telemetry.addLine("Done state machine");
                    setPathState(PathState.DRIVE_INTAKEPOS);
                }
                break;
            case DRIVE_INTAKEPOS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    //How many seconds are we shooting for
                    telemetry.addLine("Second Path Completed");
                    follower.followPath(driveShootPosIntakePos, true);
                }
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

    @Override
    public void init(){
        pathState = PathState.DRIVE_SHOOTPS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add any other init mechanisms

        buildPath();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop(){
        follower.update();
        setPathStateUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
    }

}
