package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "PathNaz")
public class PathNaz {
    private Follower f;
    //poses :DD
    private final Pose startPose = new Pose(56, 8);
    private final Pose intakePose = new Pose(13, 7);
    private final Pose collectPose = new Pose(12, 7);
    private final Pose backShoot = new Pose(47, 116);

    public static double H_90 = Math.toRadians(90);
    public static double H_180 = Math.toRadians(180);

    public static double H_0 = Math.toRadians(0);
    public static double H_270 = Math.toRadians(270);

    //paths :P
    private PathChain toShoot;
    PathChain openIntake;
    PathChain collect;
    PathChain backToShoot;


    protected Pose getStartPose() {
        return startPose;
    }




    public void farisPath() {

        // ---------- Preload ----------
        toShoot = f.pathBuilder()
                .addPath(new BezierLine(startPose, intakePose))
                .setConstantHeadingInterpolation(H_180)
                .build();
        openIntake = f.pathBuilder()
                .addPath(new BezierLine(intakePose, collectPose))
                .setConstantHeadingInterpolation(H_180)
                .build();
        collect = f.pathBuilder()
                .addPath(new BezierCurve(collectPose, backShoot))
                .setLinearHeadingInterpolation(H_180, H_270)
                .build();



    }


}