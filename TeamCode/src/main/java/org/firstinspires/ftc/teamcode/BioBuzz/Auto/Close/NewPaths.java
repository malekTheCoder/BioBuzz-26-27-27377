package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class NewPaths {

    private final Follower f;

    public PathChain path1;

    public PathChain path2;

    public PathChain path3;

    public PathChain path4;

    public PathChain path5;

    public PathChain path6;

    // =============================
    // POSES (ALL CENTRALIZED)
    // =============================

    public static Pose P_START = new Pose(45.1923, 9.0); // 9" = half an 18" robot, so it starts flush with the wall

    // Path 1 (shoot preload)
    public static Pose P_SHOOT = new Pose(58.3937, 23.9549);

    // Path 2
    public static Pose P_2 = new Pose(41.4761, 9.1282);

    // Path 3
    public static Pose P_3 = new Pose(9.068, 9.2071);

    // Path 4
    public static Pose P_4_CP = new Pose(38, 138); // keeps path 4 clear of the left flower and the hives
    public static Pose P_4 = new Pose(54.3081, 118.8197);

    // Path 5
    public static Pose P_5 = new Pose(51.1268, 126.0); // was 135.86: robot went past the top wall and hit the top flower

    // Path 6 (intake on)
    public static Pose P_6 = new Pose(58, 118.9085);

    public static double H_90 = Math.toRadians(90);

    public static double H_180 = Math.toRadians(180);
    public static double H_270 = Math.toRadians(270);

    public NewPaths(Follower follower) {
        f = follower;
    }

    // This field is the same after a half-turn (red starts bottom left, blue starts top right),
    // so the other alliance is a 180° rotation around the field center, not a left/right flip
    public Pose rotatePose(Pose pose) {
        return new Pose(144 - pose.getX(), 144 - pose.getY());
    }

    public double rotateAngleRad(double angle) {
        return angle + Math.PI;
    }

    // poses above are drawn for RED (start bottom left)
    public static boolean isPathRed = true;

    public void mirrorAll() {
        P_START = rotatePose(P_START);

        P_SHOOT = rotatePose(P_SHOOT);

        P_2 = rotatePose(P_2);

        P_3 = rotatePose(P_3);

        P_4_CP = rotatePose(P_4_CP);
        P_4 = rotatePose(P_4);

        P_5 = rotatePose(P_5);

        P_6 = rotatePose(P_6);

        H_90 = rotateAngleRad(H_90);
        H_180 = rotateAngleRad(H_180);
        H_270 = rotateAngleRad(H_270);
    }

    // start pose + heading for follower.setPose()
    public static Pose getStartPose() {
        return new Pose(P_START.getX(), P_START.getY(), H_90);
    }


    // =============================
    // BUILD ALL PATHS
    // =============================

    public void build() {

        // ---------- Path 1 (shoot preload) ----------
        path1 = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setConstantHeadingInterpolation(H_90)
                .build();

        // ---------- Path 2 ----------
        path2 = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_2))
                .setLinearHeadingInterpolation(H_90, H_180)
                .build();

        // ---------- Path 3 ----------
        path3 = f.pathBuilder()
                .addPath(new BezierLine(P_2, P_3))
                .setConstantHeadingInterpolation(H_180)
                .build();

        // ---------- Path 4 ----------
        path4 = f.pathBuilder()
                .addPath(new BezierCurve(P_3, P_4_CP, P_4))
                .setLinearHeadingInterpolation(H_180, H_270)
                .build();

        // ---------- Path 5 ----------
        path5 = f.pathBuilder()
                .addPath(new BezierLine(P_4, P_5))
                .setLinearHeadingInterpolation(H_270, H_90)
                .build();

        // ---------- Path 6 (intake on) ----------
        path6 = f.pathBuilder()
                .addPath(new BezierLine(P_5, P_6))
                .setLinearHeadingInterpolation(H_90, H_270)
                .build();
    }
}
