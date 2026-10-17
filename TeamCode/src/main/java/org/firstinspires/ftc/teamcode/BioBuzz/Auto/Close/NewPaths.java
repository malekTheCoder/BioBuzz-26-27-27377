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

    public static Pose P_START = new Pose(45.1923, 8.5);

    // Path 1 (shoot preload)
    public static Pose P_SHOOT = new Pose(58.3937, 23.9549);

    // Path 2
    public static Pose P_2 = new Pose(41.4761, 9.1282);

    // Path 3
    public static Pose P_3 = new Pose(9.068, 9.2071);

    // Path 4
    public static Pose P_4_CP = new Pose(18.3134, 87.8641);
    public static Pose P_4 = new Pose(54.3081, 118.8197);

    // Path 5
    public static Pose P_5 = new Pose(51.1268, 135.8577);

    // Path 6 (intake on)
    public static Pose P_6 = new Pose(58, 118.9085);

    public static double H_90 = Math.toRadians(90);

    public static double H_180 = Math.toRadians(180);
    public static double H_270 = Math.toRadians(270);

    public NewPaths(Follower follower) {
        f = follower;
    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }

    // poses above are drawn for BLUE (left half of the field)
    public static boolean isPathRed = false;

    public void mirrorAll() {
        P_START = P_START.mirror();

        P_SHOOT = P_SHOOT.mirror();

        P_2 = P_2.mirror();

        P_3 = P_3.mirror();

        P_4_CP = P_4_CP.mirror();
        P_4 = P_4.mirror();

        P_5 = P_5.mirror();

        P_6 = P_6.mirror();

        H_90 = mirrorAngleRad(H_90);
        H_180 = mirrorAngleRad(H_180);
        H_270 = mirrorAngleRad(H_270);
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
