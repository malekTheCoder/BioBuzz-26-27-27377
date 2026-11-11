package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class Paths {

    // how it will work

    // preshot -> garden -. flower -> shoot garden shoot flower -> thru middle cluster to old balls

    private final Follower f;

    public PathChain firstShot;
    public PathChain gardenCollect;

    public PathChain gardenIntake;

    public PathChain gardenShot;

    public PathChain clusterShotOne;


    public PathChain intake3;
    public PathChain shoot3;

    public PathChain intake6;
    public PathChain shoot6;

    public PathChain unloadRamp;

    public PathChain intake9;
    public PathChain shoot9;


    public PathChain intakeHuman;


    public PathChain shootHuman;


    public PathChain intakeExtra1;
    public PathChain shootExtra1;
    public PathChain intakeExtra2;

    public PathChain shootExtra2;
    public PathChain intakeExtra3;
    public PathChain shootExtra3;

    public PathChain leave;

    public PathChain simplePath1;
    public PathChain simplePath2;
    public PathChain simplePath3;

    // =============================
    // POSES (ALL CENTRALIZED)
    // =============================

    // Simple path-only auto.
    public static Pose P_SIMPLE_START = new Pose(54.9, 8.7103);
    public static Pose P_SIMPLE_PATH1 = new Pose(54.9, 13.5);
    public static Pose P_SIMPLE_POINT2_START = new Pose(54.9, 13.5);
    public static Pose P_SIMPLE_POINT2 = new Pose(9.5, 20.0257);
    public static Pose P_SIMPLE_POINT2_CONTROL1 = new Pose(9.5, 75.4478);
    public static Pose P_SIMPLE_POINT3_START = new Pose(6.4238, 8.0257);
    public static Pose P_SIMPLE_POINT3 = new Pose(57.2496, 125.3026);
    public static Pose P_SIMPLE_POINT3_CONTROL1 = new Pose(30.9374, 137.7721);

    public static Pose P_SHOOT = new Pose(89.000, 102.000);



    // First Shot
    public static Pose P_START = new Pose(55, 8);

    public static Pose P_SH_1 = new Pose(60, 83);

    //Garden Intake and Shot

    public static Pose P_G_START = new Pose (6, 10);

    public static Pose P_G_COLLECT = new Pose (6, 4);

    public static Pose P_G_SHOOT = new Pose (52, 138);

    public static Pose P_CLUSTER_ONE = new Pose(62, 28);

    // Intake 3
    public static Pose P_I3_CP = new Pose(79.512, 85.6);
    public static Pose P_I3_END = new Pose(95.550, 85.6);
    public static Pose P_I3_WALL = new Pose(124, 85.6);

    // Intake 6
    public static Pose P_I6_CP = new Pose(73.435, 62.5);
    public static Pose P_I6_END = new Pose(96.563, 62.5);
    public static Pose P_I6_WALL = new Pose(133, 62.5);

    // Ramp
    public static Pose P_RAMP_CP = new Pose(92.680, 75.123);
    public static Pose P_RAMP_END = new Pose(127.8, 75);

    // Intake 9
    public static Pose P_I9_CP = new Pose(90, 39);
    public static Pose P_I9_END = new Pose(99.264, 39);
    public static Pose P_I9_WALL = new Pose(137, 39);
    public static Pose P_I9_RETURN = new Pose(99.433, 39);

    // Extra balls
    public static Pose P_HP_CP1 = new Pose(111.920, 56.783);
    public static Pose P_HP_END = new Pose(137.788, 49.329);
    public static Pose P_HP_WALL = new Pose(137.807, 6);
    public static Pose P_HP_RETURN = new Pose(137.638, 49.348);

    public static Pose P_EX_CP1 = new Pose(93.862, 66.851);
    public static Pose P_EX_END1 = new Pose(133.027, 60.774);

    public static Pose P_EX_CP2 = new Pose(93.862, 66.682);
    public static Pose P_EX_END2 = new Pose(133.027, 60.774);

    public static Pose P_EX_CP3 = new Pose(94.368, 66.513);
    public static Pose P_EX_END3 = new Pose(133.027, 60.605);

    // Leave
    public static Pose P_LEAVE_END = new Pose(89.135, 80);
    public static double H_38 = Math.toRadians(38);
    public static double H_0 = Math.toRadians(0);

    public static double H_90 = Math.toRadians(90);

    public static double H_43 = Math.toRadians(43);
    public static double H_270 = Math.toRadians(270);

    public static double H_300 = Math.toRadians(300);

    public Paths(Follower follower) {
        f = follower;
    }

    private static final double FIELD_SIZE = 144.0;

    private static Pose mirrorDiagonal(Pose pose) {
        return new Pose(FIELD_SIZE - pose.getX(), FIELD_SIZE - pose.getY());
    }

    public double mirrorAngleRad(double angle) {
        double mirrored = (angle + Math.PI) % (2 * Math.PI);
        return mirrored < 0 ? mirrored + 2 * Math.PI : mirrored;
    }

    public static boolean isPathRed = true;

    public void mirrorAll() {
        P_START = mirrorDiagonal(P_START);
        P_SHOOT = mirrorDiagonal(P_SHOOT);

        P_SIMPLE_START = mirrorDiagonal(P_SIMPLE_START);
        P_SIMPLE_PATH1 = mirrorDiagonal(P_SIMPLE_PATH1);
        P_SIMPLE_POINT2_START = mirrorDiagonal(P_SIMPLE_POINT2_START);
        P_SIMPLE_POINT2 = mirrorDiagonal(P_SIMPLE_POINT2);
        P_SIMPLE_POINT2_CONTROL1 = mirrorDiagonal(P_SIMPLE_POINT2_CONTROL1);
        P_SIMPLE_POINT3_START = mirrorDiagonal(P_SIMPLE_POINT3_START);
        P_SIMPLE_POINT3 = mirrorDiagonal(P_SIMPLE_POINT3);
        P_SIMPLE_POINT3_CONTROL1 = mirrorDiagonal(P_SIMPLE_POINT3_CONTROL1);

        P_SH_1 = mirrorDiagonal(P_SH_1);

        P_G_START = mirrorDiagonal(P_G_START);
        P_G_COLLECT = mirrorDiagonal(P_G_COLLECT);
        P_G_SHOOT = mirrorDiagonal(P_G_SHOOT);

        P_I3_CP = mirrorDiagonal(P_I3_CP);
        P_I3_END = mirrorDiagonal(P_I3_END);
        P_I3_WALL = mirrorDiagonal(P_I3_WALL);

        P_I6_CP = mirrorDiagonal(P_I6_CP);
        P_I6_END = mirrorDiagonal(P_I6_END);
        P_I6_WALL = mirrorDiagonal(P_I6_WALL);

        P_RAMP_CP = mirrorDiagonal(P_RAMP_CP);
        P_RAMP_END = mirrorDiagonal(P_RAMP_END);

        P_I9_CP = mirrorDiagonal(P_I9_CP);
        P_I9_END = mirrorDiagonal(P_I9_END);
        P_I9_WALL = mirrorDiagonal(P_I9_WALL);
        P_I9_RETURN = mirrorDiagonal(P_I9_RETURN);



        P_HP_CP1 = mirrorDiagonal(P_HP_CP1);
        P_HP_END = mirrorDiagonal(P_HP_END);
        P_HP_WALL = mirrorDiagonal(P_HP_WALL);
        P_HP_RETURN = mirrorDiagonal(P_HP_RETURN);

        P_EX_CP1 = mirrorDiagonal(P_EX_CP1);
        P_EX_END1 = mirrorDiagonal(P_EX_END1);

        P_EX_CP2 = mirrorDiagonal(P_EX_CP2);
        P_EX_END2 = mirrorDiagonal(P_EX_END2);

        P_EX_CP3 = mirrorDiagonal(P_EX_CP3);
        P_EX_END3 = mirrorDiagonal(P_EX_END3);

        P_LEAVE_END = mirrorDiagonal(P_LEAVE_END);

        H_0 = mirrorAngleRad(H_0);
        H_38 = mirrorAngleRad(H_38);
        H_43 = mirrorAngleRad(H_43);
        H_90 = mirrorAngleRad(H_90);
        H_270 = mirrorAngleRad(H_270);
        H_300 = mirrorAngleRad(H_300);

    }


    // =============================
    // BUILD ALL PATHS
    // =============================

    public void simpleBuild() {
        simplePath1 = f.pathBuilder()
                .addPath(new BezierLine(P_SIMPLE_START, P_SIMPLE_PATH1))
                .setConstantHeadingInterpolation(H_90)
                .build();

        simplePath2 = f.pathBuilder()
                .addPath(new BezierCurve(P_SIMPLE_POINT2_START, P_SIMPLE_POINT2_CONTROL1, P_SIMPLE_POINT2))
                .setLinearHeadingInterpolation(H_90, H_270)
                .build();

        simplePath3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SIMPLE_POINT3_START, P_SIMPLE_POINT3_CONTROL1, P_SIMPLE_POINT3))
                .setConstantHeadingInterpolation(H_270)
                .build();
    }
//    public static Pose P_START = new Pose(55, 8);
//
//    public static Pose P_SH_1 = new Pose(55.62, 23);
//
//    //Garden Intake and Shot
//
//    public static Pose P_G_START = new Pose (6, 10);
//
//    public static Pose P_G_COLLECT = new Pose (6, 4);
//
//    public static Pose P_G_SHOOT = new Pose (52, 138);
//
//    public static Pose P_CLUSTER_ONE = new Pose(62, 28);

    public void realAuto () {
        firstShot = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SH_1))
                .setConstantHeadingInterpolation(H_90)
                .build();
        gardenCollect = f.pathBuilder()
                .addPath(new BezierLine(P_SH_1, P_G_START))
                .setLinearHeadingInterpolation(H_90, H_270)
                .build();
        gardenIntake = f.pathBuilder()
                .addPath(new BezierLine(P_G_START, P_G_COLLECT))
                .setConstantHeadingInterpolation(H_270)
                .build();
        gardenShot = f.pathBuilder()
                .addPath(new BezierLine(P_G_COLLECT, P_G_SHOOT))
                .setLinearHeadingInterpolation(H_270, H_300) // probably edit 300?
                .build();
        clusterShotOne = f.pathBuilder()
                .addPath(new BezierLine(P_G_SHOOT, P_CLUSTER_ONE))
                .setConstantHeadingInterpolation(H_300)
                .build();

    }
}
