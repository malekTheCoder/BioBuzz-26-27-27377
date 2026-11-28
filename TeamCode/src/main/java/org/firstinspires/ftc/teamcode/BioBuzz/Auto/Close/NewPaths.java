package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class NewPaths {

    private final Follower f;

    public PathChain preload;

    public PathChain intake;

    public PathChain shoot2;


    // =============================
    // POSES (ALL CENTRALIZED)
    // =============================

    public static Pose P_START = new Pose(55.2, 9); // 9" = half an 18" robot, so it starts flush with the wall

    // Path 1 (shoot preload)
    public static Pose P_SHOOT = new Pose(55.2, 16);

    // Path 2
    public static Pose CP_INTAKE = new Pose (35, 50.5);
    // Path 3
    public static Pose P_INTAKE = new Pose(11.0, 11.0); // front stops 3" from the edge (was 0"), still reaches all 4 balls

    public static Pose CP_SHOOT2 = new Pose (16, 78);

    // Path 4
    public static Pose P_SHOOT2 = new Pose(55, 138); // keeps path 4 clear of the side flower and the hives

    public static double H_90 = Math.toRadians(90);

    public static double H_180 = Math.toRadians(180);
    public static double H_270 = Math.toRadians(270);
    public static double H_285 = Math.toRadians(285);

    public NewPaths(Follower follower) {
        f = follower;
    }

    // This field is the same after a half-turn (red starts bottom left, blue starts top right),
    // so the other alliance is a 180Â° rotation around the field center, not a left/right flip
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

        CP_INTAKE = rotatePose(CP_INTAKE);

        P_INTAKE = rotatePose(P_INTAKE);

        CP_SHOOT2 = rotatePose(CP_SHOOT2);

        P_SHOOT2 = rotatePose(P_SHOOT2);


        H_90 = rotateAngleRad(H_90);
        H_180 = rotateAngleRad(H_180);
        H_270 = rotateAngleRad(H_270);
        H_285 =rotateAngleRad(H_285);
    }

    // start pose + heading for follower.setPose()
    public static Pose getStartPose() {
        return new Pose(P_START.getX(), P_START.getY(), H_90);
    }


    // =============================
    // BUILD ALL PATHS
    // =============================

    public void autoNewPaths() {

        // ---------- Path 1 (shoot preload) ----------
        preload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setConstantHeadingInterpolation(H_90)
                .build();

        // ---------- Path 2 ----------
        intake = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, CP_INTAKE, P_INTAKE))
                .setLinearHeadingInterpolation(H_90, H_270)
                .build();

        // ---------- Path 3 ----------
        shoot2 = f.pathBuilder()
                .addPath(new BezierCurve(P_INTAKE, CP_SHOOT2, P_SHOOT2))
                .setLinearHeadingInterpolation(H_270, H_285)
                .build();
    }
}