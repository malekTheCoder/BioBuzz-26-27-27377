package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    private final Follower f;

    public PathChain Circle;



    // =============================
    // POSES (ALL CENTRALIZED)
    // =============================

    public static Pose P_START = new Pose(65.35201149425286, 49.62140804597702);
    public static Pose P_END = new Pose(63.753591954022994, 11.926005747126421);


    public static double H_225 = Math.toRadians(225);
    public static double H_90 = Math.toRadians(90);
    public static double H_43 = Math.toRadians(43);
    public static double H_270 = Math.toRadians(270);

    public Paths(Follower follower) {
        f = follower;
    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }

    public static boolean isPathRed = true;

    public void mirrorAll() {
        P_START = P_START.mirror();
        P_END = P_END.mirror();

        H_225 = mirrorAngleRad(H_225);
        H_90 = mirrorAngleRad(H_90);
        H_43 = mirrorAngleRad(H_43);
        H_270 = mirrorAngleRad(H_270);
    }


    // =============================
    // BUILD ALL PATHS
    // =============================
    public void test(){
        Circle = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_END))
                .setLinearHeadingInterpolation(H_90, H_225)
                 .addPath(new BezierLine(P_END,P_END))
                .setLinearHeadingInterpolation(H_225, H_90)
                .build();
    }

//    public void goalRocketBuild() {
//        shootPreload = f.pathBuilder()
//                .addPath(new BezierLine(P_START, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//        intake6 = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
//                .setTangentHeadingInterpolation()
//                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        shoot6 = f.pathBuilder()
//                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//        // ---------- Ramp ----------
//        unloadRamp = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_RAMP_CP, P_RAMP_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .build();
//        intake3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
//                .setConstantHeadingInterpolation(H_0)
//                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        shoot3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//    }
//
//    public void goal3Build() {
//        shootPreload = f.pathBuilder()
//                .addPath(new BezierLine(P_START, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        leave = f.pathBuilder()
//                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
//                .setConstantHeadingInterpolation(H_38)
//                .build();
//    }
//
//    public void goal6Build() {
//        shootPreload = f.pathBuilder()
//                .addPath(new BezierLine(P_START, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        // ---------- Intake + Shoot 3 ----------
//        intake3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
//                .setConstantHeadingInterpolation(H_0)
//                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        shoot3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        leave = f.pathBuilder()
//                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
//                .setConstantHeadingInterpolation(H_38)
//                .build();
//    }
//
//    public void goal9Build() {
//
//        // ---------- Preload ----------
//        shootPreload = f.pathBuilder()
//                .addPath(new BezierLine(P_START, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        // ---------- Intake + Shoot 3 ----------
//        intake3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
//                .setConstantHeadingInterpolation(H_0)
//                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        shoot3 = f.pathBuilder()
//                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        // ---------- Intake + Shoot 6 ----------
//        intake6 = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
//                .setTangentHeadingInterpolation()
//                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        shoot6 = f.pathBuilder()
//                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
//                .setLinearHeadingInterpolation(H_0, H_38)
//                .build();
//
//        // ---------- Ramp ----------
//        unloadRamp = f.pathBuilder()
//                .addPath(new BezierCurve(P_SHOOT, P_RAMP_CP, P_RAMP_END))
//                .setLinearHeadingInterpolation(H_38, H_0)
//                .build();
//    }
//
//        public void goal21Build() {
//
//            // ---------- Preload ----------
//            shootPreload = f.pathBuilder()
//                    .addPath(new BezierLine(P_START, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_0, H_38)
//                    .build();
//
//            // ---------- Intake + Shoot 3 ----------
//            intake3 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
//                    .setLinearHeadingInterpolation(H_38, H_0)
//                    .addPath(new BezierLine(P_I3_END, P_I3_WALL))
//                    .setConstantHeadingInterpolation(H_0)
//                    .addPath(new BezierLine(P_I3_WALL, P_I3_END))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .build();
//
//            shoot3 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_0, H_38)
//                    .build();
//
//            // ---------- Intake + Shoot 6 ----------
//            intake6 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
//                    .setLinearHeadingInterpolation(H_38, H_0)
//                    .addPath(new BezierLine(P_I6_END, P_I6_WALL))
//                    .setTangentHeadingInterpolation()
//                    .addPath(new BezierLine(P_I6_WALL, P_I6_END))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .build();
//
//            shoot6 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_0, H_38)
//                    .build();
//
//            // ---------- Ramp ----------
//            unloadRamp = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_RAMP_CP, P_RAMP_END))
//                    .setLinearHeadingInterpolation(H_38, H_0)
//                    .build();
//
//            // ---------- Intake + Shoot 9 ----------
//            intake9 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_RAMP_END, P_I9_CP, P_I9_END))
//                    .setConstantHeadingInterpolation(H_0)
//                    .addPath(new BezierLine(P_I9_END, P_I9_WALL))
//                    .setTangentHeadingInterpolation()
//                    .addPath(new BezierLine(P_I9_WALL, P_I9_RETURN))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .build();
//
//            shoot9 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_I9_RETURN, P_I9_CP, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_0, H_38)
//                    .build();
//            intakeHuman = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_HP_CP1, P_HP_END))
//                    .setLinearHeadingInterpolation(H_38, H_270)
//                    .addPath(new BezierLine(P_HP_END, P_HP_WALL))
//                    .setTangentHeadingInterpolation()
//                    .addPath(new BezierLine(P_HP_WALL, P_HP_RETURN))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .build();
//
//            // ---------- Shoot Human ----------
//            shootHuman = f.pathBuilder()
//                    .addPath(new BezierCurve(P_HP_RETURN, P_HP_CP1, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_270, H_38)
//                    .build();
//
//            // ---------- Extra Balls ----------
//            intakeExtra1 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_EX_CP1, P_EX_END1))
//                    .setLinearHeadingInterpolation(H_38, H_43)
//                    .build();
//
//            shootExtra1 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_EX_END1, P_EX_CP1, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_43, H_38)
//                    .build();
//
//            intakeExtra2 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_EX_CP2, P_EX_END2))
//                    .setLinearHeadingInterpolation(H_38, H_43)
//                    .build();
//
//            shootExtra2 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_EX_END2, P_EX_CP2, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_43, H_38)
//                    .build();
//
//            intakeExtra3 = f.pathBuilder()
//                    .addPath(new BezierCurve(P_SHOOT, P_EX_CP3, P_EX_END3))
//                    .setLinearHeadingInterpolation(H_38, H_43)
//                    .build();
//
//            shootExtra3 = f.pathBuilder()
//                    .addPath(new BezierLine(P_EX_END3, P_SHOOT))
//                    .setLinearHeadingInterpolation(H_43, H_38)
//                    .build();
//
//            // ---------- Leave ----------
//            leave = f.pathBuilder()
//                    .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
//                    .setConstantHeadingInterpolation(H_38)
//                    .build();
//        }
    }
