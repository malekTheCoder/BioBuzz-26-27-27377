package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PracticeAutoPath {

    private Follower f;

    public PathChain shootOne;
    public PathChain preIntake;

    public PathChain intake;
    public PathChain shootTwo;
    public PathChain park;



    public static Pose START_POS = new Pose(58, 8, Math.toRadians(90));
    public static Pose SHOOT_ONE_POS = new Pose(58, 25, Math.toRadians(90));
    public static Pose PRE_INTAKE_POS = new Pose(21, 8, Math.toRadians(180));
    public static Pose INTAKE_POS = new Pose(8, 8, Math.toRadians(180));
    public static Pose SHOOT_TWO_POS = new Pose (58, 25, Math.toRadians(90));
    public static Pose PARK_POSE = new Pose (8, 95, Math.toRadians(180));

    public PracticeAutoPath(Follower f) {
        this.f = f;
    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = true;
    public void mirrorAll() {
        START_POS = START_POS.mirror();
        SHOOT_ONE_POS = SHOOT_ONE_POS.mirror();
        PRE_INTAKE_POS = PRE_INTAKE_POS.mirror();
        INTAKE_POS = INTAKE_POS.mirror();
        SHOOT_TWO_POS = SHOOT_TWO_POS.mirror();
        PARK_POSE = PARK_POSE.mirror();

        double startAngle = mirrorAngleRad(START_POS.getHeading());
        double shootOneAngle = mirrorAngleRad(SHOOT_ONE_POS.getHeading());
        double PreGardenAngle = mirrorAngleRad(PRE_INTAKE_POS.getHeading());
        double gardenAngle = mirrorAngleRad(INTAKE_POS.getHeading());
        double shootTwoAngle = mirrorAngleRad(SHOOT_TWO_POS.getHeading());
        double parkAngle = mirrorAngleRad(PARK_POSE.getHeading());



        START_POS.setHeading(startAngle);
        SHOOT_ONE_POS.setHeading(shootOneAngle);
        PRE_INTAKE_POS.setHeading(PreGardenAngle);
        INTAKE_POS.setHeading(gardenAngle);
        SHOOT_TWO_POS.setHeading(shootTwoAngle);
        PARK_POSE.setHeading(parkAngle);

    }
    public void goal3Build() {

        shootOne = f
                .pathBuilder()
                .addPath(new BezierLine(START_POS, SHOOT_ONE_POS))
                .setConstantHeadingInterpolation(START_POS.getHeading())
                .build();

        preIntake = f
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_ONE_POS, PRE_INTAKE_POS))
                .setLinearHeadingInterpolation(SHOOT_ONE_POS.getHeading(), PRE_INTAKE_POS.getHeading())
                .build();

        intake = f
                .pathBuilder()
                .addPath(new BezierLine(PRE_INTAKE_POS, INTAKE_POS))
                .setConstantHeadingInterpolation(PRE_INTAKE_POS.getHeading())
                .build();

        shootTwo = f
                .pathBuilder()
                .addPath(new BezierLine(INTAKE_POS, SHOOT_TWO_POS))
                .setLinearHeadingInterpolation(INTAKE_POS.getHeading(), SHOOT_TWO_POS.getHeading())
                .build();

        park = f
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_TWO_POS, PARK_POSE))
                .setLinearHeadingInterpolation(SHOOT_TWO_POS.getHeading(), PARK_POSE.getHeading())
                .build();
    }
}