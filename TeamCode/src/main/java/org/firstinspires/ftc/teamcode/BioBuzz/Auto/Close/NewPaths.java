package org.firstinspires.ftc.teamcode.BioBuzz.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// Poses and paths from the Pedro Pathing Visualizer (6 paths)
public class NewPaths {

    private final Follower f;

    // ---------- Poses ----------
    public static final Pose START = new Pose(45.1923, 8.5, Math.toRadians(90));
    public static final Pose POINT1 = new Pose(58.3937, 23.9549, Math.toRadians(90));   // shooting spot
    public static final Pose POINT2 = new Pose(41.4761, 9.1282, Math.toRadians(180));
    public static final Pose POINT3 = new Pose(9.068, 9.2071, Math.toRadians(180));
    public static final Pose POINT4_CONTROL = new Pose(18.3134, 87.8641);
    public static final Pose POINT4 = new Pose(54.3081, 118.8197, Math.toRadians(270));
    public static final Pose POINT5 = new Pose(51.1268, 135.8577, Math.toRadians(90));
    public static final Pose POINT6 = new Pose(58, 118.9085, Math.toRadians(270));

    // ---------- Paths ----------
    public PathChain path1, path2, path3, path4, path5, path6;

    public NewPaths(Follower follower) {
        f = follower;
    }

    public void build() {
        path1 = line(START, POINT1);
        path2 = line(POINT1, POINT2);
        path3 = line(POINT2, POINT3);

        path4 = f.pathBuilder()
                .addPath(new BezierCurve(POINT3, POINT4_CONTROL, POINT4))
                .setLinearHeadingInterpolation(POINT3.getHeading(), POINT4.getHeading())
                .build();

        path5 = line(POINT4, POINT5);
        path6 = line(POINT5, POINT6);
    }

    // straight line, heading turns smoothly from start heading to end heading
    private PathChain line(Pose from, Pose to) {
        return f.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }
}
