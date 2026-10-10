package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

@Config
public class Common {
    public static Pose AUTO_END_POSE = new Pose(0,0,0);
    public static Robot robot;
    public static MultipleTelemetry dashTelemetry;
    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // auto and turret both use this to know which side we picked
    public static boolean isRed = false;

    // red goal position blue gets mirrored
    public static double RED_GOAL_X = 57.5;
    public static double RED_GOAL_Y = 85.4;
    public static double RED_BOTTOM_GOAL_X = 57.5;
    public static double RED_BOTTOM_GOAL_Y = 59.31;
    public static double FIELD_MIDLINE_Y = 72.0;

    public static Pose getGoalForAlliance(boolean redAlliance) {
        return getGoalForAlliance(redAlliance, true);
    }

    public static Pose getAllianceGoal() {
        return getGoalForAlliance(isRed);
    }

    // Pedro's mirror() puts blue on the opposite X side at the same Y.
    public static Pose getGoalForAlliance(boolean redAlliance, boolean topHalf) {
        double redX = topHalf ? RED_GOAL_X : RED_BOTTOM_GOAL_X;
        double y = topHalf ? RED_GOAL_Y : RED_BOTTOM_GOAL_Y;
        Pose redGoal = new Pose(redX, y);
        return redAlliance ? redGoal : redGoal.mirror();
    }
}
