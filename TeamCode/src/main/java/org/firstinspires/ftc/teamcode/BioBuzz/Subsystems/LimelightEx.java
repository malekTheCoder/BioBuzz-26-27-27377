package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@Configurable
public class LimelightEx {
    private LLResult result;
    private Pose latestPose;
    private final Limelight3A limelight;

    public static long STALENESS_TIME = 100;
    public static double HEADING_OFFSET = 270;

    public static double INCHES_PER_METER = 39.3701;

    public static int BLUE_GOAL_ID = 20; // check against the game manual
    public static int RED_GOAL_ID = 24;

    public LimelightEx(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.start();
    }

    public LLResult update() {
        result = limelight.getLatestResult();
        return result;
    }

    public LLResult getResult() {
        return result;
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        if (result != null && result.isValid()) {return result.getColorResults();
        }
        return java.util.Collections.emptyList();
    }



    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        if (result != null) return result.getDetectorResults();
        return null;
    }

    public Pose getPoseEstimate(double robotHeading) {
        Pose pose = null;
        limelight.updateRobotOrientation(Math.toDegrees(robotHeading) - HEADING_OFFSET );
        if (result != null && result.isValid() && result.getStaleness() < STALENESS_TIME) {
            Pose3D pose3D = result.getBotpose_MT2();
            if (pose3D != null) {
                double x = (pose3D.getPosition().y*INCHES_PER_METER) + 72;

                double y = 72 - (pose3D.getPosition().x*INCHES_PER_METER);

                pose = new Pose(x, y, robotHeading);
            }
        }
        latestPose = pose;
        return pose;
    }

    // The goal april tag for our alliance, ignoring the obelisk and the other goal. null if not visible
    public LLResultTypes.FiducialResult getGoalTag() {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;

        int goalId = Common.isRed ? RED_GOAL_ID : BLUE_GOAL_ID;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == goalId) return tag;
        }
        return null;
    }

    // Flat distance from the camera to the goal april tag, in inches. -1 if no tag is visible
    public double getGoalDistance() {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) return -1;

        Pose3D tagPose = tag.getTargetPoseCameraSpace();
        if (tagPose == null) return -1;

        // camera space: x = right, z = forward (meters)
        double x = tagPose.getPosition().x;
        double z = tagPose.getPosition().z;
        return Math.hypot(x, z) * INCHES_PER_METER;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public boolean hasResult() {
        boolean hasTarget = limelight.getLatestResult().isValid();
        return hasTarget;
    }

    public void printTelemetry() {

        telemetry.addLine("LIMELIGHT");
        if (latestPose != null) {
            telemetry.addData("limelight robot pose x (INCHES): ", latestPose.getX());
            telemetry.addData("limelight robot pose y (INCHES): ", latestPose.getY());
        }

    }
}