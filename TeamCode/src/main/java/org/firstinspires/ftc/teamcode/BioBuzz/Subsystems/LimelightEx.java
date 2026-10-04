package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

@Config
@Configurable
public class LimelightEx {

    public static long STALENESS_TIME = 100;
    public static double HEADING_OFFSET = 270.0;
    public static double MAX_RELOCALIZE_JUMP_INCHES = 24.0;

    // Pipeline numbers (must match the Limelight web UI)
    public static int PIPELINE_APRILTAG = 0;
    public static int PIPELINE_POLLEN = 3;
    public static int PIPELINE_NECTAR_BLUE = 4;
    public static int PIPELINE_NECTAR_RED = 5;

    private static final double INCHES_PER_METER = 39.3701;
    private static final double FIELD_CENTER_INCHES = 72.0;

    private final Limelight3A limelight;
    private LLResult result;
    private Pose latestPose;
    private int currentPipeline = -1;

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
        return result != null && result.isValid()
                ? result.getColorResults()
                : Collections.emptyList();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        return result != null && result.isValid()
                ? result.getDetectorResults()
                : Collections.emptyList();
    }

    /** Only switches when needed so we don't spam the camera. */
    public void setPipeline(int index) {
        if (index != currentPipeline) {
            limelight.pipelineSwitch(index);
            currentPipeline = index;
        }
    }

    /**
     * Returns the biggest (closest) ball from the given pipeline, or null.
     * Ignores results still coming from the previous pipeline right after a switch.
     */
    public LLResultTypes.ColorResult getBestBall(int pipelineIndex) {
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != pipelineIndex) {
            return null;
        }
        LLResultTypes.ColorResult best = null;
        for (LLResultTypes.ColorResult c : result.getColorResults()) {
            if (best == null || c.getTargetArea() > best.getTargetArea()) best = c;
        }
        return best;
    }

    public int getBallCount() {
        return getColorResult().size();
    }

    /**
     * Gets a fresh MegaTag2 position in Pedro field coordinates.
     * Heading stays from Pedro/odometry because MegaTag2 uses that heading as an input.
     */
    public Pose getPoseEstimate(double robotHeading) {
        limelight.updateRobotOrientation(Math.toDegrees(robotHeading) - HEADING_OFFSET);
        result = limelight.getLatestResult();

        if (result == null
                || !result.isValid()
                || result.getStaleness() >= STALENESS_TIME
                || result.getBotposeTagCount() == 0) {
            latestPose = null;
            return null;
        }

        Pose3D limelightPose = result.getBotpose_MT2();
        if (limelightPose == null) {
            latestPose = null;
            return null;
        }

        // Limelight starts at field center. Pedro starts at a field corner.
        double x = limelightPose.getPosition().y * INCHES_PER_METER + FIELD_CENTER_INCHES;
        double y = FIELD_CENTER_INCHES - limelightPose.getPosition().x * INCHES_PER_METER;
        latestPose = new Pose(x, y, robotHeading);
        return latestPose;
    }

    /** Use this once at startup because the old Pedro X/Y may still be unknown. */
    public boolean setStartingPose(Follower drivetrain, double startingHeading) {
        Pose visionPose = getPoseEstimate(startingHeading);
        if (visionPose == null) return false;

        drivetrain.setPose(visionPose);
        return true;
    }

    /** Corrects Pedro X/Y only when the vision pose is close enough to the current pose. */
    public boolean relocalize(Follower drivetrain) {
        Pose visionPose = getPoseEstimate(drivetrain.getHeading());
        if (visionPose == null) return false;

        Pose currentPose = drivetrain.getPose();
        double jump = Math.hypot(
                visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY()
        );
        if (jump > MAX_RELOCALIZE_JUMP_INCHES) return false;

        drivetrain.setPose(visionPose);
        return true;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void stop() {
        limelight.stop();
    }

    public void printTelemetry() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("valid pose", latestPose != null);
        if (latestPose != null) {
            telemetry.addData("pose x (in)", latestPose.getX());
            telemetry.addData("pose y (in)", latestPose.getY());
        }
    }
}