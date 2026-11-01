package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

/** Searches for pollen or the alliance's nectar, then approaches one visible ball. */
public final class TrackBallsAction implements Action {
    private static final double PIPELINE_DWELL_SECONDS = 0.45;
    private static final double PICKUP_SECONDS = 0.4;
    private static final double SEARCH_TURN = 0.18;

    private final LimelightEx camera;
    private final Follower follower;
    private final Intake intake;
    private final BooleanSupplier active;
    private final double timeoutSeconds;
    private final int[] pipelines;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started;
    private boolean stopped;
    private int pipelineIndex;
    private double switchAt;
    private double pickupUntil = -1;

    public TrackBallsAction(LimelightEx camera, Follower follower, Intake intake,
                            boolean redAlliance, double timeoutSeconds, BooleanSupplier active) {
        this.camera = camera;
        this.follower = follower;
        this.intake = intake;
        this.active = active;
        this.timeoutSeconds = timeoutSeconds;
        pipelines = new int[] { LimelightEx.PIPELINE_POLLEN,
                redAlliance ? LimelightEx.PIPELINE_NECTAR_RED : LimelightEx.PIPELINE_NECTAR_BLUE };
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (stopped) return false;
        if (!started) {
            started = true;
            timer.reset();
            follower.startTeleopDrive(true);
            camera.setPipeline(pipelines[pipelineIndex]);
            switchAt = PIPELINE_DWELL_SECONDS;
        }

        double now = timer.seconds();
        if (!active.getAsBoolean()) { stop(); return false; }
        if (pickupUntil >= 0) {
            follower.setTeleOpDrive(0, 0, 0, true);
            if (now >= pickupUntil) { stop(); return false; }
            return true;
        }
        if (now >= timeoutSeconds) { stop(); return false; }

        camera.update();
        LLResultTypes.ColorResult ball = camera.getBestBall(pipelines[pipelineIndex]);
        if (ball == null) {
            intake.stop();
            if (now >= switchAt) {
                pipelineIndex = (pipelineIndex + 1) % pipelines.length;
                camera.setPipeline(pipelines[pipelineIndex]);
                switchAt = now + PIPELINE_DWELL_SECONDS;
            }
            follower.setTeleOpDrive(0, 0, SEARCH_TURN, true);
            packet.put("Ball search", "scanning pipeline " + pipelines[pipelineIndex]);
            return true;
        }

        BallTracking.Command command = BallTracking.command(
                ball.getTargetXDegrees(), ball.getTargetArea());
        switchAt = now + PIPELINE_DWELL_SECONDS;
        intake.intakeElements(1.0);
        if (command.close) {
            follower.setTeleOpDrive(0, 0, 0, true);
            pickupUntil = now + PICKUP_SECONDS;
            packet.put("Ball search", "pickup");
        } else {
            // Pedro's turning input has the opposite sign from v1tele's raw mecanum mix.
            follower.setTeleOpDrive(command.forward, 0, -command.turn, true);
            packet.put("Ball search", "tracking pipeline " + pipelines[pipelineIndex]);
        }
        packet.put("Ball tx", ball.getTargetXDegrees());
        packet.put("Ball ta", ball.getTargetArea());
        return true;
    }

    public void stop() {
        if (stopped) return;
        stopped = true;
        intake.stop();
        if (!started) follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
        camera.setPipeline(LimelightEx.PIPELINE_APRILTAG);
    }
}
