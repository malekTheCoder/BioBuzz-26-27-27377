package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Standalone Limelight check. Uses nothing from the rest of the robot code.
// D-pad UP / DOWN on gamepad 1 switches pipeline 0-9 so you can find which one sees the tag.
@TeleOp(name = "Limelight Test", group = "Test")
public class Limelighttest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(50);
        int pipeline = 0;
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        boolean lastUp = false, lastDown = false;

        while (!isStopRequested()) {
            if (gamepad1.dpad_up && !lastUp) {
                pipeline = Math.min(9, pipeline + 1);
                limelight.pipelineSwitch(pipeline);
            }
            if (gamepad1.dpad_down && !lastDown) {
                pipeline = Math.max(0, pipeline - 1);
                limelight.pipelineSwitch(pipeline);
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addLine("D-pad UP / DOWN = change pipeline");
            telemetry.addData("Requested pipeline", pipeline);
            telemetry.addData("Connected / running", limelight.isConnected() + " / " + limelight.isRunning());

            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("Camera pipeline", status.getPipelineIndex());
                telemetry.addData("Camera FPS", "%.1f", status.getFps());
            }

            LLResult result = limelight.getLatestResult();
            if (result == null) {
                telemetry.addData("Result", "null");
            } else {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Result pipeline", result.getPipelineIndex());
                telemetry.addData("Staleness ms", result.getStaleness());
                telemetry.addData("Tags seen", result.getFiducialResults().size());
                telemetry.addData("Botpose tag count", result.getBotposeTagCount());
            }
            telemetry.update();
        }
        limelight.stop();
    }
}