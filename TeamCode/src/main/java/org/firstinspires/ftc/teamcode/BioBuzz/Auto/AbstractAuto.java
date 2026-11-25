package org.firstinspires.ftc.teamcode.BioBuzz.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common.robot;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Common;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Robot;


public abstract class AbstractAuto extends LinearOpMode {
    private long lastTurretTelemetryNanos;

    protected final void update() {
        robot.run();
        long now = System.nanoTime();
        if (now - lastTurretTelemetryNanos >= 100_000_000L) {
            robot.printTelemetry();
            Pose pose = robot.drivetrain.getPose();
            dashTelemetry.addData("Auto pose", "%.1f, %.1f, %.1f deg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            dashTelemetry.update();
            lastTurretTelemetryNanos = now;
        }
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);

        robot.actionScheduler.setUpdate(this::update);

        configure();

        robot.turret.setAlliance();

        onInit();

        if (isStopRequested()) return;

        waitForStart();
        if (isStopRequested()) return;

        resetRuntime();
        robot.drivetrain.setPose(getStartPose());

        onRun();
        while (opModeIsActive()) {
            update();
            idle();
        }
        Common.AUTO_END_POSE = robot.drivetrain.getPose();
    }

    protected void onInit() {
        robot.drivetrain.setPose(getStartPose());
    }

    protected void configure() {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            dashTelemetry.addLine("PRESS A TO TOGGLE SIDES");
            dashTelemetry.addData("IS RED: ", Common.isRed);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) Common.isRed = !Common.isRed;

            dashTelemetry.update();
        }
    }
    protected abstract Pose getStartPose();
    protected abstract void onRun();
}