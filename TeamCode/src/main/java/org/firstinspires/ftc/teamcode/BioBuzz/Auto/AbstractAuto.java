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
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.RobotActions;


public abstract class AbstractAuto extends LinearOpMode {
    protected final void update() {
        robot.run();
        robot.printTelemetry();
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);

        robot.actionScheduler.setUpdate(this::update);

        configure();

        // set the goal after we pick red or blue
        robot.turret.setAlliance();
        RobotActions.setGoal(Common.getAllianceGoal());

        onInit();

        if (isStopRequested()) return;

        waitForStart();

        resetRuntime();
        robot.drivetrain.setPose(getStartPose());

        onRun();
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
