package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.ShooterLeft;

@Config
@TeleOp
public class shooterSubsystemTeleOp extends LinearOpMode {

    // Target wheel speed in RPM (tune in dashboard)

        ShooterLeft leftShooter;
        public static double TARGET_RPM = 5000;
        public static double rpm;
        public static double targetTicks;
        public static double ticks;

        @Override
        public void runOpMode() throws InterruptedException {

            leftShooter = new ShooterLeft(hardwareMap);
            leftShooter.
            Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

            waitForStart();

            while (opModeIsActive()){

                targetTicks = TARGET_RPM * ShooterLeft.TICKS_PER_REV / 60.0; // RPM → ticks/sec
                if (gamepad1.a && TARGET_RPM > 0) {
                    leftShooter.updatePIDF();
                    leftShooter.setLeftVelocity(targetTicks);
                } else {
                leftShooter.stop();
            }

            rpm = leftShooter.getLeftVelocity() * 60.0 / ShooterLeft.TICKS_PER_REV; // ticks/sec → RPM
            ticks = leftShooter.getLeftVelocity();

            dashboardTelemetry.addData("Target RPM", TARGET_RPM);
            dashboardTelemetry.addData("Velocity RPM", rpm);
            dashboardTelemetry.addData("Target (ticks/s)", targetTicks);
            dashboardTelemetry.addData("Velocity (ticks/s)", ticks);
            dashboardTelemetry.update();
        }
        leftShooter.stop();
    }
}
