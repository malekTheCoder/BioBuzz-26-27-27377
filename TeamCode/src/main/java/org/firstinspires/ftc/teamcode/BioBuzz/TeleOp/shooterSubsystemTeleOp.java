package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.ShooterLeft;

@Config
@TeleOp
public class shooterSubsystemTeleOp extends LinearOpMode {

    // Target wheel speed in RPM (tune in dashboard)
    public static double TARGET_RPM = 5000;

        ShooterLeft leftShooter;

        @Override
        public void runOpMode() throws InterruptedException {

            leftShooter = new ShooterLeft(hardwareMap);

            waitForStart();

            while (opModeIsActive()){

                double targetTicks = TARGET_RPM * ShooterLeft.TICKS_PER_REV / 60.0; // RPM → ticks/sec
                if (TARGET_RPM > 0) {
                    leftShooter.updatePIDF();
                    leftShooter.setLeftVelocity(targetTicks);
                } else {
                leftShooter.stop();
            }

            double rpm = leftShooter.getLeftVelocity() * 60.0 / ShooterLeft.TICKS_PER_REV; // ticks/sec → RPM
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Velocity RPM", rpm);
            telemetry.addData("Target (ticks/s)", targetTicks);
            telemetry.addData("Velocity (ticks/s)", leftShooter.getLeftVelocity());
            telemetry.update();
        }
        leftShooter.stop();
    }
}
