package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Turret;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {
    private static final double TICKS_PER_REV = 4000.0;

    // --- Tune these ---
    private static final String SERVO_NAME = "leftTurretServo"; // TODO: your real config name
    private static final boolean SERVO_REVERSED = true;
    private static final double GEAR_RATIO = 4.782;
    private static final double KP = 0.02;           // power per degree of error
    private static final double KD = 0.0005;         // light braking (was 0.002, too strong)
    private static final double BRAKE_ZONE_DEG = 20; // braking only applies this close to target
    private static final double MIN_POWER = 0.10;    // smallest power that moves the turret
    private static final double MAX_POWER = 0.5;
    private static final double DEADBAND_DEG = 1.5;  // stop zone
    private static final double RESTART_DEG = 3.0;   // must drift this far before it moves again
    private static final double VEL_FILTER = 0.3;

    // Stall fix: if it stops short, power slowly ramps up until it moves
    private static final double BOOST_STEP = 0.003;
    private static final double BOOST_MAX = 0.15;
    private static final double STALL_SPEED_DEG_S = 3.0;

    // Runaway guard
    private static final long GUARD_GRACE_MS = 700;
    private static final double GUARD_GROWTH_DEG = 15.0;

    private Double targetDeg = null;

    @Override
    public void runOpMode() {
        AnalogInput enc = hardwareMap.get(AnalogInput.class, Turret.LEFT_ENCODER_NAME);
        CRServo servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        servo.setDirection(SERVO_REVERSED ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Point turret FORWARD before pressing Start (that is 0 deg).");
        telemetry.addLine("Dpad: up=180, right=90, left=270, down=0. B = stop. A = re-zero here.");
        telemetry.update();
        waitForStart();

        double fullTurn = enc.getMaxVoltage();
        double lastRaw = rawTicks(enc, fullTurn);
        double accumulated = 0;
        double zeroOffset = 0;
        boolean lastA = false;

        double lastDeg = 0;
        long lastNanos = System.nanoTime();
        double velDegPerSec = 0;
        boolean settled = false;
        double boost = 0;

        Double guardTarget = null;
        double startError = 0;
        long startTime = 0;
        boolean tripped = false;

        while (opModeIsActive()) {
            double raw = rawTicks(enc, fullTurn);
            accumulated += wrapTicks(raw - lastRaw);
            lastRaw = raw;

            if (gamepad1.a && !lastA) {
                zeroOffset = accumulated;
            }
            lastA = gamepad1.a;

            if (gamepad1.dpad_up) targetDeg = 180.0;
            else if (gamepad1.dpad_right) targetDeg = 90.0;
            else if (gamepad1.dpad_left) targetDeg = 270.0;
            else if (gamepad1.dpad_down) targetDeg = 0.0;
            if (gamepad1.b) targetDeg = null;

            double encoderTicks = accumulated - zeroOffset;
            if (Turret.LEFT_ENCODER_REVERSED) encoderTicks = -encoderTicks;
            double encoderDeg = encoderTicks * 360.0 / TICKS_PER_REV;
            double turretDeg = encoderDeg / GEAR_RATIO;

            long now = System.nanoTime();
            double dt = (now - lastNanos) / 1e9;
            if (dt > 0) {
                double instVel = (turretDeg - lastDeg) / dt;
                velDegPerSec += VEL_FILTER * (instVel - velDegPerSec);
            }
            lastDeg = turretDeg;
            lastNanos = now;

            double power = 0;
            double error = 0;

            if (targetDeg != null) {
                error = targetDeg - turretDeg;

                if (guardTarget == null || !guardTarget.equals(targetDeg)) {
                    guardTarget = targetDeg;
                    startError = Math.abs(error);
                    startTime = System.currentTimeMillis();
                    tripped = false;
                    settled = false;
                    boost = 0;
                }

                if (System.currentTimeMillis() - startTime > GUARD_GRACE_MS
                        && Math.abs(error) > startError + GUARD_GROWTH_DEG) {
                    tripped = true;
                    targetDeg = null;
                    guardTarget = null;
                }
            } else {
                guardTarget = null;
                boost = 0;
            }

            if (targetDeg != null) {
                if (Math.abs(error) <= DEADBAND_DEG) { settled = true; boost = 0; }
                else if (Math.abs(error) > RESTART_DEG) settled = false;

                if (!settled) {
                    // Stalled short of target: ramp extra power until it moves again
                    if (Math.abs(velDegPerSec) < STALL_SPEED_DEG_S) {
                        boost = Math.min(BOOST_MAX, boost + BOOST_STEP);
                    } else {
                        boost = Math.max(0, boost - 0.01);
                    }

                    double dir = Math.signum(error);
                    double p = KP * error;
                    if (Math.abs(error) < BRAKE_ZONE_DEG) p -= KD * velDegPerSec;

                    double mag = dir * p;                     // power toward the target
                    double floor = MIN_POWER + boost;
                    if (mag < floor) mag = floor;
                    power = Range.clip(dir * mag, -MAX_POWER, MAX_POWER);
                }
            }
            servo.setPower(power);

            if (tripped) {
                telemetry.addLine("STOPPED: error was growing. Flip SERVO_REVERSED.");
            }
            telemetry.addData("Encoder revs", "%.3f", encoderDeg / 360.0);
            telemetry.addData("Turret angle", "%.1f deg", turretDeg);
            telemetry.addData("Target", targetDeg == null ? "none" : String.format("%.0f deg", targetDeg));
            telemetry.addData("Error", "%.1f deg", error);
            telemetry.addData("Speed", "%.0f deg/s", velDegPerSec);
            telemetry.addData("Boost", "%.3f", boost);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
            idle();
        }
        servo.setPower(0);
    }

    private static double rawTicks(AnalogInput enc, double fullTurn) {
        double v = Range.clip(enc.getVoltage(), 0, fullTurn);
        return v / fullTurn * TICKS_PER_REV;
    }

    private static double wrapTicks(double ticks) {
        double half = TICKS_PER_REV / 2.0;
        return (((ticks + half) % TICKS_PER_REV) + TICKS_PER_REV) % TICKS_PER_REV - half;
    }
}