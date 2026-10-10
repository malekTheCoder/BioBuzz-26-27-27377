package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private PIDFController controller;

    // Tune these in FTC Dashboard
    public static double kP = 0.02; // tune
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;

    public static double GEAR_RATIO = 1.0; // servo revs per turret rev, check this
    public static boolean ENCODER_REVERSED = false; // flip if angle goes the wrong way
    public static double MIN_ANGLE = -90; // degrees, soft limit so wires don't wrap
    public static double MAX_ANGLE = 90;
    public static double TOLERANCE = 1.0; // degrees
    public static double MAX_POWER = 0.8;
    public static boolean LIMELIGHT_ON_TURRET = true; // false if the limelight is mounted on the chassis

    private double lastServoAngle; // raw 0-360 reading from last loop
    private double servoRotations = 0; // full servo turns since start
    private double startServoAngle;
    private double targetAngle = 0;
    private boolean manual = false;
    private double lastFrameTime = -1;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretServo.setDirection(DcMotorSimple.Direction.FORWARD);

        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        controller = new PIDFController(kP, kI, kD, kF);

        // Turret must be facing forward (0 degrees) when the opmode starts
        lastServoAngle = readServoAngle();
        startServoAngle = lastServoAngle;
    }

    // Axon analog output: 0 - 3.3V maps to 0 - 360 degrees of servo rotation
    private double readServoAngle() {
        double angle = turretEncoder.getVoltage() / turretEncoder.getMaxVoltage() * 360.0;
        return ENCODER_REVERSED ? 360.0 - angle : angle;
    }

    // Call every loop so the encoder wraparound is tracked and the turret drives to its target
    public void update() {
        double servoAngle = readServoAngle();
        double delta = servoAngle - lastServoAngle;
        if (delta > 180) servoRotations--;
        else if (delta < -180) servoRotations++;
        lastServoAngle = servoAngle;

        if (manual) return;

        controller.setPIDF(kP, kI, kD, kF);
        if (atTarget()) {
            turretServo.setPower(0);
            return;
        }
        double power = controller.calculate(getAngle(), targetAngle);
        turretServo.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    public void setTargetAngle(double angle) {
        manual = false;
        targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    // tx = horizontal offset from the limelight, in degrees (positive = goal is to the right)
    public void aimWithTx(double tx) {
        // positive turret angle is counterclockwise, so a goal to the right means turning negative
        if (LIMELIGHT_ON_TURRET) setTargetAngle(getAngle() - tx);
        else setTargetAngle(-tx);
    }

    // Aims at the goal tag if it's visible. Returns false if it isn't.
    // The limelight is slower than the loop, so each frame's tx is only used once.
    // Reusing it would keep adding the same correction while the turret is already moving.
    private boolean aimWithGoalTag(LimelightEx limelight) {
        LLResultTypes.FiducialResult tag = limelight.getGoalTag();
        if (tag == null) return false;

        double frameTime = limelight.getResult().getTimestamp();
        if (frameTime != lastFrameTime) {
            lastFrameTime = frameTime;
            aimWithTx(tag.getTargetXDegrees());
        } else if (manual) {
            setTargetAngle(getAngle());
        }
        return true;
    }

    // Auto aim with the limelight. Call every loop. Holds position when the goal isn't in view
    public void autoAim(LimelightEx limelight) {
        if (!aimWithGoalTag(limelight) && manual) {
            setTargetAngle(getAngle());
        }
    }

    // Auto aim with the limelight, falling back to odometry when the goal isn't in view
    public void autoAim(LimelightEx limelight, Pose robotPose, Pose goal) {
        if (!aimWithGoalTag(limelight)) {
            aimAtPoint(robotPose, goal);
        }
    }

    // Points the turret at a field point using odometry.
    // 0 degrees = robot forward, positive = counterclockwise (same as pedro heading)
    public void aimAtPoint(Pose robotPose, Pose target) {
        double fieldAngle = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX());
        double relative = fieldAngle - robotPose.getHeading();
        relative = Math.atan2(Math.sin(relative), Math.cos(relative)); // wrap to -180..180
        setTargetAngle(Math.toDegrees(relative));
    }

    // Manual control, e.g. from a joystick. Won't drive past the soft limits
    public void setPower(double power) {
        manual = true;
        double angle = getAngle();
        if ((angle >= MAX_ANGLE && power > 0) || (angle <= MIN_ANGLE && power < 0)) power = 0;
        turretServo.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getAngle() {
        double servoDegrees = servoRotations * 360.0 + lastServoAngle - startServoAngle;
        return servoDegrees / GEAR_RATIO;
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getAngle()) < TOLERANCE;
    }

    public void stop() {
        manual = true;
        controller.reset();
        turretServo.setPower(0);
    }
}
