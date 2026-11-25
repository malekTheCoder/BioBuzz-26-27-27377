package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    public static double POLLEN_OPEN = 0.0;
    public static double POLLEN_CLOSED = 0.15;
    public static double NECTAR_OPEN = 0.4;
    public static double NECTAR_CLOSED = 0.2;

    private final Servo pollenServo;
    private final Servo nectarServo;

    public Gate(HardwareMap hardwareMap) {
        pollenServo = hardwareMap.get(Servo.class, "pollen_gate");
        pollenServo.setDirection(Servo.Direction.FORWARD);

        nectarServo = hardwareMap.get(Servo.class, "nectar_gate");
        nectarServo.setDirection(Servo.Direction.FORWARD);
        gateMovement(false);
    }

    public void pollenGate(boolean open) {
        pollenServo.setPosition(open ? POLLEN_OPEN : POLLEN_CLOSED);
    }

    public void nectarGate(boolean open) {
        nectarServo.setPosition(open ? NECTAR_OPEN : NECTAR_CLOSED);
    }

    public void gateMovement(boolean open) {
        pollenGate(open);
        nectarGate(open);
    }
}
