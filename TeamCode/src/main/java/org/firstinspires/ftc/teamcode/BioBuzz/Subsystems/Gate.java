package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    private Servo pollenServo;
    private Servo nectarServo;

    public Gate(HardwareMap hardwareMap) {
        pollenServo = hardwareMap.get(Servo.class, "pollen_gate");
        pollenServo.setDirection(Servo.Direction.FORWARD);

        nectarServo = hardwareMap.get(Servo.class, "nectar_gate");
        nectarServo.setDirection(Servo.Direction.FORWARD);
    }

    public void pollenGate(boolean open) {
        if (open == true) {
            pollenServo.setPosition(0.3); // choose this
        } else {
            pollenServo.setPosition(0); // choose this
        }
    }

    public void nectarGate(boolean open) {
        if (open == true) {
            nectarServo.setPosition(0.3); // choose this
        } else {
            nectarServo.setPosition(0); // choose this
        }
    }

    public void gateMovement(boolean open) {
        pollenGate(open);
        nectarGate(open);
    }
    // CHANGE: same name as before, so auto doesn't change. Now moves BOTH gates. This also
    // fixes the missing "}" that was hidden inside a comment.
}