package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    private Servo gateServo;

    public Gate(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.FORWARD);
    }

    public void gateMovement(boolean open) {
        if (open == true) {
            gateServo.setPosition(1); // choose this
        } else {
            gateServo.setPosition(0); // choose this
        }

    }
}