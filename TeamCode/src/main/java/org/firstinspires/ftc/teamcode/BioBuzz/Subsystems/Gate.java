package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class Gate {

    private Servo gateServo;

    ElapsedTime timer = new ElapsedTime();
    public Gate(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.FORWARD);
    }

    public void openGate(float time) {
        gateServo.setPosition(1);
        timer.reset();
        if (timer.seconds() >= time) {
            gateServo.setPosition(0);
        }
    }

    public void closeGate() {
        gateServo.setPosition(0);
    }
}