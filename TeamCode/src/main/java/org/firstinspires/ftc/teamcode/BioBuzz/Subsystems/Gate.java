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

    public void openGate() {
        gateServo.setPosition(1);
    }

    public void closeGate() {
        gateServo.setPosition(0);
    }
}