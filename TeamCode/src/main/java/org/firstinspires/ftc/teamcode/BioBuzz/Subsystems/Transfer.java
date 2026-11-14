package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    private DcMotor transferMotor;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loadElements(double power) {
        transferMotor.setPower(power);
    }


    public void stop() {
        transferMotor.setPower(0);
    }
}

