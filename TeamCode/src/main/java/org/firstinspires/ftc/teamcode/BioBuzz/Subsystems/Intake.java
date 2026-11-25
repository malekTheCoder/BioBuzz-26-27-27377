package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intakeElements(double power) {
        intakeMotor.setPower(power);
    }


    public void stop() {
        intakeMotor.setPower(0);
    }
}
