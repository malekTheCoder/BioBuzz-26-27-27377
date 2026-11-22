package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;
    private DcMotor intakeMotor2;



    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class,"transfer");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

    }

    // FIXED: Now actually uses the power you pass ingdfs
    public void intakeElements(double power) {
        intakeMotor2.setPower(power);
        intakeMotor.setPower(power);
    }


    public void stop() {
        intakeMotor2.setPower(0);
        intakeMotor.setPower(0);
    }
}