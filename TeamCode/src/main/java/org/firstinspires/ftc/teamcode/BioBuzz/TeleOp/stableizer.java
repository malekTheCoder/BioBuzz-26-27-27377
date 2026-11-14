package org.firstinspires.ftc.teamcode.BioBuzz.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.Subsystems.Drivetrain;

    @TeleOp
    public class stableizer extends OpMode {

        private Servo pollenGate, nectarGate;

        @Override
        public void init() {

            pollenGate = hardwareMap.get(Servo.class,"pollen_gate");
            nectarGate = hardwareMap.get(Servo.class,"nectar_gate");
        }

        @Override

        public void loop() {

              pollenGate.setPosition(0);
              nectarGate.setPosition(0);
        }
    }


