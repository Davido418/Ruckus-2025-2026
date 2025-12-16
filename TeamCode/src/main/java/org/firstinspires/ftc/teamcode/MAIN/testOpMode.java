package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Servo test = hardwareMap.get(Servo.class, "hood");

        waitForStart();
        while(opModeIsActive()) {
            test.setPosition(1);

        }

    }
}
