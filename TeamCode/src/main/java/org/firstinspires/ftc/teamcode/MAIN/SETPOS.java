package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MAIN.Hardware;
import org.firstinspires.ftc.teamcode.MAIN.Constants;


@TeleOp(name = "SETPOS")
@Config
public class SETPOS extends LinearOpMode {

    Servo servo;
    AnalogInput encoder;
    public static double pos;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo"); //CONTROL HUB
        //axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");

        //telemetry.addData("ServoPosition",encoder.getPositionAndVelocity());

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(pos);
        }
        // Run until the op mode is stopped



        //drivetrain

    }
}


