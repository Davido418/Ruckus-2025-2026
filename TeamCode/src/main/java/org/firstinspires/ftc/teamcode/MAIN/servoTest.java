package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.MAIN.Hardware;
import org.firstinspires.ftc.teamcode.MAIN.Constants;


@TeleOp(name = "servoTestCR")
public class servoTest extends LinearOpMode {

CRServo servo;
Encoder encoder;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "Servo"); //CONTROL HUB
        encoder = hardwareMap.get(Encoder.class, "Encoder");
        servo.setPower(1);
        telemetry.addData("ServoPosition",encoder.getPositionAndVelocity());

        waitForStart();

        // Run until the op mode is stopped



            //drivetrain

        }
    }


