package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "liftTest")
@Config
public class liftTest extends LinearOpMode{
    CRServo liftL, liftR;
    public static double power = 1;
    public static boolean LForward = true;
    public static boolean RForward = true;

    @Override
    public void runOpMode() throws InterruptedException {
        liftL = hardwareMap.get(CRServo.class, "liftL");
        liftR = hardwareMap.get(CRServo.class, "liftR");
        waitForStart();
        liftL.setDirection(CRServo.Direction.FORWARD);
        liftR.setDirection(CRServo.Direction.FORWARD);
        while(opModeIsActive()){
        liftR.setPower(power);
        liftL.setPower(power);
        if(LForward = false) {
            liftL.setDirection(CRServo.Direction.REVERSE);
        }
        if(RForward = false) {
            liftR.setDirection(CRServo.Direction.REVERSE);
            }
        }
    }
}

