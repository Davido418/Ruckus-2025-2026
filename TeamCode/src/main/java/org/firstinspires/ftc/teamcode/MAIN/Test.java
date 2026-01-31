package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test")
@Config
public class Test extends LinearOpMode {
    public static double pos = 0.45;
    DcMotorEx outtake_top, outtake_bottom,intake;
    Servo hood;

    public static double velocity = 2000;

    public static int max_ticks_per_second = 2500;
    public static double F = 13.1068;
    CRServo servo1, servo2, servo3, servo4;


    public static double P = 0.015;

    public  double distance;
    private final ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {timer.reset();

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        servo4 = hardwareMap.get(CRServo.class, "servo4");

        waitForStart();

        while(opModeIsActive()) {

            servo1.setPower(1);
            servo2.setPower(1);
            servo3.setPower(1);
            servo4.setPower(1);
            //idle();
        }
        }
    }


//2400