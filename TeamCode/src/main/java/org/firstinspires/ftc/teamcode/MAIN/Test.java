package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


    public static double P = 0.015;

    public  double distance;
    private final ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {timer.reset();

        AutoShooter autoShooter = new AutoShooter(hardwareMap);

//        Hardware hardware = new Hardware(hardwareMap);
//        AutoShooter autoShooter = new AutoShooter(hardwareMap);
        outtake_top = hardwareMap.get(DcMotorEx.class, "outtake_top"); //EXPANSION HUB
        hood = hardwareMap.get(Servo.class, "hood");
        outtake_top.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake_top.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
        outtake_bottom = hardwareMap.get(DcMotorEx.class, "outtake_bottom"); //EXPANSION HUB
        outtake_bottom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake_bottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class, "intake"); //EXPANSION HUB
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        PIDFCoefficients pidf= new PIDFCoefficients(P,0,0,F);
        outtake_top.setVelocityPIDFCoefficients(P, 0, 0, F);
        outtake_bottom.setVelocityPIDFCoefficients(P, 0, 0, F);
        outtake_bottom.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        outtake_top.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        hood.setPosition(pos);
        distance = AutoShooter.getDistanceFromLimelightToGoal();

        while(!opModeIsActive()){
            telemetry.addData("hi",timer.milliseconds());
            telemetry.addData("distance", distance);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {

            outtake_top.setVelocity(velocity);
            outtake_bottom.setVelocity(velocity);
            intake.setPower(0.4);

//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("velocity top", outtake_top.getVelocity());
//            packet.put("velocity bottom", outtake_bottom.getVelocity());
//            dash.sendTelemetryPacket(packet);
            telemetry.addData("velocity top", outtake_top.getVelocity());
            telemetry.addData("velocity bottom", outtake_bottom.getVelocity());
            //telemetry.addData("distance", distance);
            telemetry.addLine("Hi!");
            telemetry.update();
            //idle();
        }
        }
    }


//2400