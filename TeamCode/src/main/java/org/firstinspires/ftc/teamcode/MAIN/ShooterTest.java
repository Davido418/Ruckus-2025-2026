package org.firstinspires.ftc.teamcode.MAIN;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MAIN.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;



import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@TeleOp(name = "ShooterTest")
@Config
public class ShooterTest extends LinearOpMode {
    public double close1_pos, close2_pos, far1_pos;
    CRServo hood2;
    public  double angle;
    ElapsedTime timer = new ElapsedTime();


    public  double pow_out;
    public static double pos;
    public  double pow_int;
    int state = 0;
    public  double drive;
    public static int timer1, timer2,timer3,timer4,timer5,timer6,timer7,timeint,timerwindup;
    public double distance1;
    public static double distance;
    public static double power,intpower,outpower;

    OverflowEncoder par0, par1, perp;
    public static double pow = 0.8 ;


    public double shooter_pow_c1, shooter_pow_c2, shooter_pow_f1;

    @Override
    public void runOpMode() throws InterruptedException {

        // Run until the op mode is stopped
        Hardware hardware = new Hardware(hardwareMap);
        AutoShooter autoShooter = new AutoShooter(hardwareMap);

        //hardware.intake.setPower(1);

        hardware.hood.setPosition(pos);

        /*distance1 =AutoShooter.getDistanceFromLimelightToGoal();
        telemetry.addData("Distance", distance1);
        telemetry.update();

            power = 0.0031110124333925586*distance1 + 0.5071305506216692;
            hardware.hood.setPosition(0.35);


           /* hardware.outtake_bottom.setPower(-0.2);
            hardware.outtake_top.setPower(-0.2);
            hardware.intake.setPower(0.45);
            sleep(2000);
            hardware.intake.setPower(-0.2);
            hardware.outtake_bottom.setPower(power);
            hardware.outtake_top.setPower(power);
            sleep(3000);*/
                /*hardware.intake.setPower(0.7);
            hardware.outtake_bottom.setPower(-0.5);
            hardware.outtake_top.setPower(-0.5);
            sleep(5000);
        hardware.intake.setPower(-0.5);
        sleep(500);
        hardware.intake.setPower(0);



        hardware.outtake_bottom.setPower(power);
        hardware.outtake_top.setPower(power);
        sleep(2000);

            hardware.intake.setPower(1);
            sleep(100);
        hardware.intake.setPower(0);
        sleep(2000);
        hardware.intake.setPower(1);
        sleep(150);
        hardware.intake.setPower(0);
        sleep(2000);
        hardware.intake.setPower(1);
        sleep(300);
        hardware.intake.setPower(0);
        sleep(2000);
        hardware.outtake_bottom.setPower(0);
        hardware.outtake_top.setPower(0);*/







        waitForStart();
        if(opModeIsActive()){
            if(autoShooter.isTracking()){
                

        }


            //while(opModeIsActive()){


            //}

            //with full intake power;
            //telemetry.update();

        }


    }





    }




