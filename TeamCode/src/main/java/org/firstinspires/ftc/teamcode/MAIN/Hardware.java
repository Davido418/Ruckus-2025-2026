package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    //8Motors
    //Four Drivetrain
    //2 outtake motors
    //1 intake motor
    //1 turret
    public DcMotorEx FL;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx BR;
    public DcMotorEx outtake_top;
    public DcMotorEx outtake_bottom;
    public DcMotorEx intake;
    public DcMotorEx turret;
    public Servo hood;
    int P = 3;
    public int max_ticks_per_second = 2500;
    public double F = 32767/max_ticks_per_second;



    public Hardware(HardwareMap hardwareMap){
        FL = hardwareMap.get(DcMotorEx.class, "FL"); //CONTROL HUB
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.get(DcMotorEx.class, "FR");//CONTROL HUB
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL = hardwareMap.get(DcMotorEx.class, "BL"); //CONTROL HUB
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.get(DcMotorEx.class, "BR"); //CONTROL HUB
        BR.setDirection(DcMotorSimple.Direction.FORWARD);



        //FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //we dont need this for now
        intake = hardwareMap.get(DcMotorEx.class, "intake"); //EXPANSION HUB

        Hardware hardware = new Hardware(hardwareMap);
        AutoShooter autoShooter = new AutoShooter(hardwareMap);
        outtake_top = hardwareMap.get(DcMotorEx.class, "outtake_top"); //EXPANSION HUB
        outtake_top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake_bottom = hardwareMap.get(DcMotorEx.class, "outtake_bottom"); //EXPANSION HUB
        outtake_bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidf= new PIDFCoefficients(P,0,0,F);
        outtake_top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        outtake_bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }
}
