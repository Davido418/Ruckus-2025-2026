package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "PIDF tuner RL")
    @Config

    public class RL_VelocityPIDF extends LinearOpMode{
        Hardware hardware = new Hardware(hardwareMap);
        AutoShooter autoShooter = new AutoShooter(hardwareMap);

        public static double P = 2;
        public static double I = 0;
        public static double D = 0;
        public static double max_ticks_per_sec = 5400; //We need to tune
        public static double F = 32767/max_ticks_per_sec;
        public static double velocity = 650;

        @Override



        public void runOpMode() throws InterruptedException {
            PIDFCoefficients pidfNewBottom = new PIDFCoefficients(P, I, D, F);
            PIDFCoefficients pidfNewTop = new PIDFCoefficients(P, I, D, F);
            hardware.outtake_bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewBottom);
            hardware.outtake_top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewTop);
            waitForStart();
            hardware.outtake_bottom.setVelocity(velocity);
            hardware.outtake_top.setVelocity(velocity);
            telemetry.addData("outtake top Velocity in t/s", hardware.outtake_top.getVelocity());
            telemetry.addData("outtake bottom Velocity in t/s", hardware.outtake_bottom.getVelocity());
            telemetry.update();
        }
    }


