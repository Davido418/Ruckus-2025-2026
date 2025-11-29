package org.firstinspires.ftc.teamcode.MAIN;
<<<<<<< HEAD

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
=======
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
>>>>>>> d6f3bfe9d7c3527f97823f9ccae4549e8feb14f0

@Config
@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTest extends LinearOpMode {

<<<<<<< HEAD
    // ===== DASHBOARD-TUNABLE CONSTANTS =====

    // Set this from Dashboard
    public static double TARGET_RPM = 0.0;

    // CHANGE this to your actual ticks-per-rev * gear ratio
    public static double TICKS_PER_REV = 28.0;

    // PID gains (tune live)
    public static double kP = 0.0002;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Feedforward gains (tune live)
    public static double kS = 0.05;
    public static double kV = 0.00003;

    // ===== END CONFIG =====

    private VelocityPIDF shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new VelocityPIDF(
                hardwareMap,
                kP, kI, kD,
                kS, kV
        );

        telemetry.addLine("ShooterTest (Dashboard)");
        telemetry.addLine("Set TARGET_RPM, kP, kS, kV from Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update gains live (Dashboard -> code)
            shooter.setPID(kP, kI, kD);
            shooter.setFeedforward(kS, kV);

            // Set velocity target from Dashboard
            shooter.setTargetVelocityRPM(TARGET_RPM, TICKS_PER_REV);

            // Run controller
            shooter.update();

            // Telemetry
            double targetTPS = shooter.getTargetVelocityTPS();
            double actualTPS = shooter.getCurrentVelocityTPS();
            double actualRPM = VelocityPIDF.ticksPerSecondToRPM(actualTPS, TICKS_PER_REV);

            telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
            telemetry.addData("Target TPS", "%.1f", targetTPS);
            telemetry.addData("Actual TPS", "%.1f", actualTPS);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);

            telemetry.update();
        }

        shooter.stop();
    }
}
=======

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

    public static double velocity;
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
        hardware.outtake_bottom.setVelocity(velocity);
        hardware.outtake_top.setVelocity(velocity);

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




>>>>>>> d6f3bfe9d7c3527f97823f9ccae4549e8feb14f0
