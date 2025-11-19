package org.firstinspires.ftc.teamcode.MAIN;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Straight")
@Config
public class StraightAuto extends LinearOpMode {


    double sticky2;
    double distance;
    //RobotState currentState = RobotState.IDLE;

    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        AutoShooter autoShooter = new AutoShooter(hardwareMap);



        waitForStart();

        while (opModeIsActive()) {
            hardware.FL.setPower(0.25);
            hardware.BL.setPower(0.25);
            hardware.FR.setPower(0.25);
            hardware.BR.setPower(0.25);
        }
    }
}

//2400