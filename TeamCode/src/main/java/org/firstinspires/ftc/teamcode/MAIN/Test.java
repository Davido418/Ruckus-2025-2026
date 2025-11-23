package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TurretTest")
@Config
public class Test extends LinearOpMode {



    int state = 0;
    double ood = 0;

    double power = 0;
    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);
        AutoShooter autoShooter = new AutoShooter(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            if (autoShooter.around()){
                hardware.turret.setTargetPosition(0);
                hardware.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.turret.setPower(0.4);
                sleep(500);
                //autoShooter.turret.setPower(0.014 * (-autoShooter.gettx()));

            }else{
                hardware.turret.setPower(0);
            }


            telemetry.addData("Pos", hardware.turret.getCurrentPosition());
            telemetry.update();
        }
    }
}

//2400