package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PTuner extends LinearOpMode {
    public static double kP, targetPosition;
    @Override
    public void runOpMode() throws InterruptedException {
        AutoShooter autoShooter= new AutoShooter(hardwareMap);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            if (autoShooter.gettx() != 420) {
                if (kP * (targetPosition - autoShooter.gettx()) < 0.2)
                    turret.setPower(0.01 * (targetPosition - autoShooter.gettx()));
            }
            telemetry.addData("currentPosition", autoShooter.gettx());
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("power", kP * (targetPosition - autoShooter.gettx()));

            telemetry.update();
        }
    }
}
