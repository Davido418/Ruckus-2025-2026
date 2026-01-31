package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MAIN.Hardware;
import org.firstinspires.ftc.teamcode.MAIN.Constants;
import org.firstinspires.ftc.teamcode.tuning.LocalizationTest;


@TeleOp(name = "Tag Localization Test")
public class TagLocalizationTest extends LinearOpMode {
    //HardwareMap hardwareMap;
    public static Limelight3A limelight;
    public static IMU imu;
    public static double lastKnownPosX;
    public static double lastKnownPosY;
    public double poseIncreaseX =0;
    public double poseIncreaseY =0;
    LLResult result;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addLine("result is valid");
                if (botpose != null) {
                    lastKnownPosX = botpose.getPosition().x;;
                    lastKnownPosY = botpose.getPosition().y;
                    telemetry.addLine(botpose.getPosition().toString());
                    telemetry.addData("Status", "Tag Detected");
                    telemetry.addData("x position", lastKnownPosX);
                    telemetry.addData("y position", lastKnownPosY);
                    telemetry.addData("Yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                }
            } else {
                telemetry.addData("Status", "No AprilTag Detected");
                lastKnownPosX+=poseIncreaseX;
                lastKnownPosY+=poseIncreaseY;
            }

            telemetry.addLine("Loop not stuck");
            telemetry.update();
        }
        }


        }



