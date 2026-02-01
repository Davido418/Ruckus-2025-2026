package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.tuning.LocalizationTest;


@TeleOp(name = "Tag Localization Test")
public class TagLocalizationTest extends LinearOpMode {
    //HardwareMap hardwareMap;
    public static Limelight3A limelight;
    public static double lastKnownPosX;
    public static double lastKnownPosY;
    public static double lastKnownYaw;
    ThreeDeadWheelLocalizer localizer;
    public double poseIncreaseX =0;
    public double poseIncreaseY =0;
    boolean ran = false;
    LLResult result;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new ThreeDeadWheelLocalizer(hardwareMap, localizer.inPerTick, new Pose2d(0,0,0));
        waitForStart();
        while (opModeIsActive()) {
            localizer.update();
            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addLine("result is valid");
                if (botpose != null) {
                    lastKnownPosX = botpose.getPosition().x;
                    lastKnownPosY = botpose.getPosition().y;
                    lastKnownYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addLine(botpose.getPosition().toString());
                    telemetry.addData("Status", "Tag Detected");
                    ran = true;
                }
            } else {
                if(ran = true) {
                    localizer.rebaseline(new Pose2d(0, 0, 0));
                    ran =false;
                }
                Pose2d pose = localizer.getPose();
                double thetaYaw = lastKnownYaw*Math.PI/180 - pose.heading.toDouble();
                poseIncreaseX = pose.position.x*Math.cos(thetaYaw)-pose.position.y*Math.sin(thetaYaw);
                poseIncreaseY = pose.position.x*Math.sin(thetaYaw)+pose.position.y*Math.cos(thetaYaw);
                telemetry.addData("Status", "No AprilTag Detected");
                telemetry.addData("WARNING", "This value is based on encoder position so it can vary.");
                lastKnownPosX+=poseIncreaseX;
                lastKnownPosY+=poseIncreaseY;
            }

            telemetry.addLine("Loop not stuck");
            telemetry.addData("x position", lastKnownPosX);
            telemetry.addData("y position", lastKnownPosY);
            telemetry.addData("Yaw", lastKnownYaw);
            telemetry.update();
        }
        }


        }



