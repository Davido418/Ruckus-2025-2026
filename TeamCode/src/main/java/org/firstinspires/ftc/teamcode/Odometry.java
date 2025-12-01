package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Odometry {

    private MecanumDrive drive;

    public double getX() {
        drive.updatePoseEstimate();
        return drive.localizer.getPose().position.x;
    }

    public double getY() {
        drive.updatePoseEstimate();
        return drive.localizer.getPose().position.y;
    }

    public double getHeading() {
        drive.updatePoseEstimate();
        return drive.localizer.getPose().heading.toDouble();  // radians
    }
}
