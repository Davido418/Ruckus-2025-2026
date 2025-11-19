package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ReturnDistance {
    HardwareMap hardwareMap;
    DcMotorEx turret;
    private Limelight3A limelight;
    double limelightMountAngleDegrees  = 15;
    double limelightLensHeightInches = 12.1;
    double goalHeightInches = 29.5;
    double angleToGoalDegrees;
    double angleToGoalRadians;

    double DistanceFromLimeLighttoGoalInches;



    private double distance;

    public ReturnDistance(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public double getDistanceFromLimelightToGoal(){
        LLResult result =limelight.getLatestResult();
        if(result != null && result.isValid()) {
            double targetOffsetAngle_Vertical = result.getTy(); // Get 'ty' equivalent
            angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            angleToGoalRadians = Math.toRadians(angleToGoalDegrees); // More robust than manual conversion
            DistanceFromLimeLighttoGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            return DistanceFromLimeLighttoGoalInches;
        } else {
            // No tag found, return -1 or another error value
            return -1;
        }
    }


}
