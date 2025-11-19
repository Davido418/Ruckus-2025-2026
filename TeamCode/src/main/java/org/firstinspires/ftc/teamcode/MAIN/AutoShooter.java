package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class AutoShooter {
    HardwareMap hardwareMap;
    DcMotorEx turret;
    int alliance =5;
    private static Limelight3A limelight;

    static double limelightMountAngleDegrees  = 15;
    static double limelightLensHeightInches = 12.1;
    static double goalHeightInches = 29.5;
    static double angleToGoalDegrees;
    static double angleToGoalRadians;

    static double DistanceFromLimeLighttoGoalInches;

    public AutoShooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public double gettx() {

        LLResult result = limelight.getLatestResult();
        double lastResult = result.getTx();
        if (result != null) {
            if (result.isValid()) {
                return result.getTx();
            }
        }
        return 0;
    }
    public double getTxBlue() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get all visible AprilTags
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (fiducialResults != null) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr : fiducialResults) {
                    int id = fr.getFiducialId();

                    // Example: only care about tag 4 for blue basket
                    if (id == 4) {
                        // Return the tx for *this* tag
                        return result.getTx();
                    }
                }
            }
        }

        return Double.NaN;  // No blue tag visible
    }
    public double getTxRed() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get all visible AprilTags
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (fiducialResults != null) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr : fiducialResults) {
                    int id = fr.getFiducialId();

                    // Example: only care about tag 4 for blue basket
                    if (id == 5) {
                        // Return the tx for *this* tag
                        return result.getTx();
                    }
                }
            }
        }

        return Double.NaN;  // No blue tag visible
    }



    public boolean isTracking() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return true;
            }
        }
        return false;
    }


    public static double getDistanceFromLimelightToGoal(){
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
    public double gettxBlue() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == 5) return result.getTx();
                }
            }
        }
        return Double.NaN;
    }
    public double gettxRed() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == 4) return result.getTx();
                }
            }
        }
        return Double.NaN;
    }
    public double getTurretPosition() {
        double value = turret.getCurrentPosition();
        return value;
    }





}
