package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ShooterTest {
    HardwareMap hardwareMap;
    static DcMotorEx turret;

    public static int alliance2 = 5;
    private static Limelight3A limelight;

    static double limelightMountAngleDegrees  = 15;
    static double limelightLensHeightInches   = 12.1;
    static double goalHeightInches            = 29.5;

    public ShooterTest(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ---------------- LIMELIGHT HELPERS ----------------

    public double gettx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) return result.getTx();
        return 0;
    }

    public boolean isTracking() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public static double getDistanceFromLimelightToGoal(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(limelightMountAngleDegrees + ty);
            return (goalHeightInches - limelightLensHeightInches) / Math.tan(angle);
        }
        return -1;
    }

    public double gettxBlue() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == 5)
                        return result.getTx();
                }
            }
        }
        return 0;
    }

    public double gettxRed() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == 4)
                        return result.getTx();
                }
            }
        }
        return 0;
    }

    // ---------------- POWER COMPUTATION ----------------

    public double getPower() {
        if (alliance2 == 5 && isTracking()) {
            return 0.014 * -gettxRed();
        } else {
            return 0.014 * gettxBlue();
        }
    }

    // ---------------- MAIN UPDATE FUNCTION ----------------

    public void update() {
        ShooterTest.turret.setPower(getPower());


    }


}
