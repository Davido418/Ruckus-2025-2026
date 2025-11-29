package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityPIDF {

    // Hardware (paired shooter motors)
    private final DcMotorEx outtakeTop;
    private final DcMotorEx outtakeBottom;

    // PID gains
    private double kP;
    private double kI;
    private double kD;

    // Feedforward gains (from regression / characterization)
    // power ≈ kS * sign(velocity) + kV * velocity
    private double kS;
    private double kV;
    // Optional acceleration term if you want: private double kA = 0;

    // State
    private double targetVelocityTPS = 0;  // target in ticks per second
    private double integral = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Limits
    private double maxIntegral = 1.0; // anti-windup clamp

    public VelocityPIDF(HardwareMap hardwareMap,
                        double kP, double kI, double kD,
                        double kS, double kV) {
        this.outtakeTop = hardwareMap.get(DcMotorEx.class, "outtake_top");
        this.outtakeBottom = hardwareMap.get(DcMotorEx.class, "outtake_bottom");

        // Configure motors as needed
        outtakeTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // If your motors are mirrored, reverse one of them here
        // outtakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;

        timer.reset();
    }

    // ---- Public API ----

    /** Set target velocity in ticks per second (TPS). */
    public void setTargetVelocityTPS(double targetTPS) {
        this.targetVelocityTPS = targetTPS;
    }

    /** Helper: set target velocity in RPM, given ticks-per-rev of your motor. */
    public void setTargetVelocityRPM(double targetRPM, double ticksPerRev) {
        this.targetVelocityTPS = rpmToTicksPerSecond(targetRPM, ticksPerRev);
    }

    /** Call every loop. This will read the current velocity, compute PIDF, and set power. */
    public void update() {
        double dt = timer.seconds();
        timer.reset();

        // Current velocity: average both motors’ velocities (in ticks/sec from SDK)
        double currentTop = outtakeTop.getVelocity();
        double currentBottom = outtakeBottom.getVelocity();
        double currentVelocity = (currentTop + currentBottom) * 0.5;

        // PID error
        double error = targetVelocityTPS - currentVelocity;

        // Integral with basic anti-windup
        integral += error * dt;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;

        // Derivative
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        // PID output
        double pidOutput = kP * error + kI * integral + kD * derivative;

        // Feedforward (kS * sign(v) + kV * v)
        double ff = 0;
        if (Math.abs(targetVelocityTPS) > 1e-3) {
            ff = kS * Math.signum(targetVelocityTPS) + kV * targetVelocityTPS;
        }

        // Total power
        double power = pidOutput + ff;

        // Clip to [-1, 1]
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply to both motors (they always run together)
        outtakeTop.setPower(power);
        outtakeBottom.setPower(power);
    }

    /** Stop motors and reset PID state. */
    public void stop() {
        targetVelocityTPS = 0;
        integral = 0;
        lastError = 0;
        outtakeTop.setPower(0);
        outtakeBottom.setPower(0);
    }

    // ---- Optional tuning helpers ----

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    public double getTargetVelocityTPS() {
        return targetVelocityTPS;
    }

    public double getCurrentVelocityTPS() {
        return (outtakeTop.getVelocity() + outtakeBottom.getVelocity()) * 0.5;
    }

    // ---- Unit conversion helpers ----

    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    public static double ticksPerSecondToRPM(double tps, double ticksPerRev) {
        return (tps * 60.0) / ticksPerRev;
    }
}
