package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTest extends LinearOpMode {

    // ===== DASHBOARD-TUNABLE CONSTANTS =====

    // Set this from Dashboard
    public static double TARGET_RPM = 0.0;

    // CHANGE this to your actual ticks-per-rev * gear ratio
    public static double TICKS_PER_REV = 28.0;

    // PID gains (tune live)
    public static double kP = 0.0002;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Feedforward gains (tune live)
    public static double kS = 0.05;
    public static double kV = 0.00003;

    // ===== END CONFIG =====

    private VelocityPIDF shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new VelocityPIDF(
                hardwareMap,
                kP, kI, kD,
                kS, kV
        );

        telemetry.addLine("ShooterTest (Dashboard)");
        telemetry.addLine("Set TARGET_RPM, kP, kS, kV from Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update gains live (Dashboard -> code)
            shooter.setPID(kP, kI, kD);
            shooter.setFeedforward(kS, kV);

            // Set velocity target from Dashboard
            shooter.setTargetVelocityRPM(TARGET_RPM, TICKS_PER_REV);

            // Run controller
            shooter.update();

            // Telemetry
            double targetTPS = shooter.getTargetVelocityTPS();
            double actualTPS = shooter.getCurrentVelocityTPS();
            double actualRPM = VelocityPIDF.ticksPerSecondToRPM(actualTPS, TICKS_PER_REV);

            telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
            telemetry.addData("Target TPS", "%.1f", targetTPS);
            telemetry.addData("Actual TPS", "%.1f", actualTPS);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);

            telemetry.update();
        }

        shooter.stop();
    }
}
