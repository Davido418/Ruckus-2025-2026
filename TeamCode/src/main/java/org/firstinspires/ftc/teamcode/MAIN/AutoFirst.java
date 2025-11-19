package org.firstinspires.ftc.teamcode.MAIN;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.List;

@Config
@Autonomous
public class AutoFirst extends LinearOpMode {

    public enum Alliance { RED, BLUE }
    public Alliance alliance = Alliance.RED;

    public static class AutoShooter {
        HardwareMap hardwareMap;
        public static DcMotorEx turret;
        private static Limelight3A limelight;
        static double limelightMountAngleDegrees = 15;
        static double limelightLensHeightInches = 12.1;
        static double goalHeightInches = 29.5;

        public AutoShooter(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        }

        public boolean isTracking() {
            LLResult result = limelight.getLatestResult();
            return result != null && result.isValid();
        }

        public double getTxBlue() {
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

        public double getTxRed() {
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
    }

    public class Shooter {
        private DcMotorEx outtake_top, outtake_bottom, intake;
        private Servo hood;
        private AutoShooter autoShooter;

        public Shooter(HardwareMap hardwareMap, AutoShooter autoShooter) {
            this.autoShooter = autoShooter;
            outtake_top = hardwareMap.get(DcMotorEx.class, "outtake_top");
            outtake_bottom = hardwareMap.get(DcMotorEx.class, "outtake_bottom");
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            hood = hardwareMap.get(Servo.class, "hood");
        }

        public class AimContinuous implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (autoShooter.isTracking()) {
                    double tx = (alliance == Alliance.BLUE) ? autoShooter.getTxBlue() : autoShooter.getTxRed();
                    if (!Double.isNaN(tx)) {
                        double power = 0.014 * -tx;
                        if (Math.abs(power) < 0.02) power = 0; // deadzone to prevent jitter
                        AutoShooter.turret.setPower(power);
                    } else {
                        AutoShooter.turret.setPower(0);
                    }
                } else {
                    AutoShooter.turret.setPower(0);
                }
                return true; // KEEP RUNNING FOREVER
            }
        }

        public Action aimContinuous() {
            return new AimContinuous();
        }

        public class Shoot implements Action {
            private final ElapsedTime shootTimer = new ElapsedTime();
            private int stage = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t = shootTimer.milliseconds();
                hood.setPosition(0.495);

                switch (stage) {
                    case 0:
                        intake.setPower(-0.3);
                        if (t > 500) {
                            intake.setPower(0);
                            outtake_bottom.setPower(0.755);
                            outtake_top.setPower(0.755);
                            shootTimer.reset();
                            stage = 1;
                        }
                        break;
                    case 1:
                        if (t > 3000) {
                            intake.setPower(1);
                            shootTimer.reset();
                            stage = 2;
                        }
                        break;
                    case 2:
                        if (t > 100) {
                            intake.setPower(0);
                            shootTimer.reset();
                            stage = 3;
                        }
                        break;
                    case 3:
                        if (t > 2500) {
                            intake.setPower(1);
                            shootTimer.reset();
                            stage = 4;
                        }
                        break;
                    case 4:
                        if (t > 150) {
                            intake.setPower(0);
                            shootTimer.reset();
                            stage = 5;
                        }
                        break;
                    case 5:
                        if (t > 2500) {
                            intake.setPower(1);
                            shootTimer.reset();
                            stage = 6;
                        }
                        break;
                    case 6:
                        if (t > 300) {
                            intake.setPower(0);
                            outtake_bottom.setPower(0);
                            outtake_top.setPower(0);
                            return false;
                        }
                        break;
                }
                return true;
            }
        }

        public Action shoot() {
            return new Shoot();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutoShooter autoShooter = new AutoShooter(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, autoShooter);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Pre-start: choose alliance and aim turret
        int side = 0;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = Alliance.RED;
                side = 1;
            }

            if (gamepad1.right_bumper) {
                alliance = Alliance.BLUE;
                side = -1;
            }


            if (autoShooter.isTracking()) {
                double tx = (alliance == Alliance.BLUE) ? autoShooter.getTxBlue() : autoShooter.getTxRed();
                if (!Double.isNaN(tx)) {
                    double power = 0.014 * -tx;
                    if (Math.abs(power) < 0.02) power = 0;
                    AutoShooter.turret.setPower(power);
                }
            } else {
                AutoShooter.turret.setPower(0);
            }

            telemetry.addData("Alliance", alliance);
            telemetry.update();
        }

        waitForStart();

        // Wait for first valid Limelight reading before firing
        while (opModeIsActive() && !autoShooter.isTracking()) {
            AutoShooter.turret.setPower(0);
            telemetry.addData("Status", "Waiting for target...");
            telemetry.update();
        }

        /*TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(side * 20, 5), 0);*/
        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, 40), 0);

        // Run aiming continuously and shooting + movement in parallel
        Actions.runBlocking(
                new SequentialAction(
                        path1.build()

                )
                /*new ParallelAction(
                        shooter.aimContinuous(),
                        new SequentialAction(
                                //shooter.shoot(),
                                path1.build()
                        )
                )*/
        );

        while (opModeIsActive()) {
        }
    }
}
