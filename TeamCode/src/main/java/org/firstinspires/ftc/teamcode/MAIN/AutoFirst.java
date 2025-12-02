package org.firstinspires.ftc.teamcode.MAIN;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.List;
//HELP
@Config
@Autonomous
public class AutoFirst extends LinearOpMode {

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public enum Alliance { RED, BLUE }
    public Alliance alliance = Alliance.RED;
    public static double x_test;
    public static double y_test;

    public static double windup1,windup2,windup3,shootwait,in1,in2,in3,wait1,wait2,gatetime,intakepow,outtakepow,shootpowclose,shootpowfar,hoodposition,winduppreload,outtaketime;

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

        public double gettx() {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get all visible AprilTags
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (fiducialResults != null) {
                    for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr : fiducialResults) {
                        int id = fr.getFiducialId();

                        // Example: only care about tag 4 for blue basket
                        if (id == 24) {
                            // Return the tx for *this* tag
                            return result.getTx();
                        }
                        if (id == 20){
                            return result.getTx();
                        }
                    }
                }
            }

            return Double.NaN;  // No blue tag visible
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

        public class Aim implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                autoShooter.turret.setPower(0.017 * (-autoShooter.gettx()));
                telemetry.addData("shooter aiming",autoShooter.gettx());
                dashboardTelemetry.addData("shooter aiming",autoShooter.gettx());
                return true;

            }


        }

        public Action aim() {return new Aim();}


        public class IntakeOn implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    intake.setPower(intakepow);
                    outtake_bottom.setPower(-outtakepow);
                    outtake_top.setPower(-outtakepow);

                    done = true;
                }
                return false; // we're done after setting it once
            }
        }
        public Action intakeOn() { return new IntakeOn(); }


        public class IntakeOff implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    intake.setPower(0);
                    outtake_bottom.setPower(0);
                    outtake_top.setPower(0);
                    done = true;
                }
                return false; // also a one-shot
            }
        }
        public Action intakeOff() { return new IntakeOff(); }



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

        public class Outtake implements Action{
            private final ElapsedTime shootTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t = shootTimer.milliseconds();
                if (t<750){
                    outtake_bottom.setPower(-outtakepow);
                    outtake_top.setPower(-outtakepow);
                    intake.setPower(-0.4);
                }else{
                    outtake_bottom.setPower(0);
                    outtake_top.setPower(0);
                    intake.setPower(0);
                    return false;
                }
                return true;
            }

        }
        public Action outtake() {
            return new Outtake();
        }



        public class Shooter_OnClose implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    outtake_bottom.setPower(shootpowclose);
                    outtake_top.setPower(shootpowclose);
                    hood.setPosition(hoodposition);

                    done = true;
                }
                return false;  // DONE immediately
            }
        }
        public Action shooter_OnClose() { return new Shooter_OnClose(); }

        public class Shooter_OnFar implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    outtake_bottom.setPower(shootpowfar);
                    outtake_top.setPower(shootpowfar);
                    hood.setPosition(hoodposition);

                    done = true;
                }
                return false;  // DONE immediately
            }
        }
        public Action shooter_OnFar() { return new Shooter_OnFar(); }




        public class IntakeForTime implements Action {
            private final double durationSec;
            private final double power;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            public IntakeForTime(double durationSec, double power) {
                this.durationSec = durationSec;
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    timer.reset();
                    intake.setPower(power);
                }

                if (timer.seconds() >= durationSec) {
                    intake.setPower(0);
                    return false;
                }

                return true;
            }
        }

        public Action intakeFor(double seconds, double power) {
            return new IntakeForTime(seconds, power);
        }

        public class OuttakeForTime implements Action {
            private final double durationSec;
            private final double power;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            public OuttakeForTime(double durationSec, double power) {
                this.durationSec = durationSec;
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    timer.reset();
                    intake.setPower(power);
                }

                if (timer.seconds() >= durationSec) {
                    intake.setPower(0);
                    return false;
                }

                return true;
            }
        }

        public Action outtakeFor(double seconds, double power) {
            return new OuttakeForTime(seconds, power);
        }






        public class Shooter_Off implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake_bottom.setPower(0);
                outtake_top.setPower(0);
                return false;
            }
        }
        public Action shooterOff() { return new Shooter_Off(); }


        public class Intake_Outtake implements Action{
            private final ElapsedTime shootTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t = shootTimer.milliseconds();
                if (t<100){
                    intake.setPower(1);
                }else{
                    intake.setPower(0);
                    return false;
                }
                return true;
            }

        }

        public Action intake_outtake() {
            return new Intake_Outtake();
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
        double shootpowfar =0.82;
        int shootwait =0;
        int wait1 = 1;
        int wait2 = 1;
        double windup1 = 0.5;
        double windup2 = 0.5;
        double windup3 = 0.5;
        double winduppreload = 2.5;
        int gatetime = 2;
        double hoodposition =0.4;
        double in1 =0.1;
        double in2 = 0.125;
        double in3 = 0.25;
        int intakepow = 1;
        double outtakepow = 0.4;
        double outtaketime = 0.2;
        double shootpowclose = 0.62;




        // Pre-start: choose alliance and aim turret
        int side = 0;
        int id =0;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = Alliance.RED;
                id = 24;
                side = 1;
            }

            if (gamepad1.right_bumper) {
                alliance = Alliance.BLUE;
                id = 20;
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

        Pose2d initialPose = new Pose2d(64, side*13, side*Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        // Wait for first valid Limelight reading before firing
        /*while (opModeIsActive() && !autoShooter.isTracking()) {
            AutoShooter.turret.setPower(0);
            telemetry.addData("Status", "Waiting for target...");
            telemetry.update();
        }*/

        /*TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(side * 20, 5), 0);*/



        TrajectoryActionBuilder intake1 = drive.actionBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(40, side*18, side*Math.toRadians(90)), side*Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(37, side*28, side*Math.toRadians(90)), side*Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(35, side*61, side*Math.toRadians(90)), side*Math.toRadians(100));
                //1 Outtake

                //2Intake

                //2 Outtake

                //3 Intake

                //3 Outtake

        //.splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(135)), Math.toRadians(270))
        TrajectoryActionBuilder outtake_path1 = intake1.endTrajectory().fresh()
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(18, side*22), side*Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(180));

        TrajectoryActionBuilder intake2 = outtake_path1.endTrajectory().fresh()
                .setTangent(0)
                //.splineToSplineHeading(new Pose2d(0, side*20, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, side*35, side*Math.toRadians(90)), side*Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(12, 61, Math.toRadians(90)), Math.toRadians(90));
                .splineToConstantHeading(new Vector2d(12, side*44),side* Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4, side*59), side*Math.toRadians(90));




        TrajectoryActionBuilder outtake_path2 = intake2.endTrajectory().fresh()
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(0, side*25), side*Math.toRadians(220))
                //.splineToSplineHeading(new Pose2d(0, 22, Math.toRadians(120)), Math.toRadians(220))
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(200));

        TrajectoryActionBuilder intake3 = outtake_path2.endTrajectory().fresh()
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(-11.2, side*26, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-11.2, side*55, side*Math.toRadians(90)), side*Math.toRadians(90));

        TrajectoryActionBuilder outtake_path3 = intake3.endTrajectory().fresh()
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(280));
        //.splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(135)), Math.toRadians(270))









        //.splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(135)), Math.toRadians(270))


                        // Run aiming continuously and shooting + movement in parallel
        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                                new ParallelAction(
                                    shooter.intakeFor(outtaketime,-0.5),
                                    shooter.outtakeFor(outtaketime, -0.5)
                                ),
                                shooter.shooter_OnFar(),
                                new SleepAction(winduppreload),
                                        shooter.intakeFor(in1,1),
                                        new SleepAction(wait1),
                                        shooter.intakeFor(in2,1),
                                        new SleepAction(wait2),
                                        shooter.intakeFor(in3,1),
                                        shooter.shooterOff()



                        ),
                new ParallelAction(
                        shooter.aim(),
                        new SequentialAction(

                                //first path
                                new ParallelAction(
                                        intake1.build(),
                                        shooter.intakeOn()
                                ),
                                shooter.intakeOff(),
                                new ParallelAction(
                                        outtake_path1.build(),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        shooter.intakeFor(outtaketime,-0.5),
                                                        shooter.outtakeFor(outtaketime, -0.5)

                                                ),


                                                //shooter.outtake(),
                                                shooter.shooter_OnClose()

                                        )

                                ),
                                new SleepAction(windup1),
                                shooter.intakeFor(in1,1),
                                new SleepAction(wait1),
                                shooter.intakeFor(in2,1),
                                new SleepAction(wait2),
                                shooter.intakeFor(in3,1),
                                shooter.shooterOff(),

                                //second path

                                new ParallelAction(
                                        intake2.build(),
                                        shooter.intakeOn()
                                ),

                                shooter.intakeOff(),
                                new SleepAction(gatetime),
                                new ParallelAction(
                                        outtake_path2.build(),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        shooter.intakeFor(outtaketime,-0.5),
                                                        shooter.outtakeFor(outtaketime, -0.5)

                                                ),


                                                //shooter.outtake(),
                                                shooter.shooter_OnClose()

                                        )

                                ),
                                new SleepAction(windup2),
                                shooter.intakeFor(in1,1),
                                new SleepAction(wait1),
                                shooter.intakeFor(in2,1),
                                new SleepAction(wait2),
                                shooter.intakeFor(in3,1),
                                shooter.shooterOff(),


                                //third path

                                new ParallelAction(
                                        intake3.build(),
                                        shooter.intakeOn()
                                ),

                                shooter.intakeOff(),
                                new ParallelAction(
                                        outtake_path3.build(),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        shooter.intakeFor(outtaketime,-0.5),
                                                        shooter.outtakeFor(outtaketime, -0.5)

                                                ),


                                                //shooter.outtake(),
                                                shooter.shooter_OnClose()

                                        )

                                ),
                                new SleepAction(windup3),
                                shooter.intakeFor(in1,1),
                                new SleepAction(wait1),
                                shooter.intakeFor(in2,1),
                                new SleepAction(wait2),
                                shooter.intakeFor(in3,1),
                                shooter.shooterOff()



                        )



                        /*new ParallelAction(
                                intake2.build(),
                                shooter.intake()

                        ),
                        outtake_path2.build(),
                        new ParallelAction(
                                shooter.intake(),
                                intake3.build()


                        ),
                        outtake_path3.build()*/


                /*new ParallelAction(
                        shooter.aimContinuous(),
                        new SequentialAction(
                                //shooter.shoot(),
                                path1.build()
                        )
                )*/
                        )
                )
        );



        while (opModeIsActive()) {
        }
    }
}

