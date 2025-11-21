package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RuckusTeleOp")
@Config
public class Main extends LinearOpMode {


    enum RobotState {
        IDLE,
        INTAKING,
        OUTTAKING,
        SHOOTING,
        ACC
    }
    int state = 0;
    double ood = 0;

    double power = 0;
    int position = 0;
    public static int time;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    double sticky2;
    double distance;
    RobotState currentState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        AutoShooter autoShooter = new AutoShooter(hardwareMap);




        waitForStart();

        while (opModeIsActive()) {
            // copy gamepad states
            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            autoShooter.turret.setPower(0.014 * (-autoShooter.gettx()));
            distance = AutoShooter.getDistanceFromLimelightToGoal();

            if (autoShooter.isTracking()) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                telemetry.addData("Distance", AutoShooter.getDistanceFromLimelightToGoal());

            } else {
                gamepad1.stopRumble();
                gamepad2.stopRumble();
            }

            if (29.5<=distance && distance<=43.5){
                hardware.hood.setPosition(0.35);
            }
            if((43.5< distance) &&(distance<46)){
                hardware.hood.setPosition(0.45);
            }

            if (distance>=46 && distance<=82){
                hardware.hood.setPosition(0.45);
            }
            if(distance > 82){
                hardware.hood.setPosition(0.53);
            }



            // --- DRIVE CONTROL ---
            double x = currentGamepad1.left_stick_x;
            double y = -currentGamepad1.left_stick_y;
            double r = 0.8 * currentGamepad1.right_stick_x;

            double speedGain = (gamepad1.left_trigger < 0.5) ? 2.0 : 1.0;

            // Parabolic smoothing
            x = (Math.cosh(x * Math.sqrt(Math.sqrt(Math.pow(x, 2)))) - 1) * Math.signum(x);
            y = (Math.cosh(y * Math.sqrt(Math.sqrt(Math.pow(y, 2)))) - 1) * Math.signum(y);
            x *= speedGain;
            y *= speedGain;
            x = Math.max(-1, Math.min(1, x));
            y = Math.max(-1, Math.min(1, y));

            hardware.FL.setPower(y + x + r);
            hardware.FR.setPower(y - x - r);
            hardware.BL.setPower(y - x + r);
            hardware.BR.setPower(y + x - r);


            if (currentGamepad2.dpad_up) {
                hardware.hood.setPosition(0.70);
            }
            if (currentGamepad2.dpad_down) {
                hardware.hood.setPosition(0.1);
            }


            currentState = RobotState.IDLE;

            // --- Intake logic ---
            if (currentGamepad2.right_bumper || currentGamepad1.right_bumper) {
                hardware.intake.setPower(1);
                hardware.outtake_bottom.setPower(-0.2);
                hardware.outtake_top.setPower(-0.2);


            } else if (currentGamepad2.a || currentGamepad1.a) {
                hardware.intake.setPower(-0.5);
                hardware.outtake_bottom.setPower(-0.2);
                hardware.outtake_top.setPower(-0.2);
            } else {
                hardware.intake.setPower(0);
            }


// --- Outtake / Shooter logic ---
            if (currentGamepad2.left_bumper && !prevGamepad2.left_bumper) {
                timer.reset();
            }

            if (currentGamepad2.left_bumper || currentGamepad1.left_bumper) {
                timer2.reset();

                if (29.5<=distance && distance<=46){

                    //power = 0.0031110124333925586*distance + 0.4871305506216692;
                    power = 0.0031110124333925586*distance + 0.4971305506216692;


                }

                if (distance>=46 && distance<=82){
                    power = 0.003998017340689982*distance + 0.42136198084035686;


                }
                if (distance > 82){
                    power = 0.75;
                }
                hardware.outtake_bottom.setPower(power);
                hardware.outtake_top.setPower(power);




            } else {
                hardware.outtake_bottom.setPower(0);
                hardware.outtake_top.setPower(0);
            }
            if(!autoShooter.isTracking() && (autoShooter.turret.getCurrentPosition() > 100 || autoShooter.turret.getCurrentPosition() < -100)){
                hardware.turret.setTargetPosition(0);
                hardware.turret.setPower(0.014*(-hardware.turret.getCurrentPosition()));
                turret.setPower(0.014 * (-autoShooter.gettx()));
            }
if(!autoShooter.isTracking()) {
    int triggerL = (int) Math.floor(gamepad2.left_trigger * 10);
    int triggerR = (int) Math.floor(gamepad2.right_trigger * 10);
    position = hardware.turret.getCurrentPosition() - triggerL + triggerR;
    turret.setPower(0.014 * (-autoShooter.gettx()));
}

            telemetry.addData("State", currentState);
            telemetry.addData("Intake Power", hardware.intake.getPower());
            telemetry.addData("Outtake Bottom", hardware.outtake_bottom.getVelocity());
            telemetry.addData("Outtake Top", hardware.outtake_top.getVelocity());
            telemetry.addData("Turret Position", hardware.turret.getCurrentPosition());
            telemetry.update();
        }
    }
}

//2400