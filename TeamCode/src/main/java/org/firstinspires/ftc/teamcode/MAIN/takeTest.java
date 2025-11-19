package org.firstinspires.ftc.teamcode.MAIN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.MAIN.Hardware;
import org.firstinspires.ftc.teamcode.MAIN.Constants;


@TeleOp(name = "takeTest")
public class takeTest extends LinearOpMode {

    public static double turretpos1, turretpos2;
    public double close1_pos, close2_pos, far1_pos = 0.1;

    public double shooter_pow_c1, shooter_pow_c2, shooter_pow_f1;
    public double back, front;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors from hardware map
        Hardware hardware = new Hardware(hardwareMap);

        Gamepad currentgamepad2 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad prevgamepad2 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        waitForStart();

        // Run until the op mode is stopped
        while (opModeIsActive()) {
            prevgamepad2.copy(currentgamepad2);
            prevGamepad2.copy(currentGamepad2);

            currentgamepad2.copy(gamepad2);
            currentGamepad2.copy(gamepad2);
            if (gamepad2.right_bumper) {
                hardware.intake.setPower(1);//intake
                hardware.outtake_bottom.setPower(-0.1);
                hardware.outtake_top.setPower(-0.1);
            } else if (gamepad2.left_bumper) {
                hardware.outtake_bottom.setPower(1);
                hardware.outtake_top.setPower(1);
                hardware.intake.setPower(1);
            } else {
                hardware.intake.setPower(0);//intake
                hardware.outtake_bottom.setPower(0);
                hardware.outtake_top.setPower(0);
            }



            //drivetrain

        }
    }
}

