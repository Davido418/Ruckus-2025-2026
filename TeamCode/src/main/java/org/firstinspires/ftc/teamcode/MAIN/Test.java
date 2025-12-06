package org.firstinspires.ftc.teamcode.MAIN;
import static org.firstinspires.ftc.teamcode.MAIN.AutoFirst.AutoShooter.turret;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test")
@Config
public class Test extends LinearOpMode {
    public static int pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);
        AutoShooter autoShooter = new AutoShooter(hardwareMap);
        hardware.hood.setPosition(pos);

        }
    }


//2400