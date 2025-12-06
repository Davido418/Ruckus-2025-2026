package org.firstinspires.ftc.teamcode.MAIN;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Odometry {
    String Alliance = "blue";
    double blueBasketX=5;//set
    double blueBasketY=5;//set
    double redBasketX=5;//set
    double redBasketY=5;//set

    private MecanumDrive drive;

    public double getX() {
        drive.updatePoseEstimate();
        return 72-drive.localizer.getPose().position.x;
    }

    public double getY() {
        drive.updatePoseEstimate();
        return 72-drive.localizer.getPose().position.y;
    }

    public double getHeading() {
        drive.updatePoseEstimate();
        return drive.localizer.getPose().heading.toDouble();  // radians
    }
    public double getTxOdo(){
        if(Alliance == "red"){
            return Math.atan((getY()-blueBasketY)/(getX()-blueBasketX));

        }else{
            return Math.atan((getY()-redBasketY)/(redBasketX-getX()));
        }
    }
}

