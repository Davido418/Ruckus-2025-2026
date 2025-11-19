package org.firstinspires.ftc.teamcode.MAIN;

import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class Auto {
    public class Drive{
        //Nameing convention:
        //1 means the closest row of balls to the "goal", 2 means secend row, etc.
        //b means the secend starting position, eg. from the starting position lined up with the "goal"
//        public Action red1ToShoot(){
//
//        }
//        public Action red2ToShoot(){
//
//        }
//        public Action red3ToShoot(){
//
//        }
//        public Action blue1ToShoot(){
//
//        }
//        public Action blue2ToShoot(){
//
//        }
//        public Action blue3ToShoot(){
//
//        }
//        public Action red1ToShootB(){
//
//        }
//        public Action red2ToShootB(){
//
//        }
//        public Action red3ToShootB(){
//
//        }
//        public Action blue1ToShootB(){
//
//        }
//        public Action blue2ToShootB(){
//
//        }
//        public Action blue3ToShootB(){
//
//        }
//        public Action blueShooterTo1(){
//
//        }
//        public Action blueShooterTo2(){
//
//        }
//        public Action blueShooterTo3(){
//
//        }
//        public Action redShooterTo1(){
//
//        }
//        public Action redShooterTo2(){
//
//        }
//        public Action redShooterTo3(){
//
//        }
//
//
//
//
//
//
//    }
//
//    public class Shooter{
//        public Action intake(){
//
//
//        }
//        public Action shoot(){
//
//        }
//

    }
}
