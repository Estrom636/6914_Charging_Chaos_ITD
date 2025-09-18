package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

public class Hardware extends MecanumDrive{

    public DcMotorEx leftFront, leftBack, rightBack, rightFront, liftLeft, liftRight;

    public Servo horizontal, horizontal2, claw, liftLeftS, liftRightS, intakeLift, intakeLift2, lightLeft, lightRight, intakeLight;

    public CRServo inLeft, inRight;

    public ColorSensor inCol;

    public TouchSensor leftT, rightT;

    public Rev2mDistanceSensor transfer;

    public ModernRoboticsI2cRangeSensor lBack, rBack;

    public Limelight3A limelight;

    public Hardware(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        horizontal = hardwareMap.get(Servo.class, "hExt");
        horizontal2 = hardwareMap.get(Servo.class, "hExt2");
        horizontal2.setDirection(Servo.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        liftLeftS = hardwareMap.get(Servo.class, "leftLiftS");
        liftRightS = hardwareMap.get(Servo.class, "rightLiftS");

        intakeLift = hardwareMap.get(Servo.class, "inLift");
        intakeLift2 = hardwareMap.get(Servo.class, "inLift2");
        intakeLift2.setDirection(Servo.Direction.REVERSE);
        inLeft = hardwareMap.get(CRServo.class, "inL");
        inRight = hardwareMap.get(CRServo.class, "inR");

        lightLeft = hardwareMap.get(Servo.class, "lightL");
        lightRight = hardwareMap.get(Servo.class, "lightR");
        intakeLight = hardwareMap.get(Servo.class, "inLight");

        inCol = hardwareMap.get(ColorSensor.class, "ColIntake");
        leftT = hardwareMap.get(TouchSensor.class, "leftT");
        rightT = hardwareMap.get(TouchSensor.class, "rightT");
        transfer = hardwareMap.get(Rev2mDistanceSensor.class, "transfer");

        lBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "lBack");
        rBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rBack");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
}


