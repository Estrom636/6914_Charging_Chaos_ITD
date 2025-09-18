package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose; //For keeping last know position of the robot
    public static boolean aliCol; //Tracks what aliance color the robot is. true = red, false = blue
    public static boolean scoreType; //Tracks if robot is running SPECIMEN or SAMPLES. true = SPECIMEN, false = SAMPLE
    public static int intakeCount; //Keeps track of the number of times the intake was lowered per teleOp
    public static int snapCount; //Keeps track of the number of manual LimeLight snapshots per match
}

