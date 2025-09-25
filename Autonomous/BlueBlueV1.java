package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "BlueBlue", group = "Autonomous")
public class BlueBlueV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(-8.25, 64, Math.PI/2));

        //Setting Pose Storage Variables
        PoseStorage.aliCol = false;
        PoseStorage.scoreType = true;

        //Setting intake inizalization position
        drive.intakeLift.setPosition(0.8);
        drive.intakeLift2.setPosition(0.8);
        drive.horizontal.setPosition(0.6);
        drive.horizontal2.setPosition(0.8);

        //Setting lift arm inizalization position
        drive.liftLeftS.setPosition(0.6);
        drive.liftRightS.setPosition(1);

        //Setting claw inizalization position
        drive.claw.setPosition(.8);

        //setting the LED colors
        drive.lightRight.setPosition(0.611);
        drive.lightLeft.setPosition(0.611);
        drive.intakeLight.setPosition(0);

        //creating the fail safe variable
        boolean pickUpFail;

        //Createing and starting limelight
        drive.limelight.start();
        drive.limelight.pipelineSwitch(0);
        drive.limelight.deleteSnapshots();
        LLResult result;
        drive.limelight.start();

        //Checking if limelight is reading data
        //This is check by just holding a sample infront of the camera
        while(!isStarted()){
            result = drive.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                drive.lightRight.setPosition(0.5);
                drive.lightLeft.setPosition(0.5);
            } else {
                drive.lightRight.setPosition(0.611);
                drive.lightLeft.setPosition(0.611);
            }
        }


        //reset the lift motor encoders
        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //this is to wait till the start button on the driver station is pressed
        waitForStart();

        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //PreLoad
        while (((drive.liftLeft.getCurrentPosition() * -1) < 1000) && drive.liftRight.getCurrentPosition() < 1000){
            drive.liftRight.setPower(Math.max((1000-(drive.liftRight.getCurrentPosition())) * 0.005, 0.1));
            drive.liftLeft.setPower(Math.min((1000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.005, -0.1));
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.35);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-8.25,64,Math.PI/2))
                        .strafeToSplineHeading(new Vector2d(-4, 30), Math.PI/2)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;


        drive.intakeLift.setPosition(0.48);
        drive.intakeLift2.setPosition(0.48);

        while (((drive.liftLeft.getCurrentPosition() * -1) > 800) && drive.liftRight.getCurrentPosition() > 800){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        drive.claw.setPosition(.45);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-4,30,Math.PI/2))
                        .waitSeconds(0.125)
                        .strafeTo(new Vector2d(-4,45))
                        .splineTo(new Vector2d(-35,35), 3*Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-35, 20), 3*Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-50, 20), 3*Math.PI/2)
                        .strafeToSplineHeading(new Vector2d(-50, 40), 3*Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-50, 56), 3*Math.PI/2)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;
        while (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }


        //Specimen 2
        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.claw.setPosition(.45);

        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.liftLeftS.setPosition(0.25);
        drive.liftRightS.setPosition(0.65);

        if(drive.rBack.getDistance(DistanceUnit.INCH) == 0 || drive.lBack.getDistance(DistanceUnit.INCH) == 0){
            pickUpFail = true;
            drive.lightRight.setPosition(1);
            drive.lightLeft.setPosition(1);
        }else{
            pickUpFail = false;
            drive.lightRight.setPosition(0.611);
            drive.lightLeft.setPosition(0.611);
        }

        while((drive.rBack.getDistance(DistanceUnit.INCH) + drive.lBack.getDistance(DistanceUnit.INCH)) / 2 >= 6.5){
            if(drive.rBack.getDistance(DistanceUnit.INCH) == 0 || drive.lBack.getDistance(DistanceUnit.INCH) == 0){
                pickUpFail = true;
                drive.lightRight.setPosition(1);
                drive.lightLeft.setPosition(1);
            }else{
                pickUpFail = false;
                drive.lightRight.setPosition(0.611);
                drive.lightLeft.setPosition(0.611);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.25, 0), 0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        PoseStorage.currentPose = drive.pose;

        drive.claw.setPosition(.8);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position, 3*Math.PI/2))
                        .waitSeconds(.125)
                        .build()
        );

        if(!pickUpFail){
            while (((drive.liftLeft.getCurrentPosition() * -1) < 1000) && drive.liftRight.getCurrentPosition() < 1000){
                drive.liftRight.setPower(Math.max((1000-(drive.liftRight.getCurrentPosition())) * 0.005, 0.1));
                drive.liftLeft.setPower(Math.min((1000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.005, -0.1));
            }
            drive.liftRight.setPower(.001);
            drive.liftLeft.setPower(-.001);
        }

        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.35);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position, 3*Math.PI/2))
                        .strafeToSplineHeading(new Vector2d(-2,38), Math.PI/2)
                        .strafeToSplineHeading(new Vector2d(-2, 30), Math.PI/2)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;


        while (((drive.liftLeft.getCurrentPosition() * -1) > 800) && drive.liftRight.getCurrentPosition() > 800){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        drive.claw.setPosition(.45);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-2, 30, Math.PI/2))
                        .waitSeconds(.125)
                        .build()
        );

        drive.claw.setPosition(.8);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-2, 30, Math.PI/2))
                        .strafeTo(new Vector2d(-2,45))
                        .strafeToSplineHeading(new Vector2d(-50, 56), 3*Math.PI/2)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;

        while (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }


        //Specimen 3

        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.claw.setPosition(.45);

        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.liftLeftS.setPosition(0.25);
        drive.liftRightS.setPosition(0.65);


        if(drive.rBack.getDistance(DistanceUnit.INCH) == 0 || drive.lBack.getDistance(DistanceUnit.INCH) == 0){
            pickUpFail = true;
            drive.lightRight.setPosition(1);
            drive.lightLeft.setPosition(1);
        }else{
            pickUpFail = false;
            drive.lightRight.setPosition(0.611);
            drive.lightLeft.setPosition(0.611);
        }

        while((drive.rBack.getDistance(DistanceUnit.INCH) + drive.lBack.getDistance(DistanceUnit.INCH)) / 2 >= 6.5){
            if(drive.rBack.getDistance(DistanceUnit.INCH) == 0 || drive.lBack.getDistance(DistanceUnit.INCH) == 0){
                pickUpFail = true;
                drive.lightRight.setPosition(1);
                drive.lightLeft.setPosition(1);
            }else{
                pickUpFail = false;
                drive.lightRight.setPosition(0.611);
                drive.lightLeft.setPosition(0.611);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.25, 0), 0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        PoseStorage.currentPose = drive.pose;

        drive.claw.setPosition(.8);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position, 3*Math.PI/2))
                        .waitSeconds(.125)
                        .build()
        );


        if(!pickUpFail){
            while (((drive.liftLeft.getCurrentPosition() * -1) < 1000) && drive.liftRight.getCurrentPosition() < 1000){
                drive.liftRight.setPower(Math.max((1000-(drive.liftRight.getCurrentPosition())) * 0.005, 0.1));
                drive.liftLeft.setPower(Math.min((1000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.005, -0.1));
            }
            drive.liftRight.setPower(.001);
            drive.liftLeft.setPower(-.001);
        }

        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.35);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position, 3*Math.PI/2))
                        .strafeToSplineHeading(new Vector2d(-6,38), Math.PI/2)
                        .strafeToSplineHeading(new Vector2d(-6, 30), Math.PI/2)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;


        while (((drive.liftLeft.getCurrentPosition() * -1) > 800) && drive.liftRight.getCurrentPosition() > 800){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        drive.claw.setPosition(.45);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-6, 30, Math.PI/2))
                        .waitSeconds(.125)
                        .build()
        );

        drive.claw.setPosition(.8);

        drive.horizontal.setPosition(0.2);
        drive.horizontal2.setPosition(0.4);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-6, 30, Math.PI/2))
                        .splineTo(new Vector2d(-48, 56), 3*Math.PI/4)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;

        while (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
        }

        PoseStorage.currentPose = drive.pose;
    }

}
