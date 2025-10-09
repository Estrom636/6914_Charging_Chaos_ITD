package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@Autonomous(name = "BlueYellowV3", group = "Autonomous")
public class BlueYellowV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(32, 64, Math.PI));

        //Setting Pose Storage Variables
        PoseStorage.aliCol = false;
        PoseStorage.scoreType = false;

        //Setting intake inizalization position
        drive.intakeLift.setPosition(0.8);
        drive.intakeLift2.setPosition(0.8);
        drive.horizontal.setPosition(0.6);
        drive.horizontal2.setPosition(0.8);

        //Setting lift arm inizalization positions
        drive.liftLeftS.setPosition(0.6);
        drive.liftRightS.setPosition(1);

        //Setting claw inizalization position
        drive.claw.setPosition(.8);

        //Setting the LED colors
        //blue, yellow, and off
        drive.lightRight.setPosition(0.611);
        drive.lightLeft.setPosition(0.388);
        drive.intakeLight.setPosition(0);

        //reset lift encoders
        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Creating check variables for fail safes
        boolean liftDown = false;
        boolean intakeIn = false;

        //Creating fail safe variables
        boolean liftFail = false;
        boolean transferFail = false;
        boolean intakeMiss1 = false;
        boolean intakeMiss2 = false;
        boolean intakeMiss3 = false;
        boolean intakeMiss4 = false;

        //Create and starting limelight
        drive.limelight.start();
        drive.limelight.pipelineSwitch(3);
        drive.limelight.deleteSnapshots();
        LLResult result;
        double tx = 0.0;
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
                drive.lightLeft.setPosition(0.388);
            }
        }

        //This is to wait till the start button on the driver station is pressed
        waitForStart();
        //Reset run time to zero because it is used for timing
        resetRuntime();

        //PreLoad

        //reset lift encoders
        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift arm to prescore position
        drive.liftLeftS.setPosition(0.5);
        drive.liftRightS.setPosition(1);

        //start lift up movement
        drive.liftRight.setPower(0.75);
        drive.liftLeft.setPower(-0.75);

        //move to infront of baskets for scoring
        //update the position in position storage
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(32.25,64, Math.PI))
                        .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;

        //finish moving lift to high basket height then hold
        while (((drive.liftLeft.getCurrentPosition() * -1) < 2900) && drive.liftRight.getCurrentPosition() < 2900){
            drive.liftRight.setPower(Math.max((2900-(drive.liftRight.getCurrentPosition())) * 0.01, 0.2));
            drive.liftLeft.setPower(Math.min((2900-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.2));
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        //lift arms to scoring position
        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.80);
        sleep(100);

        //open then close claw
        drive.claw.setPosition(.45);
        sleep(250);
        drive.claw.setPosition(.8);


        //Sample 2 -> High
        //this if is for all of pick up to scoring for sample number 2
        //only changed in test to skip this
        if(true){
            //extend intake most of the way out
            drive.horizontal.setPosition(0.3);
            drive.horizontal2.setPosition(0.5);

            //lift arms into pick up hold position
            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            //movement to position to pick up middle spike mark sample
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .waitSeconds(0.25)
                            .strafeToSplineHeading(new Vector2d(56, 52), 3*Math.PI/2, new TranslationalVelConstraint(5))
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //limelight snapshot for later imaging changes if needed
            drive.limelight.captureSnapshot("Pick Up 1 Start (Mid)");


            //lower intake
            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            //reset the check variables
            liftDown = false;
            intakeIn = false;
            //get the start time for intaking
            double startTime4 = getRuntime();
            //while for lowering the lift and intaking the sample from the ground
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss1) {
                //if sets a time limit for picking up and lowering the lift
                if (getRuntime() - startTime4 > 2) {
                    //if the time is longer then 2 seconds
                    //check if lift is still up if yes then set liftFail to true
                    if (!liftDown) {
                        liftFail = true;
                    }
                    //check if the intake is in meaning the nothing has been picked up
                    //if yes set intakeMiss1 to true, stop movement, lift and retract intake, and then move back to basket scoring location
                    if (!intakeIn) {
                        intakeMiss1 = true;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                        .build()
                        );
                    }
                } else {
                    //if the time is less then 2 seconds
                    //move lift all the way down then open claw and set liftDown to true
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()) {
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    } else {
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        drive.claw.setPosition(.45);
                        liftDown = true;
                    }

                    if (!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)) {
                        //if color sensor does not read color
                        //get limelight data and check and set it
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25; //dividing by the angle to turn into a decimal value
                        } else {
                            tx = 0;
                        }
                        //set intake wheel to pull in
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        //move forward with changing the angle based in the limelight value
                        //update the position in position storage
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), tx * 0.3));
                        drive.updatePoseEstimate();
                    } else {
                        //if something is in the intake
                        //stop all movement, lift and retract intake, and set intakeIn to true
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(0);
                        drive.inLeft.setPower(0);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        intakeIn = true;
                    }
                }
            }
            //update the position in position storage
            PoseStorage.currentPose = drive.pose;

            //setting LED colors
            //blue and yellow
            drive.lightRight.setPosition(0.611);
            drive.lightLeft.setPosition(0.388);

            //only happens if the sample was picked up and the lift functions
            if(!intakeMiss1 && !liftFail){
                //reset lift encoders
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(400);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //get start time for transfer
                double startTime2_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime2_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            //if run time is longer then 2 seconds and there is nothing in the transfer area
                            //set transferFail to true and close the claw
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            //if the time is less then 2 seconds and there is nothing in the transfer area
                            //run the intake out well it is up in transfer position
                            if(drive.intakeLift.getPosition() < .6){
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        }else{
                            //break out of while loop if the transfer happened in under 2 seconds
                            break;
                        }
                    }
                }
                //only runs if the sample transfered
                if(!transferFail){
                    //stop intake wheels
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    //lower lift arms to pick up
                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    //close the claw
                    drive.claw.setPosition(.8);
                    sleep(200);

                    //lift arm to prescore position
                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    //start lift up movement
                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    //move to infront of baskets for scoring
                    //update the position in position storage
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    //finish moving lift to high basket height then hold
                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3000) && drive.liftRight.getCurrentPosition() < 3000){
                        drive.liftRight.setPower(Math.max((3000-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    //lift arms to scoring position
                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    //open then close claw
                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    //lift arms into pick up hold position
                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }


        //Sample 3 -> High
        //Only runs if the lift works and there has not been a transfer fail
        if(!liftFail && !transferFail){
            //extend intake most of the way out
            drive.horizontal.setPosition(0.3);
            drive.horizontal2.setPosition(0.5);

            //lift arms into pick up hold position
            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            //movement to position to pick up inner spike mark sample
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .waitSeconds(0.25)
                            .strafeToSplineHeading(new Vector2d(56, 50), 11*Math.PI/8, new TranslationalVelConstraint(5))
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //limelight snapshot for later imaging changes if needed
            drive.limelight.captureSnapshot("Pick Up 2 Start (Inner)");

            //lower intake
            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            //reset the check variables
            liftDown = false;
            intakeIn = false;
            //get the start time for intaking
            double startTime4 = getRuntime();
            //while for lowering the lift and intaking the sample from the ground
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss2) {
                //if sets a time limit for picking up and lowering the lift
                if (getRuntime() - startTime4 > 2) {
                    //if the time is longer then 2 seconds
                    //check if lift is still up if yes then set liftFail to true
                    if (!liftDown) {
                        liftFail = true;
                    }
                    //check if the intake is in meaning the nothing has been picked up
                    //if yes set intakeMiss1 to true, stop movement, lift and retract intake
                    if (!intakeIn) {
                        intakeMiss2 = true;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                    }
                } else {
                    //if the time is less then 2 seconds
                    //move lift all the way down then open claw and set liftDown to true
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()) {
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    } else {
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        drive.claw.setPosition(.45);
                        liftDown = true;
                    }

                    if (!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)) {
                        //if color sensor does not read color
                        //get limelight data and check and set it
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25; //dividing by the angle to turn into a decimal value
                        } else {
                            tx = 0;
                        }
                        //set intake wheel to pull in
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        //move forward with changing the angle based in the limelight value
                        //update the position in position storage
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), tx * 0.2));
                        drive.updatePoseEstimate();
                    } else {
                        //if something is in the intake
                        //stop all movement, lift and retract intake, and set intakeIn to true
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(0);
                        drive.inLeft.setPower(0);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        intakeIn = true;
                    }
                }
            }
            //update the position in position storage
            PoseStorage.currentPose = drive.pose;

            //setting LED colors
            //blue and yellow
            drive.lightRight.setPosition(0.611);
            drive.lightLeft.setPosition(0.388);

            //only happens if the sample was picked up and the lift functions
            if(!intakeMiss2 && !liftFail){
                //reset lift encoders
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(600);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //get start time for transfer
                double startTime2_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime2_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            //if run time is longer then 2 seconds and there is nothing in the transfer area
                            //set transferFail to true and close the claw
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            //if the time is less then 2 seconds and there is nothing in the transfer area
                            //run the intake out well it is up in transfer position
                            if(drive.intakeLift.getPosition() < .6){
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        }else{
                            //break out of while loop if the transfer happened in under 2 seconds
                            break;
                        }
                    }
                }
                //only runs if the sample transfered
                if(!transferFail){
                    //stop intake wheels
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    //lower lift arms to pick up
                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    //close the claw
                    drive.claw.setPosition(.8);
                    sleep(200);

                    //lift arm to prescore position
                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    //start lift up movement
                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    //move to infront of baskets for scoring
                    //update the position in position storage
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    //finish moving lift to high basket height then hold
                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3000) && drive.liftRight.getCurrentPosition() < 3000){
                        drive.liftRight.setPower(Math.max((3000-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    //lift arms to scoring position
                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    //open then close claw
                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    //lift arms into pick up hold position
                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }


        //Sample 4 -> High
        //Only runs if the lift works and there has not been a transfer fail
        if(!liftFail && !transferFail) {
            //lowers the intakedown
            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            //move to pick up sample from wall spike mark
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5 * Math.PI / 4))
                            .strafeToSplineHeading(new Vector2d(52, 30), 31 * Math.PI / 16)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //limelight snapshot for later imaging changes if needed
            drive.limelight.captureSnapshot("Pick Up 3 Start (Wall)");

            //set lift motors to down, set intake wheels to in, open claw, and extend intake out
            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
            drive.inRight.setPower(-1);
            drive.inLeft.setPower(1);
            drive.horizontal.setPosition(0.4);
            drive.horizontal2.setPosition(0.6);
            drive.claw.setPosition(.45);

            //turn a small amount to pull away from wall
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(52, 30, 31 * Math.PI / 16))
                            .waitSeconds(0.2)
                            .turnTo(28 * Math.PI / 16)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //limelight snapshot for later imaging changes if needed
            drive.limelight.captureSnapshot("Pick Up 3 Middle (Wall)");

            //lift arms into pick up hold position
            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            //reset the check variables
            liftDown = false;
            intakeIn = false;
            //get the start time for intaking
            double startTime3 = getRuntime();
            //while for lowering the lift and intaking the sample from the ground
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss3) {
                //if sets a time limit for picking up and lowering the lift
                if (getRuntime() - startTime3 > 4) {
                    //if the time is longer then 2 seconds
                    //check if lift is still up if yes then set liftFail to true
                    if (!liftDown) {
                        liftFail = true;
                    }
                    //check if the intake is in meaning the nothing has been picked up
                    //if yes set intakeMiss1 to true, stop movement, lift and retract intake
                    if (!intakeIn) {
                        intakeMiss3 = true;
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        sleep(200);
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(56,40,3*Math.PI/2))
                                        .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                        .build()
                        );
                    }
                } else {
                    //if the time is less then 2 seconds
                    //move lift all the way down then open claw and set liftDown to true
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()) {
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    } else {
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        liftDown = true;
                    }

                    if (!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)) {
                        //if color sensor does not read color
                        //get limelight data and check and set it
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25; //dividing by the angle to turn into a decimal value
                        } else {
                            tx = 0;
                        }
                        //set intake wheel to pull in
                        //changing the angle based in the limelight value
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), tx * 0.3));
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);

                        //extend intake out more
                        drive.horizontal.setPosition(0.2);
                        drive.horizontal2.setPosition(0.4);
                    } else {
                        //if something is in the intake
                        //lift and retract intake, and set intakeIn to true
                        drive.inRight.setPower(0);
                        drive.inLeft.setPower(0);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        intakeIn = true;
                    }
                }
            }
            //update the position in position storage
            PoseStorage.currentPose = drive.pose;

            //setting LED colors
            //blue and yellow
            drive.lightRight.setPosition(0.611);
            drive.lightLeft.setPosition(0.388);

            //only happens if the sample was picked up and the lift functions
            if (!intakeMiss3 && !liftFail) {
                //reset lift encoders
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(600);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //get start time for transfer
                double startTime3_1 = getRuntime();
                while (!transferFail) {
                    if (getRuntime() - startTime3_1 > 2) {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            //if run time is longer then 2 seconds and there is nothing in the transfer area
                            //set transferFail to true and close the claw
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    } else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            //if the time is less then 2 seconds and there is nothing in the transfer area
                            //run the intake out well it is up in transfer position
                            if (drive.intakeLift.getPosition() < .6) {
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        } else {
                            //break out of while loop if the transfer happened in under 2 seconds
                            break;
                        }
                    }
                }
                //only runs if the sample transfered
                if (!transferFail) {
                    //stop intake wheels
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    //lower lift arms to pick up
                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    //close the claw
                    drive.claw.setPosition(.8);
                    sleep(200);

                    //lift arm to prescore position
                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    //start lift up movement//start lift up movement
                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    //move to infront of baskets for scoring
                    //update the position in position storage
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(52, 30, 28 * Math.PI / 16))
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5 * Math.PI / 4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    //finish moving lift to high basket height then hold
                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100) {
                        drive.liftRight.setPower(Math.max((3100 - (drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3100 - (drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    //lift arms to scoring position
                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    //open then close claw
                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    //lift arms into pick up hold position
                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }

        //Sample 5 -> High from Sub
        //this was never run during comp
        //this will only run if lift has not failed, transfer has not failed, and intake 1 and 2 were not missed
        if(false && !liftFail && !transferFail && !intakeMiss1 && !intakeMiss2){
            //set lifts to lower
            drive.liftRight.setPower(-0.5);
            drive.liftLeft.setPower(0.5);

            //extend intake out
            drive.horizontal.setPosition(0.2);
            drive.horizontal2.setPosition(0.4);

            //move to pick up from in submersable
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 14), Math.PI)
                            .strafeToSplineHeading(new Vector2d(28, 14), Math.PI)
                            .build()
            );

            //lower the intake
            drive.intakeLift.setPosition(1);
            drive.intakeLift2.setPosition(1);

            //reset the check variables
            liftDown = false;
            intakeIn = false;
            //get the start time for intaking
            double startTime4 = getRuntime();
            //while for lowering the lift and intaking the sample from the ground
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss4) {
                if (getRuntime() - startTime4 > 10) {
                    //if the time is longer then 10 seconds
                    //check if lift is still up if yes then set liftFail to true
                    if (!liftDown) {
                        liftFail = true;
                    }
                    //check if the intake is in meaning the nothing has been picked up
                    //if yes set intakeMiss1 to true, stop movement, lift and retract intake
                    if (!intakeIn) {
                        intakeMiss4 = true;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        sleep(200);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                    }
                } else {
                    //if the time is less then 10 seconds
                    //move lift all the way down then open claw and set liftDown to true
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()) {
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    } else {
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        drive.claw.setPosition(.45);
                        liftDown = true;
                    }

                    if (!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)) {
                        //if color sensor does not read color
                        //set intake wheel to pull in
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        //move forward with changing the angle based in the limelight value
                        //update the position in position storage
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), 0));
                        drive.updatePoseEstimate();
                    } else {
                        //if something is in the intake
                        //stop all movement, lift and retract intake, and set intakeIn to true
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(0);
                        drive.inLeft.setPower(0);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        sleep(200); //wait before pulling intake back
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        intakeIn = true;
                    }
                }
            }

            //only happens if the sample was picked up and the lift functions
            if (!intakeMiss4 && !liftFail) {
                //reset lift encoders
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(500);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                //get start time for transfer
                double startTime4_1 = getRuntime();
                while (!transferFail) {
                    if (getRuntime() - startTime4_1 > 2) {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            //if run time is longer then 2 seconds and there is nothing in the transfer area
                            //set transferFail to true and close the claw
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    } else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            //if the time is less then 2 seconds and there is nothing in the transfer area
                            //run the intake out well it is up in transfer position
                            if (drive.intakeLift.getPosition() < .6) {
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        } else {
                            //break out of while loop if the transfer happened in under 2 seconds
                            break;
                        }
                    }
                }
                //only runs if the sample transfered
                if (!transferFail) {
                    //stop intake wheels
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    //lower lift arms to pick up
                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    //close the claw
                    drive.claw.setPosition(.8);
                    sleep(200);

                    //lift arm to prescore position
                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    //lift fail to true because this was not finished to be able to go place in basket
                    liftFail = true;
                }
            }
        }

        //Park
        //will only run if lift has not failed
        if(!liftFail){
            //close claw
            drive.claw.setPosition(.8);
            //move to in front of baskets
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56,56,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 48), 5*Math.PI/4)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //move lift to high for touching low bar
            if(((drive.liftLeft.getCurrentPosition() * -1) > 1500) && drive.liftRight.getCurrentPosition() > 1500){
                while (((drive.liftLeft.getCurrentPosition() * -1) > 1500) && drive.liftRight.getCurrentPosition() > 1500){
                    drive.liftRight.setPower(-1);
                    drive.liftLeft.setPower(1);
                }
            }else if(((drive.liftLeft.getCurrentPosition() * -1) < 1500) && drive.liftRight.getCurrentPosition() < 1500){
                while (((drive.liftLeft.getCurrentPosition() * -1) < 1500) && drive.liftRight.getCurrentPosition() < 1500){
                    drive.liftRight.setPower(1);
                    drive.liftLeft.setPower(-1);
                }
            }

            //set lift to hold
            drive.liftRight.setPower(.001);
            drive.liftLeft.setPower(-.001);

            //set lift arms to touch position
            drive.liftLeftS.setPosition(0.2);
            drive.liftRightS.setPosition(1);


            //move to position to back into the parking spot
            //update the position in position storage
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(48,48, 5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(35, 10), 2*Math.PI)
                            .strafeToSplineHeading(new Vector2d(23, 10), 2*Math.PI)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            //back up till the rear distance sensors are reading less than 2
            //stop all movement
            //update the position in position storage
            while((drive.rBack.getDistance(DistanceUnit.INCH) + drive.lBack.getDistance(DistanceUnit.INCH)) / 2 >= 2){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.25, 0), 0));
                drive.updatePoseEstimate();
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            PoseStorage.currentPose = drive.pose;

            //lower the lift to touch bar
            while (((drive.liftLeft.getCurrentPosition() * -1) > 1200) && drive.liftRight.getCurrentPosition() > 1200){
                drive.liftRight.setPower(-1);
                drive.liftLeft.setPower(1);
            }
        }



        //only runs well the lift failed the the run time is less than 28sec
        //update the position in position storage
        //set the lights to white
        //set the lift motors to 0
        while(liftFail && getRuntime() < 28){
            PoseStorage.currentPose = drive.pose;
            drive.lightRight.setPosition(1);
            drive.lightLeft.setPosition(1);
            drive.liftRight.setPower(0);
            drive.liftLeft.setPower(0);
        }
    }
}








