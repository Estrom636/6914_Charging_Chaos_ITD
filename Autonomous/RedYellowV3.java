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
@Autonomous(name = "RedYellowV3", group = "Autonomous")
public class RedYellowV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(32, 64, Math.PI));

        PoseStorage.aliCol = true;
        PoseStorage.scoreType = false;

        drive.intakeLift.setPosition(0.8);
        drive.intakeLift2.setPosition(0.8);
        drive.horizontal.setPosition(0.6);
        drive.horizontal2.setPosition(0.8);

        drive.liftLeftS.setPosition(0.6);
        drive.liftRightS.setPosition(1);

        drive.claw.setPosition(.8);

        drive.lightRight.setPosition(0.3);
        drive.lightLeft.setPosition(0.388);
        drive.intakeLight.setPosition(0);

        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        boolean liftDown = false;
        boolean intakeIn = false;

        boolean liftFail = false;
        boolean transferFail = false;
        boolean intakeMiss1 = false;
        boolean intakeMiss2 = false;
        boolean intakeMiss3 = false;
        boolean intakeMiss4 = false;

        drive.limelight.start();
        drive.limelight.pipelineSwitch(3);
        drive.limelight.deleteSnapshots();
        LLResult result;
        double tx = 0.0;
        drive.limelight.start();

        while(!isStarted()){
            result = drive.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                drive.lightRight.setPosition(0.5);
                drive.lightLeft.setPosition(0.5);
            } else {
                drive.lightRight.setPosition(0.3);
                drive.lightLeft.setPosition(0.388);
            }
        }

        waitForStart();
        resetRuntime();

        //PreLoad

        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.liftLeftS.setPosition(0.5);
        drive.liftRightS.setPosition(1);

        drive.liftRight.setPower(0.75);
        drive.liftLeft.setPower(-0.75);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(32.25,64, Math.PI))
                        .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                        .build()
        );
        PoseStorage.currentPose = drive.pose;

        while (((drive.liftLeft.getCurrentPosition() * -1) < 2900) && drive.liftRight.getCurrentPosition() < 2900){
            drive.liftRight.setPower(Math.max((2900-(drive.liftRight.getCurrentPosition())) * 0.01, 0.2));
            drive.liftLeft.setPower(Math.min((2900-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.2));
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);

        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.80);
        sleep(100);

        drive.claw.setPosition(.45);
        sleep(250);
        drive.claw.setPosition(.8);


        //Sample 2 -> High
        if(true){
            drive.horizontal.setPosition(0.3);
            drive.horizontal2.setPosition(0.5);

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .waitSeconds(0.25)
                            .strafeToSplineHeading(new Vector2d(56, 52), 3*Math.PI/2, new TranslationalVelConstraint(5))
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            drive.limelight.captureSnapshot("Pick Up 1 Start (Mid)");


            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            liftDown = false;
            intakeIn = false;
            double startTime4 = getRuntime();
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss1) {
                if (getRuntime() - startTime4 > 2) {
                    if (!liftDown) {
                        liftFail = true;
                    }
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
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25;
                        } else {
                            tx = 0;
                        }
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), tx * 0.3));
                        drive.updatePoseEstimate();
                    } else {
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
            PoseStorage.currentPose = drive.pose;

            drive.lightRight.setPosition(0.3);
            drive.lightLeft.setPosition(0.388);

            if(!intakeMiss1 && !liftFail){
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(400);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double startTime2_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime2_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            if(drive.intakeLift.getPosition() < .6){
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        }else{
                            break;
                        }
                    }
                }
                if(!transferFail){
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3000) && drive.liftRight.getCurrentPosition() < 3000){
                        drive.liftRight.setPower(Math.max((3000-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }


        //Sample 3 -> High
        if(!liftFail && !transferFail){
            drive.horizontal.setPosition(0.3);
            drive.horizontal2.setPosition(0.5);

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .waitSeconds(0.25)
                            .strafeToSplineHeading(new Vector2d(56, 50), 11*Math.PI/8, new TranslationalVelConstraint(5))
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            drive.limelight.captureSnapshot("Pick Up 2 Start (Inner)");

            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            liftDown = false;
            intakeIn = false;
            double startTime4 = getRuntime();
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss2) {
                if (getRuntime() - startTime4 > 2) {
                    if (!liftDown) {
                        liftFail = true;
                    }
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
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25;
                        } else {
                            tx = 0;
                        }
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), tx * 0.2));
                        drive.updatePoseEstimate();
                    } else {
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
            PoseStorage.currentPose = drive.pose;

            drive.lightRight.setPosition(0.3);
            drive.lightLeft.setPosition(0.388);

            if(!intakeMiss2 && !liftFail){
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(600);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double startTime2_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime2_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            if(drive.intakeLift.getPosition() < .6){
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        }else{
                            break;
                        }
                    }
                }
                if(!transferFail){
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5*Math.PI/4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3000) && drive.liftRight.getCurrentPosition() < 3000){
                        drive.liftRight.setPower(Math.max((3000-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3000-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }


        //Sample 4 -> High
        if(!liftFail && !transferFail) {
            drive.intakeLift.setPosition(0.98);
            drive.intakeLift2.setPosition(0.98);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5 * Math.PI / 4))
                            .strafeToSplineHeading(new Vector2d(52, 30), 31 * Math.PI / 16)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            drive.limelight.captureSnapshot("Pick Up 3 Start (Wall)");

            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
            drive.inRight.setPower(-1);
            drive.inLeft.setPower(1);
            drive.horizontal.setPosition(0.4);
            drive.horizontal2.setPosition(0.6);
            drive.claw.setPosition(.45);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(52, 30, 31 * Math.PI / 16))
                            .waitSeconds(0.2)
                            .turnTo(28 * Math.PI / 16)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            drive.limelight.captureSnapshot("Pick Up 3 Middle (Wall)");

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            liftDown = false;
            intakeIn = false;
            double startTime3 = getRuntime();
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss3) {
                if (getRuntime() - startTime3 > 4) {
                    if (!liftDown) {
                        liftFail = true;
                    }
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
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()) {
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    } else {
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        liftDown = true;
                    }

                    if (!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)) {
                        result = drive.limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            drive.lightRight.setPosition(0.5);
                            drive.lightLeft.setPosition(0.5);
                            tx = result.getTx() / 27.25;
                        } else {
                            tx = 0;
                        }
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), tx * 0.3));
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);

                        drive.horizontal.setPosition(0.2);
                        drive.horizontal2.setPosition(0.4);
                    } else {
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
            PoseStorage.currentPose = drive.pose;

            drive.lightRight.setPosition(0.3);
            drive.lightLeft.setPosition(0.388);

            if (!intakeMiss3 && !liftFail) {
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(600);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double startTime3_1 = getRuntime();
                while (!transferFail) {
                    if (getRuntime() - startTime3_1 > 2) {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    } else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            if (drive.intakeLift.getPosition() < .6) {
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        } else {
                            break;
                        }
                    }
                }
                if (!transferFail) {
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.97);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    drive.liftRight.setPower(0.75);
                    drive.liftLeft.setPower(-0.75);

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(52, 30, 28 * Math.PI / 16))
                                    .strafeToSplineHeading(new Vector2d(56, 56), 5 * Math.PI / 4)
                                    .build()
                    );
                    PoseStorage.currentPose = drive.pose;

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100) {
                        drive.liftRight.setPower(Math.max((3100 - (drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3100 - (drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);
                    sleep(100);

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }

        //Sample 5 -> High from Sub
        if(false && !liftFail && !transferFail && !intakeMiss1 && !intakeMiss2){
            drive.liftRight.setPower(-0.5);
            drive.liftLeft.setPower(0.5);

            drive.horizontal.setPosition(0.2);
            drive.horizontal2.setPosition(0.4);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56, 56, 5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 14), Math.PI)
                            .strafeToSplineHeading(new Vector2d(28, 14), Math.PI)
                            .build()
            );

            drive.intakeLift.setPosition(1);
            drive.intakeLift2.setPosition(1);

            liftDown = false;
            intakeIn = false;
            double startTime4 = getRuntime();
            while ((!liftDown || !intakeIn) && !liftFail && !intakeMiss4) {
                if (getRuntime() - startTime4 > 10) {
                    if (!liftDown) {
                        liftFail = true;
                    }
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
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.25, 0), 0));
                        drive.updatePoseEstimate();
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.inRight.setPower(0);
                        drive.inLeft.setPower(0);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        sleep(200);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        intakeIn = true;
                    }
                }
            }

            if (!intakeMiss4 && !liftFail) {
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(500);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                double startTime4_1 = getRuntime();
                while (!transferFail) {
                    if (getRuntime() - startTime4_1 > 2) {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    } else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100) {
                            if (drive.intakeLift.getPosition() < .6) {
                                drive.inRight.setPower(.75);
                                drive.inLeft.setPower(-.75);
                            }
                        } else {
                            break;
                        }
                    }
                }
                if (!transferFail) {
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.5);
                    drive.liftRightS.setPosition(1);

                    liftFail = true;
                }
            }
        }

        //Park
        if(!liftFail){
            drive.claw.setPosition(.8);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(56,56,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 48), 5*Math.PI/4)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

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

            drive.liftRight.setPower(.001);
            drive.liftLeft.setPower(-.001);

            drive.liftLeftS.setPosition(0.2);
            drive.liftRightS.setPosition(1);


            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(48,48, 5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(35, 10), 2*Math.PI)
                            .strafeToSplineHeading(new Vector2d(23, 10), 2*Math.PI)
                            .build()
            );
            PoseStorage.currentPose = drive.pose;

            while((drive.rBack.getDistance(DistanceUnit.INCH) + drive.lBack.getDistance(DistanceUnit.INCH)) / 2 >= 2){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.25, 0), 0));
                drive.updatePoseEstimate();
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            PoseStorage.currentPose = drive.pose;

            while (((drive.liftLeft.getCurrentPosition() * -1) > 1200) && drive.liftRight.getCurrentPosition() > 1200){
                drive.liftRight.setPower(-1);
                drive.liftLeft.setPower(1);
            }
        }



        while(liftFail && getRuntime() < 28){
            PoseStorage.currentPose = drive.pose;
            drive.lightRight.setPosition(1);
            drive.lightLeft.setPosition(1);
            drive.liftRight.setPower(0);
            drive.liftLeft.setPower(0);
        }
    }
}