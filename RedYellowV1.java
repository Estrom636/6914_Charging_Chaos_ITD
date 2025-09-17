package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Disabled
@Autonomous(name = "RedYellow", group = "Autonomous")
public class RedYellowV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(32.25, 64, Math.PI));

        drive.intakeLift.setPosition(0.8);
        drive.intakeLift2.setPosition(0.8);
        drive.horizontal.setPosition(0.6);
        drive.horizontal2.setPosition(0.8);

        drive.liftLeftS.setPosition(0.6);
        drive.liftRightS.setPosition(1);

        drive.claw.setPosition(.8);

        drive.lightRight.setPosition(0.3);
        drive.lightLeft.setPosition(0.388);

        drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        boolean liftDown = false;
        boolean intakeIn = false;

        boolean liftFail = false;
        boolean intakeMiss1 = false;
        boolean intakeMiss2 = false;
        boolean intakeMiss3 = false;
        boolean transferFail = false;


        waitForStart();
        resetRuntime();

        //PreLoad

        drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.liftLeftS.setPosition(0.20);
        drive.liftRightS.setPosition(0.80);

        while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100){
            drive.liftRight.setPower(Math.max((3100-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
            drive.liftLeft.setPower(Math.min((3100-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
        }
        drive.liftRight.setPower(.001);
        drive.liftLeft.setPower(-.001);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(32.25,64, Math.PI))
                        .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                        .build()
        );

        drive.claw.setPosition(.45);
        sleep(250);
        drive.claw.setPosition(.8);



        //Sample 2
        if(!liftFail){
            drive.intakeLift.setPosition(1);
            drive.intakeLift2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(44.5, 40), 3*Math.PI/2)
                            .build()
            );

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            liftDown = false;
            intakeIn = false;
            double startTime1 = getRuntime();
            while((!liftDown || !intakeIn) && !liftFail && !intakeMiss1){
                if(getRuntime() - startTime1 > 4){
                    if(!liftDown){
                        liftFail = true;
                    }
                    if(!intakeIn){
                        intakeMiss1 = true;
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(44.5,40,3*Math.PI/2))
                                        .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                                        .build()
                        );
                    }
                }else{
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    }else{
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        liftDown = true;
                    }

                    if(!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)){
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.horizontal.setPosition(0.2);
                        drive.horizontal2.setPosition(0.4);
                    }else{
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

            if(!intakeMiss1 && !liftFail){
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(150);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.claw.setPosition(.45);
                double startTime1_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime1_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            drive.inRight.setPower(.75);
                            drive.inLeft.setPower(-.75);
                        }else{
                            break;
                        }
                    }
                }
                if(!transferFail){
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100){
                        drive.liftRight.setPower(Math.max((3100-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3100-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(44.5,40,5*Math.PI/4))
                                    .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                                    .build()
                    );

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);
                }
            }
        }


        //Sample 3
        if(!liftFail && !transferFail){
            drive.intakeLift.setPosition(1);
            drive.intakeLift2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(56, 40), 3*Math.PI/2)
                            .build()
            );

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            liftDown = false;
            intakeIn = false;
            double startTime2 = getRuntime();
            while((!liftDown || !intakeIn) && !liftFail && !intakeMiss2){
                if(getRuntime() - startTime2 > 4){
                    if(!liftDown){
                        liftFail = true;
                    }
                    if(!intakeIn){
                        intakeMiss2 = true;
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(56,40,3*Math.PI/2))
                                        .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                                        .build()
                        );
                    }
                }else{
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    }else{
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        liftDown = true;
                    }

                    if(!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)){
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.horizontal.setPosition(0.2);
                        drive.horizontal2.setPosition(0.4);
                    }else{
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

            if(!intakeMiss2 && !liftFail){
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(150);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.claw.setPosition(.45);
                double startTime1_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime1_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            drive.inRight.setPower(.75);
                            drive.inLeft.setPower(-.75);
                        }else{
                            break;
                        }
                    }
                }
                if(!transferFail){
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100){
                        drive.liftRight.setPower(Math.max((3100-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3100-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(56,40,3*Math.PI/2))
                                    .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                                    .build()
                    );

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }
            }
        }


        //Park or Sample4 -> Net of High
        if(getRuntime() < 25 && !liftFail && !transferFail){
            drive.intakeLift.setPosition(1);
            drive.intakeLift2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(52, 30), 31*Math.PI/16)
                            .build()
            );

            drive.liftRight.setPower(-1);
            drive.liftLeft.setPower(1);
            drive.inRight.setPower(-1);
            drive.inLeft.setPower(1);
            drive.horizontal.setPosition(0.4);
            drive.horizontal2.setPosition(0.6);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(52, 30, 31*Math.PI/16))
                            .waitSeconds(0.2)
                            .turnTo(28*Math.PI/16)
                            .build()
            );

            drive.liftLeftS.setPosition(0.875);
            drive.liftRightS.setPosition(1);

            liftDown = false;
            intakeIn = false;
            double startTime3 = getRuntime();
            while((!liftDown || !intakeIn) && !liftFail && !intakeMiss3){
                if(getRuntime() - startTime3 > 10){
                    if(!liftDown){
                        liftFail = true;
                    }
                    if(!intakeIn){
                        intakeMiss3 = true;
                        drive.inRight.setPower(1);
                        drive.inLeft.setPower(-1);
                        drive.horizontal.setPosition(0.6);
                        drive.horizontal2.setPosition(0.8);
                        drive.intakeLift.setPosition(0.48);
                        drive.intakeLift2.setPosition(0.48);
                        sleep(200);
                    }
                }else{
                    if (!drive.rightT.isPressed() && !drive.leftT.isPressed()){
                        drive.liftRight.setPower(-1);
                        drive.liftLeft.setPower(1);
                    }else{
                        drive.liftRight.setPower(0);
                        drive.liftLeft.setPower(0);
                        liftDown = true;
                    }

                    if(!(drive.inCol.red() > 200 || drive.inCol.blue() > 200)){
                        drive.inRight.setPower(-1);
                        drive.inLeft.setPower(1);
                        drive.horizontal.setPosition(0.2);
                        drive.horizontal2.setPosition(0.4);
                    }else{
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

            if(!intakeMiss3 && !liftFail){
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(200);

                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.claw.setPosition(.45);
                double startTime1_1 = getRuntime();
                while(!transferFail){
                    if(getRuntime() - startTime1_1 > 2){
                        if(drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            transferFail = true;
                            drive.claw.setPosition(.8);
                        }
                    }else {
                        if (drive.transfer.getDistance(DistanceUnit.MM) > 100){
                            drive.inRight.setPower(.75);
                            drive.inLeft.setPower(-.75);
                        }else{
                            break;
                        }
                    }
                }
                if(!transferFail){
                    drive.inRight.setPower(0);
                    drive.inLeft.setPower(0);
                    sleep(10);

                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    sleep(200);

                    drive.claw.setPosition(.8);
                    sleep(200);

                    drive.liftLeftS.setPosition(0.20);
                    drive.liftRightS.setPosition(0.80);

                    while (((drive.liftLeft.getCurrentPosition() * -1) < 3100) && drive.liftRight.getCurrentPosition() < 3100){
                        drive.liftRight.setPower(Math.max((3100-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                        drive.liftLeft.setPower(Math.min((3100-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
                    }
                    drive.liftRight.setPower(.001);
                    drive.liftLeft.setPower(-.001);


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(52, 30, 28*Math.PI/16))
                                    .strafeToSplineHeading(new Vector2d(57, 57), 5*Math.PI/4)
                                    .build()
                    );

                    drive.claw.setPosition(.45);
                    sleep(250);
                    drive.claw.setPosition(.8);

                    sleep(50);
                    drive.liftLeftS.setPosition(0.875);
                    drive.liftRightS.setPosition(1);
                }

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                                .strafeToSplineHeading(new Vector2d(48, 48), Math.PI)
                                .build()
                );
            }
        }else if(getRuntime() < 27.5 && !liftFail && !intakeMiss2 && !transferFail){
            while (((drive.liftLeft.getCurrentPosition() * -1) < 2000) && drive.liftRight.getCurrentPosition() < 2000){
                drive.liftRight.setPower(Math.max((3100-(drive.liftRight.getCurrentPosition())) * 0.01, 0.3));
                drive.liftLeft.setPower(Math.min((3100-(drive.liftLeft.getCurrentPosition() * -1)) * -0.01, -0.3));
            }
            drive.liftRight.setPower(.001);
            drive.liftLeft.setPower(-.001);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 12), Math.PI)
                            .strafeToSplineHeading(new Vector2d(62, 12), Math.PI)
                            .strafeToSplineHeading(new Vector2d(62, 55), Math.PI)
                            .strafeToSplineHeading(new Vector2d(48, 48), Math.PI)
                            .build()
            );

            while (((drive.liftLeft.getCurrentPosition() * -1) > 10) && drive.liftRight.getCurrentPosition() > 10){
                drive.liftRight.setPower(-1);
                drive.liftLeft.setPower(1);
            }
        }else if(!liftFail){
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(57,57,5*Math.PI/4))
                            .strafeToSplineHeading(new Vector2d(48, 48), 0)
                            .build()
            );

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
                    drive.actionBuilder(new Pose2d(48,48, 0))
                            .strafeToSplineHeading(new Vector2d(35, 10), 0)
                            .strafeToSplineHeading(new Vector2d(23, 10), 0)
                            .build()
            );

            while (((drive.liftLeft.getCurrentPosition() * -1) > 1200) && drive.liftRight.getCurrentPosition() > 1200){
                drive.liftRight.setPower(-1);
                drive.liftLeft.setPower(1);
            }
        }
        while(liftFail && getRuntime() < 28){
            drive.lightRight.setPosition(1);
            drive.lightLeft.setPosition(1);
            drive.liftRight.setPower(0);
            drive.liftLeft.setPower(0);
        }
    }
}
