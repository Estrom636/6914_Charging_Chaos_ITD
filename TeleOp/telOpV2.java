package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Driver;

@TeleOp(name = "TeleOp - V2")
public class telOpV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize robot based on if there is a value stored in PoseStroage
        Hardware drive;
        if(PoseStorage.currentPose == null){
            drive = new Hardware(hardwareMap, new Pose2d(0, 0, 0));
        }else{
            drive = new Hardware(hardwareMap, PoseStorage.currentPose);
        }


        //Initialize intake set position variables
        boolean intakExt = false;
        boolean intakLift = false;
        boolean intakIn = false;
        boolean intakOut = false;


        //Initialize claw and lift arm set position variables
        boolean claw = false;
        boolean clawTog = false;
        boolean pickUp = true;
        boolean wall = false;
        boolean scorePos = false;
        boolean liftPosOver = false;


        //Initialize color variables
        boolean red = false;
        boolean blue = false;
        boolean yellow = false;


        //Initialize lift variables
        int liftPos = 0;
        int liftPosMM = 0;
        double proportionUp = 0.1;
        double proportionDown = 0.1;

        //Initialize aliance color toggle variables
        boolean aliColor = false; //true = red, false = blue
        boolean oldAliColor = false;
        boolean aliColorTog = PoseStorage.aliCol;

        //Initialize limelight active toggle variables
        boolean limeL = false;
        boolean oldlimeL = false;
        boolean limeLTog = false;

        //Initialize scoreing type toggle variables
        boolean scoreTyp = false; //true = SPECIMEN, false = SAMPLE
        boolean oldScoreTyp = false;
        boolean scoreTypTog = PoseStorage.scoreType; //importing from PoseStorage


        //Initialize end game hang toggle variables
        boolean hang = false;
        boolean oldHang = false;
        boolean hangTog = false;
        boolean allOff = false;

        //Initialize limelight data variables
        double tx = 0;
        double ty = 0;
        double ta = 0;


        //Setting LED colors
        drive.lightLeft.setPosition(0.5);
        drive.lightRight.setPosition(0.5);
        drive.intakeLight.setPosition(0);

        //Starting limelight
        drive.limelight.start();
        drive.limelight.pipelineSwitch(3);

        waitForStart();
        resetRuntime();


        drive.lightLeft.setPosition(1);
        drive.lightRight.setPosition(1);


        while (!isStopRequested()) {
            telemetry.addData("Run Time: ", getRuntime());
            //telemetry.addData("Pose X", drive.pose.position.x);
            //telemetry.addData("Pose Y", drive.pose.position.y);
            //telemetry.addData("Pose H", drive.pose.heading.real);
            //telemetry.addData("Left", drive.lBack.getDistance(DistanceUnit.MM));
            //telemetry.addData("Right", drive.rBack.getDistance(DistanceUnit.MM));

            //setting limelight pipeline based on alliance color and scoring type
            if(!aliColorTog && scoreTypTog){
                //blue
                drive.limelight.pipelineSwitch(0);
                telemetry.addLine("Limelight pipeline is 0");
            }else if(aliColorTog && scoreTypTog){
                //red
                drive.limelight.pipelineSwitch(1);
                telemetry.addLine("Limelight pipeline is 1");
            }else{
                //yellow
                drive.limelight.pipelineSwitch(3);
                telemetry.addLine("Limelight pipeline is 3");
            }

            //Limelight
            limeL = gamepad1.a;

            //toggle for limelight
            if(limeL && !oldlimeL){
                limeLTog = !limeLTog;
            }
            oldlimeL = limeL;

            //getting and setting limelight data to variables if limelight see data
            LLResult result = drive.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();
                ta = result.getTa();

                telemetry.addData("Target X", tx);
                //telemetry.addData("Target Y", ty);
                //telemetry.addData("Target Area", ta);
            } else {
                tx = 0;
                ty = 0;
                ta = 0;
                telemetry.addData("Limelight", "No Targets");
            }

            //reset the limelight variables if active limelight is turned off
            if(!limeLTog){
                tx = 0;
                ty = 0;
                ta = 0;
            }


            //updateing robot position with Road Runner and saving it to PoseStorage
            drive.updatePoseEstimate();
            PoseStorage.currentPose = drive.pose;

            //Driver One -> Drive and Intake

            //Drive
            //robot-centric drive code
            //slow robot down if intake is out, lift is above a set height or if robot is in wall pick up positions
            //all have limelight to aim at SAMPLE added onto the rotation
            if(intakExt || (liftPosMM > 1500) || (pickUp && scoreTypTog && clawTog)){
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (-1.33*Math.pow(gamepad1.left_stick_y, 3))*0.4,
                                (-1.33*Math.pow(gamepad1.left_stick_x, 3))*0.4
                        ),
                        -gamepad1.right_stick_x * 0.4 + ((tx/27.25) * 0.4)
                ));
                if(limeLTog){
                    telemetry.addLine("Half Speed + LimeLight");
                }else{
                    telemetry.addLine("Half Speed");
                }
            }else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -1.33 * Math.pow(gamepad1.left_stick_y, 3),
                                -1.33 * Math.pow(gamepad1.left_stick_x, 3)
                        ),
                        -gamepad1.right_stick_x * 0.6 + ((tx / 27.25) * 0.4)
                ));
                if(limeLTog){
                    telemetry.addLine("Full Speed + LimeLight");
                }else{
                    telemetry.addLine("Full Speed");
                }
            }


            // Pick Color We Are
            aliColor = gamepad1.right_stick_button;

            //alliance coor toggle
            if(aliColor && !oldAliColor){
                aliColorTog = !aliColorTog;
            }
            oldAliColor = aliColor;


            //setting telemetry color line
            //and if other color is in intake auto eject
            if(!aliColorTog){
                telemetry.addLine("We Are Blue");
                if (red) {
                    red = false;
                    intakLift = true;
                    intakOut = true;
                }
            }
            if(aliColorTog){
                telemetry.addLine("We Are Red");
                if (blue) {
                    blue = false;
                    intakLift = true;
                    intakOut = true;
                }
            }

            // intake Ext
            //intake out if true
            if (gamepad1.right_bumper) {
                intakExt = true;
            } else if (gamepad1.left_bumper) {
                intakExt = false;
            }
            //first value is when true
            drive.horizontal.setPosition(intakExt ? 0.2 : 0.6);
            drive.horizontal2.setPosition(intakExt ? 0.4 : 0.8);

            //intake Lift
            //intake down if true [when clicked down intake is set to go in]
            if(gamepad1.dpad_up){
                intakLift = false;
            }else if(gamepad1.dpad_down){
                drive.limelight.captureSnapshot("IntakeDown" + PoseStorage.intakeCount++); //taking snapshot so you can go back later if needed to adjust values in limelight
                intakLift = true;
                intakIn = true;
            }
            //first value is when true
            drive.intakeLift.setPosition(intakLift ? 0.98 : 0.48);
            drive.intakeLift2.setPosition(intakLift ? 0.98 : 0.48);

            //intake wheels
            //setting intake wheel directions
            if(gamepad1.y){
                intakIn = false;
                intakOut = false;
            }else if(gamepad1.b){
                intakIn = true;
                intakOut = false;
            }else if(gamepad1.x){
                intakIn = false;
                intakOut = true;
            }

            //auto intake up and in if a SAMPLE is in the intake
            if((drive.inCol.red() > 200 || drive.inCol.blue() > 200) && drive.intakeLift.getPosition() != 1){
                intakIn = false;
                intakLift = false;
                //SPECIMEN is score type dont retract
                if(drive.intakeLift.getPosition() == .48 && !scoreTypTog){
                    sleep(20);
                    intakExt = false;
                }
            }

            //different transfer types
            if(drive.transfer.getDistance(DistanceUnit.MM) < 80){
                //SAMPLE in transfer locations -> stop all
                drive.inRight.setPower(0);
                drive.inLeft.setPower(0);
                intakIn = false;
                intakOut = false;
                drive.intakeLight.setPosition(0);
            }else if(!scoreTypTog && drive.intakeLift.getPosition() == 0.48 && drive.horizontal.getPosition() == 0.6 && drive.transfer.getDistance(DistanceUnit.MM) > 135 && drive.transfer.getDistance(DistanceUnit.MM) < 150){
                //SAMPLE is in position to go into transfer location -> eject SAMPLE into transfer location
                drive.inRight.setPower(.75);
                drive.inLeft.setPower(-.75);
                drive.intakeLight.setPosition(0);
            }else if (intakOut) {
                //set intake out -> wheels out, intake light off, and reset color values
                drive.inRight.setPower(1);
                drive.inLeft.setPower(-1);
                drive.intakeLight.setPosition(0);
                yellow = false;
                blue = false;
                red = false;
            } else if(intakIn){
                //set intake in -> wheels in, and intake light on
                drive.inRight.setPower(-1);
                drive.inLeft.setPower(1);
                drive.intakeLight.setPosition(0.3);//TODO Light Control
            }else{
                //intake is off
                drive.inRight.setPower(0);
                drive.inLeft.setPower(0);
                drive.intakeLight.setPosition(0);
            }

            //Driver 2 -> Lift and Claw

            //snapshot
            if(gamepad2.a) drive.limelight.captureSnapshot("Manual" + PoseStorage.snapCount++);

            //Claw
            if (gamepad2.right_bumper && !claw) {
                clawTog = !clawTog;
            }

            claw = gamepad2.right_bumper;

            drive.claw.setPosition(clawTog ? 0.45 : 0.8);

            //Lift Arm Pos override rest
            if(liftPos <= 10){
                liftPosOver = false;
            }

            // Pick Up
            if (gamepad2.dpad_down) {
                liftPosOver = true;
                pickUp = true;
                scorePos = false;
                wall = false;
            }


            // Internal Pick Up
            if (pickUp && !scoreTypTog) {
                if (drive.liftLeftS.getPosition() <= 0.6) {
                    clawTog = false;
                    drive.claw.setPosition(0.8);
                    sleep(25);
                }
                if (gamepad2.dpad_left && (drive.transfer.getDistance(DistanceUnit.MM) < 90 || drive.transfer.getDistance(DistanceUnit.MM) > 1000)) {
                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    if (drive.liftLeftS.getPosition() == 0.975) {
                        sleep(10);
                        clawTog = false;
                    }
                } else {
                    drive.liftLeftS.setPosition(0.9);
                    drive.liftRightS.setPosition(0.95);
                }
            }


            // External Pick Up
            if (pickUp && scoreTypTog && !wall) {
                    clawTog = false;
                    drive.claw.setPosition(0.8);
                    sleep(25);
                drive.liftLeftS.setPosition(0.6);
                drive.liftRightS.setPosition(0.65);
            }

            if (gamepad2.dpad_left) {
                liftPosOver = true;
                pickUp = true;
                scorePos = false;
                wall = true;
            }
            if (wall && scoreTypTog) {
                if (drive.liftLeftS.getPosition() >= 0.9) {
                    clawTog = false;
                    drive.claw.setPosition(0.8);
                    sleep(25);
                }
                drive.liftLeftS.setPosition(0.25);
                drive.liftRightS.setPosition(0.65);
            }

            // Scoring Type Selection
            scoreTyp = gamepad2.right_stick_button;

            if (scoreTyp && !oldScoreTyp) {
                scoreTypTog = !scoreTypTog;
            }
            oldScoreTyp = scoreTyp;

            telemetry.addLine(scoreTypTog ? "Specimens" : "Samples");

            // Score Location
            if (gamepad2.dpad_right || (liftPos >= 2500 && !liftPosOver)) {
                liftPosOver = true;
                scorePos = true;
                pickUp = false;
                wall = false;
            }

            // Basket
            if (scorePos && !scoreTypTog) {
                if (drive.liftLeftS.getPosition() >= 0.9) {
                    clawTog = false;
                    drive.claw.setPosition(0.8);
                    sleep(25);
                }
                drive.liftLeftS.setPosition(0.20);
                drive.liftRightS.setPosition(0.80);
                if (claw) {
                    yellow = false;
                    blue = false;
                    red = false;
                }
            }

            // Chamber
            if (scorePos && scoreTypTog) {
                if (drive.liftLeftS.getPosition() >= 0.9) {
                    clawTog = false;
                    drive.claw.setPosition(0.8);
                    sleep(25);
                }
                drive.liftLeftS.setPosition(0.20);
                drive.liftRightS.setPosition(0.35);
            }

            //Lift Run Driver
            if(gamepad2.right_trigger > 0 && liftPos < 2900){
                drive.liftRight.setPower(gamepad2.right_trigger * proportionUp);
                drive.liftLeft.setPower(-gamepad2.right_trigger * proportionUp);
            } else if (gamepad2.left_trigger > 0 && liftPos > 10) {
                drive.liftRight.setPower(-gamepad2.left_trigger * proportionDown);
                drive.liftLeft.setPower(gamepad2.left_trigger * proportionDown);
            }else if(!gamepad2.a){
                drive.liftRight.setPower(.001);
                drive.liftLeft.setPower(-.001);
            }


            hang = gamepad2.left_bumper;

            if(hang && !oldHang){
                hangTog = !hangTog;
            }
            oldHang = hang;


            double color = 0.3;
            boolean up = true;
            while(hangTog){
                hang = gamepad2.left_bumper;

                if(hang && !oldHang){
                    hangTog = !hangTog;
                }
                oldHang = hang;

                if(up && color > 0.7){
                    up = false;
                }
                if(!up && color < 0.3){
                    up = true;
                }

                if(up){
                    color += .001;
                }
                if(!up){
                    color -= .001;
                }

                drive.lightRight.setPosition(0 + color);
                drive.lightLeft.setPosition(1 - color);

                drive.liftRight.setPower(-0.005);
                drive.liftLeft.setPower(0.005);
                drive.inLeft.setPower(0);
                drive.inRight.setPower(0);
            }

            //lift pos Stuff
            liftPos = ((drive.liftLeft.getCurrentPosition() * -1) + drive.liftRight.getCurrentPosition()) / 2;
            liftPosMM = (int) (liftPos * 0.823);
            if(scoreTypTog){
                proportionUp = (double) (1100 - liftPos) * .001;
            }else{
                proportionUp = (double) (2900 - liftPos) * .001;
            }
            proportionDown = (double) (liftPos) * .1;
            if((drive.leftT.isPressed() || drive.rightT.isPressed()) && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
                drive.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else {
                drive.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //telemetry.addData("Left Motor", drive.liftLeft.getCurrentPosition() * -1);
            //telemetry.addData("Right Motor: ", drive.liftRight.getCurrentPosition());
            telemetry.addData("Lift Pos: ", liftPos);
            //telemetry.addData("Lift Pos MM: ", liftPosMM);

            // Set Current Color
            //telemetry.addData("red", drive.inCol.red());
            //telemetry.addData("blue", drive.inCol.blue());
            if(drive.inCol.red() > 1600 && drive.inCol.blue() > 450){
                yellow = true;
                blue = false;
                red = false;
               // limeLTog = false;
            } else if (drive.inCol.red() > 900 && drive.inCol.blue() < 300) {
                red = true;
                blue = false;
                yellow = false;
                //limeLTog = false;
            } else if (drive.inCol.blue() > 950) {
                blue = true;
                red = false;
                yellow = false;
                //limeLTog = false;
            }
            if(gamepad2.left_stick_button){
                blue = false;
                red = false;
                yellow = false;
            }


            //Light Control
            if (!(tx == 0 && ty == 0 && ta == 0) && limeLTog) {
                //Green
                drive.lightLeft.setPosition(0.5);
                drive.lightRight.setPosition(0.5);
            } else if ((drive.leftT.isPressed() || drive.rightT.isPressed()) && (drive.transfer.getDistance(DistanceUnit.MM) < 90) && (drive.claw.getPosition() == 0.45)) {
                //Purple
                drive.lightLeft.setPosition(0.722);
                drive.lightRight.setPosition(0.722);
            } else {
                if (yellow) {
                    //Yellow
                    telemetry.addLine("YELLOW");
                    drive.lightLeft.setPosition(0.388);
                    drive.lightRight.setPosition(0.388);
                } else if (red) {
                    //Red
                    telemetry.addLine("RED");
                    drive.lightLeft.setPosition(0.28);
                    drive.lightRight.setPosition(0.28);
                } else if (blue) {
                    //Blue
                    telemetry.addLine("BLUE");
                    drive.lightLeft.setPosition(0.611);
                    drive.lightRight.setPosition(0.611);
                } else {
                    drive.lightLeft.setPosition(1);
                    drive.lightRight.setPosition(1);
                }
            }

            telemetry.addData("Dist Sensor: ", drive.transfer.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}




