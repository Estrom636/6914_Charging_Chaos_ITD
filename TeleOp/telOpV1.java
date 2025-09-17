package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp - V1")
public class telOpV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(0, 0, 0));

        //Change slow speed height
        //Add slow for pick up specimen

        double tx = 0;
        double ty = 0;
        double ta = 0;

        boolean intakExt = false;
        boolean intakLift = false;
        boolean intakIn = false;
        boolean intakOut = false;

        boolean claw = false;
        boolean oldClaw = false;
        boolean clawTog = false;
        boolean pickUp = true;
        boolean scorePos = false;
        boolean liftPosOver = false;

        boolean red = false;
        boolean blue = false;
        boolean yellow = false;

        int liftPos = 0;
        int liftPosMM = 0;
        double proportionUp = 0.1;
        double proportionDown = 0.1;

        boolean aliColor = false;
        boolean oldAliColor = false;
        boolean aliColorTog = false;

        boolean scoreTyp = false;
        boolean oldScoreTyp = false;
        boolean scoreTypTog = false;

        boolean hang = false;
        boolean oldHang = false;
        boolean hangTog = false;
        boolean allOff = false;

        drive.lightLeft.setPosition(0.5);
        drive.lightRight.setPosition(0.5);


        drive.limelight.pipelineSwitch(2);

        waitForStart();
        resetRuntime();

        int snapNumber = 0;

        drive.lightLeft.setPosition(1);
        drive.lightRight.setPosition(1);

        while (!isStopRequested()) {
            telemetry.addData("Run Time: ", getRuntime());
            drive.limelight.start();
            drive.limelight.pipelineSwitch(2);
            LLResult resultAT = drive.limelight.getLatestResult();
            if(resultAT != null && resultAT.isValid()){
                if(snapNumber < 2){
                    drive.limelight.captureSnapshot("AprilTag" + snapNumber);
                    snapNumber++;
                }
                double xPos = resultAT.getBotpose().getPosition().x * 39.37;
                double yPos = resultAT.getBotpose().getPosition().y * 39.37;
                double yawPos = resultAT.getBotpose().getOrientation().getYaw() + 180;
                //telemetry.addData("xPos: ", xPos);
                //telemetry.addData("yPos: ", yPos);
                //telemetry.addData("yawPos: ", yawPos);
                drive.pose = new Pose2d(xPos, yPos, 0);
            }
            drive.limelight.stop();
            drive.updatePoseEstimate();
            //Driver One -> Drive and Intake

            //Drive
            if(gamepad2.left_stick_button){
                drive.limelight.start();
                drive.leftFront.setPower(tx * -0.04);
                drive.leftBack.setPower(tx * 0.04);
                drive.rightFront.setPower(tx * 0.04);
                drive.rightBack.setPower(tx * -0.04);
                telemetry.addLine("LimeLight Drive");
            }else if(intakExt || (liftPosMM > 1500) || (pickUp && scoreTypTog && clawTog)){
                drive.limelight.stop();
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (-1.33*Math.pow(gamepad1.left_stick_y, 3))*0.4,
                                (-1.33*Math.pow(gamepad1.left_stick_x, 3))*0.4
                        ),
                        -gamepad1.right_stick_x * 0.4
                ));
                telemetry.addLine("Half Speed");
            }else{
                drive.limelight.stop();
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -1.33*Math.pow(gamepad1.left_stick_y, 3),
                                -1.33*Math.pow(gamepad1.left_stick_x, 3)
                        ),
                        -gamepad1.right_stick_x * 0.6
                ));
                telemetry.addLine("Full Speed");
            }

            // Pick Color We Are
            aliColor = gamepad1.right_stick_button;

            if(aliColor && !oldAliColor){
                aliColorTog = !aliColorTog;
            }
            oldAliColor = aliColor;

            if(!aliColorTog){
                telemetry.addLine("We Are Blue");
                drive.limelight.pipelineSwitch(0);
                if (red) {
                    red = false;
                    intakLift = true;
                    intakOut = true;
                }
            }
            if(aliColorTog){
                telemetry.addLine("We Are Red");
                drive.limelight.pipelineSwitch(3);
                if (blue) {
                    blue = false;
                    intakLift = true;
                    intakOut = true;
                }
            }

            //limelight Results
            LLResult result = drive.limelight.getLatestResult();

            if(result != null && result.isValid()){
                tx = result.getTx();
                ty = result.getTy() + 11.25;
                ta = result.getTa();

                //telemetry.addData("Target X", tx);
                //telemetry.addData("Target Y", ty);
                //telemetry.addData("Target A", ta);

                double yDis = 5.375 / Math.tan(ty * (Math.PI/180));

                int direc = 0;
                if(tx > 0) direc = 1;
                if(tx < 0) direc = -1;

                double xDis = yDis * Math.tan(Math.abs(tx) * (Math.PI/180));
                xDis = xDis * direc;

                //telemetry.addData("Target xDis", xDis);
                //telemetry.addData("Target yDis", yDis);
            }else{
                //telemetry.addData("Limelight", "No Targets");
            }

            // intake Ext
            if(gamepad1.right_bumper){
                intakExt = true;
            }
            if(gamepad1.left_bumper){
                intakExt = false;
            }

            if(intakExt){
                drive.horizontal.setPosition(0.2);
                drive.horizontal2.setPosition(0.4);
            }else{
                drive.horizontal.setPosition(0.6);
                drive.horizontal2.setPosition(0.8);
            }

            //intake Lift
            if(gamepad1.dpad_up){
                intakLift = false;
            }
            if(gamepad1.dpad_down){
                intakLift = true;
                intakIn = true;
            }

            if(intakLift){
                drive.intakeLift.setPosition(0.98);
                drive.intakeLift2.setPosition(0.98);
            }else{
                drive.intakeLift.setPosition(0.48);
                drive.intakeLift2.setPosition(0.48);
            }

            //intake wheels
            if(gamepad1.y){
                intakIn = false;
                intakOut = false;
            }
            if(gamepad1.b){
                intakIn = true;
                intakOut = false;
            }
            if(gamepad1.x){
                intakIn = false;
                intakOut = true;
            }
            if((drive.inCol.red() > 200 || drive.inCol.blue() > 200) && drive.intakeLift.getPosition() != 1){
                intakIn = false;
                intakLift = false;
                if(drive.intakeLift.getPosition() == .48){
                    sleep(20);
                    intakExt = false;
                }
            }


            if(drive.transfer.getDistance(DistanceUnit.MM) < 80){
                drive.inRight.setPower(0);
                drive.inLeft.setPower(0);
                intakIn = false;
                intakOut = false;
            }else if(!scoreTypTog && drive.intakeLift.getPosition() == 0.48 && drive.horizontal.getPosition() == 0.6 && drive.transfer.getDistance(DistanceUnit.MM) > 135 && drive.transfer.getDistance(DistanceUnit.MM) < 150){
                drive.inRight.setPower(.75);
                drive.inLeft.setPower(-.75);
            }else if (intakOut) {
                drive.inRight.setPower(1);
                drive.inLeft.setPower(-1);
            } else if(intakIn){
                drive.inRight.setPower(-1);
                drive.inLeft.setPower(1);
            }else{
                drive.inRight.setPower(0);
                drive.inLeft.setPower(0);
            }


            //Driver 2 -> Lift and Claw

            //Claw
            claw = gamepad2.right_bumper;

            if(claw && !oldClaw){
                clawTog = !clawTog;
            }
            oldClaw = claw;

            if(clawTog){
                drive.claw.setPosition(.45);
            }else{
                drive.claw.setPosition(.8);
            }

            //Lift Arm Pos override rest
            if(liftPos <= 10){
                liftPosOver = false;
            }

            //telemetry.addData("Left Servo Pos", drive.liftLeftS.getPosition());
            //telemetry.addData("Right Servo Pos", drive.liftRightS.getPosition());


            //Pick Up
            if(gamepad2.dpad_down){
                liftPosOver = true;
                pickUp = true;
                scorePos = false;
            }
            //internal
            if(pickUp && !scoreTypTog){
                if(drive.liftLeftS.getPosition() <= 0.6){
                    clawTog = false;
                    drive.claw.setPosition(.8);
                    sleep(25);
                }
                if(gamepad2.dpad_left && (drive.transfer.getDistance(DistanceUnit.MM) < 90 || drive.transfer.getDistance(DistanceUnit.MM) > 1000)){
                    drive.liftLeftS.setPosition(0.975);
                    drive.liftRightS.setPosition(0.95);
                    if(drive.liftLeftS.getPosition() == 0.975){
                        sleep(10);
                        clawTog = false;
                    }
                }else{
                    drive.liftLeftS.setPosition(0.9);
                    drive.liftRightS.setPosition(0.95);
                }

            }
            //external
            if(pickUp && scoreTypTog){
                if(drive.liftLeftS.getPosition() >= 0.9){
                    clawTog = false;
                    drive.claw.setPosition(.8);
                    sleep(25);
                }
                if(clawTog){
                    drive.liftLeftS.setPosition(0.275);
                    drive.liftRightS.setPosition(0.65);
                }else{
                    sleep(100);
                    drive.liftLeftS.setPosition(0.6);
                    drive.liftRightS.setPosition(0.65);
                }
            }

            //Scoring Type Selection
            scoreTyp = gamepad2.right_stick_button;

            if(scoreTyp && !oldScoreTyp){
                scoreTypTog = !scoreTypTog;
            }
            oldScoreTyp = scoreTyp;

            if(!scoreTypTog){
                telemetry.addLine("Samples");
            }
            if(scoreTypTog){
                telemetry.addLine("Specimens");
            }

            //Score Location
            if(gamepad2.dpad_right || (liftPos >= 2500 && !liftPosOver)){
                liftPosOver = true;
                scorePos = true;
                pickUp = false;
            }
            //Basket
            if(scorePos && !scoreTypTog){
                if(drive.liftLeftS.getPosition() >= 0.9){
                    clawTog = false;
                    drive.claw.setPosition(.8);
                    sleep(25);
                }
                drive.liftLeftS.setPosition(0.20);
                drive.liftRightS.setPosition(0.80);
                if(claw){
                    yellow = false;
                    blue = false;
                    red = false;
                }
            }
            //Chamber
            if(scorePos && scoreTypTog){
                if(drive.liftLeftS.getPosition() >= 0.9){
                    clawTog = false;
                    drive.claw.setPosition(.8);
                    sleep(25);
                }
                drive.liftRightS.setPosition(0.7);
                drive.liftLeftS.setPosition(0.5);
            }

            //Lift Run Driver
            if(gamepad2.right_trigger > 0 && liftPos < 3100){
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

            while(hangTog){
                hang = gamepad2.left_bumper;

                if(hang && !oldHang){
                    hangTog = !hangTog;
                }
                oldHang = hang;

                drive.liftRight.setPower(-0.75);
                drive.liftLeft.setPower(0.75);
            }

            //lift pos Stuff
            liftPos = ((drive.liftLeft.getCurrentPosition() * -1) + drive.liftRight.getCurrentPosition()) / 2;
            liftPosMM = (int) (liftPos * 0.823);
            proportionUp = (double) (3100 - liftPos) * .005;
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
            telemetry.addData("Lift Pos MM: ", liftPosMM);

            // Set Current Color
            telemetry.addData("red", drive.inCol.red());
            telemetry.addData("blue", drive.inCol.blue());
            if(drive.inCol.red() > 1600 && drive.inCol.blue() > 450){
                yellow = true;
                blue = false;
                red = false;
            } else if (drive.inCol.red() > 900 && drive.inCol.blue() < 300) {
                red = true;
                blue = false;
                yellow = false;
            } else if (drive.inCol.blue() > 950) {
                blue = true;
                red = false;
                yellow = false;
            }
            if(gamepad1.left_stick_button){
                blue = false;
                red = false;
                yellow = false;
            }

            //Light Control
            if(gamepad2.left_stick_button){
                drive.lightLeft.setPosition(0.5);
                drive.lightRight.setPosition(0.5);
            }else if((drive.leftT.isPressed() || drive.rightT.isPressed()) && (drive.transfer.getDistance(DistanceUnit.MM) < 90) && (drive.claw.getPosition() == 0.45)){
                drive.lightLeft.setPosition(0.722);
                drive.lightRight.setPosition(0.722);
            }else if(yellow){
                telemetry.addLine("YELLOW");
                drive.lightLeft.setPosition(0.388);
                drive.lightRight.setPosition(0.388);
            }else if(red){
                telemetry.addLine("RED");
                drive.lightLeft.setPosition(0.28);
                drive.lightRight.setPosition(0.28);
            }else if(blue){
                telemetry.addLine("BLUE");
                drive.lightLeft.setPosition(0.611);
                drive.lightRight.setPosition(0.611);
            }else{
                drive.lightLeft.setPosition(1);
                drive.lightRight.setPosition(1);
            }


            telemetry.addData("Dist Sensor: ", drive.transfer.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
