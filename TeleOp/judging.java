package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Judging Lights")
public class judging extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(0, 0, 0));

        //variables for toggling through colors
        boolean color = false;
        boolean colorTog = false;
        boolean oldcolor = false;
        int colorCycle = 0;

        //not used
        double coLor = 0.3;
        boolean up = true;

        waitForStart();
        resetRuntime();

        while(!isStopRequested()){
            //to get button on controller
            color = gamepad1.a;

            //toggle to increase what color id displaed
            if(color && !oldcolor){
                colorTog = !colorTog;
                colorCycle++;
                if(colorCycle > 3){
                    colorCycle = 0;
                }
            }
            oldcolor = color;

            //ifs to display color based on number in cycle
            if(colorCycle == 0){
                //no color
                drive.lightLeft.setPosition(0);
                drive.lightRight.setPosition(0);
                drive.intakeLight.setPosition(0);
            }
            if (colorCycle == 1) {
                //sample color
                drive.lightLeft.setPosition(0.28);
                drive.lightRight.setPosition(0.28);
                drive.intakeLight.setPosition(0);
            }
            if (colorCycle == 2) {
                //color purple
                drive.lightLeft.setPosition(0.722);
                drive.lightRight.setPosition(0.722);
                drive.intakeLight.setPosition(0);
            }
            if (colorCycle == 3) {
                //color green
                drive.lightLeft.setPosition(0.5);
                drive.lightRight.setPosition(0.5);
                drive.intakeLight.setPosition(0);
            }
        }
    }

}
