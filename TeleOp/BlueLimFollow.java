package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "limFollow", group = "Sensor")
@Disabled
public class BlueLimFollow extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap, new Pose2d(0, 0, 0));

        drive.intakeLift.setPosition(0.5);
        drive.horizontal.setPosition(0.60);

        drive.liftLeftS.setPosition(0.875);
        drive.liftRightS.setPosition(1);

        drive.claw.setPosition(.8);

        drive.lightRight.setPosition(0.3);
        drive.lightLeft.setPosition(0.3);

        Limelight3A limelight;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        LLResult result = limelight.getLatestResult();

        while (result.getTx() < 10 || 10 < result.getTx()) {

            telemetry.addData("tx", result.getTx());
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(0 + result.getTx())
                            .build()
            );
        }
    }
}
