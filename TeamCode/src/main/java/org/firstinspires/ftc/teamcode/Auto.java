package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.driveBase.composeTelemetry();
        robot.markerDispenser.setPosition(0.55);

        waitForStart();
        robot.driveBase.setHeadingToZero();

        //Start Auto
        //test gyros
        robot.driveBase.gyroToGo(270);
        sleep(1000);
        robot.driveBase.gyroToGo(0);
        sleep(1000);
        robot.driveBase.gyroToGo(60);
        sleep(1000);
        robot.driveBase.gyroToGo(0);
    }
}
