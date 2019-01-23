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
        for(int i = 0; i < 5; i++) {
            robot.driveBase.gyroToGo(80);
            robot.driveBase.gyroToGo(270);
            robot.driveBase.gyroToGo(0);
        }
    }
}
