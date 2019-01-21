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
        for (int i = 1; i <= 4; i++) {
            robot.driveBase.gyroToGo(90 * i);
            sleep(1000);
        }
        for (int i = 1; i <= 4; i++){
            robot.driveBase.gyroToGo(45 + 90*i);
        }


    }
}
