package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "AutoMarker")
public class AutoMarker extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.driveBase.composeTelemetry();
        robot.markerDispenser.setPosition(0.55);


        waitForStart();
        robot.driveBase.setHeadingToZero();

        //Start Auto
        //extend lift
        robot.Lift.ExtendingLift();
        robot.driveBase.CompassVectorDistance(1, 200, 90,0);
        robot.driveBase.CompassVectorDistance(1, 300, 0,0);
        robot.driveBase.CompassVectorDistance(1, 200, 270,0);
        robot.driveBase.CompassVectorDistance(1, 300, 90, 0);

        robot.driveBase.gyroToGo(270);
        robot.driveBase.CompassVectorDistance(1, 5000, 180,270);
        robot.markerDispenser.setPosition(0);
        sleep(500);
        robot.markerDispenser.setPosition(0.55);
        sleep(500);
        robot.driveBase.gyroToGo(210);
        robot.driveBase.CompassVectorDistance(1, 8000, 0, 220);

        robot.Lift.DetractLift();

    }
}
