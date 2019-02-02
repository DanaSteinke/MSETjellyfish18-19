

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "AutoCrater")
public class AutoCrater extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.driveBase.composeTelemetry();
        robot.markerDispenser.setPosition(0.55);
        //robot.samplingOrder.enable();


        waitForStart();
        robot.driveBase.setHeadingToZero();

        robot.Lift.ExtendingLift();
        robot.driveBase.CompassVectorDistance(1, 200, 90,0);
        robot.driveBase.CompassVectorDistance(1, 300, 0,0);
        robot.driveBase.CompassVectorDistance(1, 200, 270,0);
        robot.driveBase.CompassVectorDistance(1, 1600, 90,0);

        robot.driveBase.CompassVectorDistance(1, 4800, 180,0);
        robot.driveBase.gyroToGo(40,4);

        robot.driveBase.CompassVectorDistance(1, 3600, 180, 40);
        robot.markerDispenser.setPosition(0);
        sleep(500);
        robot.markerDispenser.setPosition(0.55);
        sleep(500);
        robot.driveBase.CompassVectorDistance(1,8000, 0,40);

        robot.Lift.DetractLift();
    }
}


