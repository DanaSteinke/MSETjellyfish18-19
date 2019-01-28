package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        //robot.samplingOrder.enable();


        waitForStart();
        robot.driveBase.setHeadingToZero();

        //start auto

        //DETERMINE GOLD MINERAL LOCATION
        /*
        telemetry.addData("GOLD LOCATION: ", robot.samplingOrder.getGoldLocation());
        telemetry.update();
        robot.samplingOrder.disable();
        */


        //testing Compass VectorDistance
        robot.driveBase.VectorDistance(1, 5000, 0);
        robot.driveBase.CompassVectorDistance(1, 5000, 180);

    }
}
