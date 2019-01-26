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
        robot.samplingOrder.enable();


        waitForStart();
        robot.driveBase.setHeadingToZero();

        //start auto

        //DETERMINE GOLD MINERAL LOCATION
        telemetry.addData("GOLD LOCATION: ", robot.samplingOrder.getGoldLocation());
        telemetry.update();
        sleep(1000);

        robot.samplingOrder.disable();
        robot.driveBase.VectorDistance(0.3,1000,0);

    }
}
