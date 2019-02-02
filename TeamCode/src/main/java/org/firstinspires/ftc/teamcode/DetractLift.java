package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "DetractLift")
public class DetractLift extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.driveBase.composeTelemetry();
        robot.markerDispenser.setPosition(0.55);

        waitForStart();
        robot.driveBase.setHeadingToZero();

        //Start Auto
        robot.Lift.DetractLift();

    }

}
