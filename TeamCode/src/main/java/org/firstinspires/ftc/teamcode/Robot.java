package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot {
    //Declare robot parts
    DriveBase driveBase = null;



    public Robot(LinearOpMode opMode) throws InterruptedException{
        //Initialize robot parts
        driveBase = new DriveBase(opMode);
    }

}
