package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {
    //Declare robot parts
    public DriveBase driveBase = null;
    public Lift Lift= null;
    public Servo markerDispenser;




    public Robot(LinearOpMode opMode) throws InterruptedException{
        //Initialize robot parts
        driveBase = new DriveBase(opMode);
        Lift = new Lift(opMode);
        markerDispenser = opMode.hardwareMap.servo.get("markerDispenser");

    }



}
