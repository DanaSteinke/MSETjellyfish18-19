package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {

    //Declare robot parts
    public DriveBase driveBase = null;
    public Lift Lift = null;
    public Servo markerDispenser;
    public DcMotor intakePivot;
    public DcMotor intake;
    public DcMotor elevator;

    public GoldAlign goldAlign;
    public SamplingOrder samplingOrder;

    public Robot(LinearOpMode opMode) throws InterruptedException {
        //Initialize robot parts
        driveBase = new DriveBase(opMode);
        Lift = new Lift(opMode);
        goldAlign = new GoldAlign(opMode);
        samplingOrder = new SamplingOrder(opMode);

        markerDispenser = opMode.hardwareMap.servo.get("markerDispenser");
        intakePivot = opMode.hardwareMap.dcMotor.get("intakePivot");
        intake = opMode.hardwareMap.dcMotor.get("intake");
        elevator = opMode.hardwareMap.dcMotor.get("elevator");

    }

    public void driveGoldAlign(){
        driveBase.driveForward(0.3);
    }


}
