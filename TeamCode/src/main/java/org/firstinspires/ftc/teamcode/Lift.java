package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by FerannoDad on 9/23/18.
 */


public class Lift {
    LinearOpMode opMode;

    public DcMotor lift;
    public TouchSensor topLimit;

    public Lift(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;

        //initialize motors
        lift = opMode.hardwareMap.dcMotor.get("lift");
        topLimit= opMode.hardwareMap.touchSensor.get("topLimit");


    }

    public void setPower(double power){
        lift.setPower(power);
    }

    public void EncoderDetractLift(double power, int distance) throws InterruptedException{

        //Restart Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position

        lift.setTargetPosition(lift.getCurrentPosition()+distance);

        //set RUN_TO_POSITION mode
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(power);

        while(lift.isBusy()){
            //wait until target position is reached
            opMode.telemetry.update();
            opMode.telemetry.addData("Lift Encoder Position", lift.getCurrentPosition());
            opMode.telemetry.addData("get Target Position", lift.getTargetPosition());

        }
        lift.setPower(0.0);
        opMode.sleep(100);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void TimedExtendLift(double power, long time) throws IllegalArgumentException{
        lift.setPower(power);
        opMode.sleep(time);
    }

    //MAGNETIC LIMIT SWITCH(TOUCH SENSOR)
    public void DetractLift() throws InterruptedException{
        TimedExtendLift(-1,500);

        while(!topLimit.isPressed() && opMode.opModeIsActive()){
            //if top limit switch is not pressed, go up
            lift.setPower(-1);

        }
        lift.setPower(0.0);
        opMode.sleep(100);
    }
    public void ExtendingLift() throws InterruptedException{
        TimedExtendLift(1,500);
        while(!topLimit.isPressed() && opMode.opModeIsActive()){
            //if top limit switch is not pressed, go up
            lift.setPower(1);
        }
        lift.setPower(0.0);
        opMode.sleep(100);
    }




}
