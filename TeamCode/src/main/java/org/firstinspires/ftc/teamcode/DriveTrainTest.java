package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FerannoDad on 9/23/18.
 */

@TeleOp(name = "DriveTrainTest", group = "TeleOp")
public class DriveTrainTest extends LinearOpMode {
    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor lift;
    //private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        lift = hardwareMap.dcMotor.get("lift");
        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //initialize Servos
        //intake = hardwareMap.crservo.get("intake");

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
        //Gamepad 1 Drive Train Controller

            //tankDrive
            if(gamepad1.right_bumper){
                motorLeft.setPower(-gamepad1.left_stick_y*0.25);
                motorRight.setPower(-gamepad1.right_stick_y*0.25);
            }
            else {
                motorLeft.setPower(-gamepad1.left_stick_y);
                motorRight.setPower(-gamepad1.right_stick_y);
            }

        //Gamepad 2 Robot Controller
            //lift
            if(gamepad2.x){
                LiftUp(-1, -100);
            }
            else if(gamepad2.y){
                LiftUp(1,100);
            }
            else{
                lift.setPower(gamepad2.left_stick_y);
            }

            //intake servo
            /*
            if(gamepad2.a==false){
                intake.setPower(0.0);
            }
            else{
                intake.setPower(1.0);
            }
            */




            //wait for hardware to catch up
            idle();
        }

    }

    public void LiftUp(double power, int distance){

        //Restart Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position
        lift.setTargetPosition(lift.getCurrentPosition()+distance);

        //set RUN_TO_POSITION mode
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(power);
        while(lift.isBusy()){

            //wait until target position is reached

        }
        lift.setPower(0.0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
