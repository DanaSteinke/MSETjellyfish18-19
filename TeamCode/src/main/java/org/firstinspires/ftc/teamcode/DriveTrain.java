package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FerannoDad on 9/23/18.
 */

@TeleOp(name = "DriveTrain", group = "TeleOp")
public class DriveTrain extends LinearOpMode {
    //Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    /*
    private DcMotor lift;
    private DcMotor plow;
    private Servo markerDispenser;
    */

    //private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //lift = hardwareMap.dcMotor.get("lift");
        //plow = hardwareMap.dcMotor.get("plow");

        //initialize Servos
        //markerDispenser = hardwareMap.servo.get("markerDispenser");
        //intake = hardwareMap.crservo.get("intake");

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
        //Gamepad 1 Drive Train Controller
            //field centric drive
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            //Motor Encoder Positions
            /*
            telemetry.addData("Encoder Position", frontLeft.getCurrentPosition());
            telemetry.addData("Power", frontLeft.getPower());
            telemetry.addData("Lift Encoder Position", lift.getCurrentPosition());
            telemetry.addData("Lift Power ", lift.getPower());
            telemetry.update();
            */


         //Gamepad 2 Robot Controller
            //lift
            /*
            lift.setPower(gamepad2.left_stick_y);


            //plow
            plow.setPower(gamepad2.right_stick_y * 0.25);

            //marker disposer
            if (gamepad2.x) {
                markerDispenser.setPosition(0);
            } else if (gamepad2.y) {
                markerDispenser.setPosition(0.5);
            }
            */


            //intake servo
            /*
            intake.setPower((gamepad2.right_stick_y+1)/2);
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
/*
    public void LiftUp(double power, int position) {

        //Restart Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position
        lift.setTargetPosition(position);

        //set RUN_TO_POSITION mode
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(power);
        while (lift.isBusy()) {

            //wait until target position is reached

        }
        lift.setPower(0.0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    */
}

