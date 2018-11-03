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
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor lift;
    private DcMotor plow;
    private Servo markerDispenser;

    //private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        lift = hardwareMap.dcMotor.get("lift");
        plow = hardwareMap.dcMotor.get("plow");
        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //initialize Servos
        markerDispenser = hardwareMap.servo.get("markerDispenser");
        //intake = hardwareMap.crservo.get("intake");

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
            //Gamepad 1 Drive Train Controller

            //tankDrive
            if (gamepad1.right_bumper) {
                motorRight.setPower(-gamepad1.left_stick_y * 0.35);
                motorLeft.setPower(-gamepad1.right_stick_y * 0.35);
            } else {
                motorRight.setPower(-gamepad1.left_stick_y);
                motorLeft.setPower(-gamepad1.right_stick_y);
            }
            //Encoder Position
            telemetry.addData("Encoder Position", motorLeft.getCurrentPosition());
            telemetry.addData("Power", motorLeft.getPower());
            telemetry.addData("Lift Encoder Position", lift.getCurrentPosition());
            telemetry.addData("Lift Power ", lift.getPower());
            telemetry.update();


            //Gamepad 2 Robot Controller
            //lift
            lift.setPower(gamepad2.left_stick_y);


            //plow
            plow.setPower(gamepad2.right_stick_y * 0.25);

            //marker disposer
            if (gamepad2.x) {
                markerDispenser.setPosition(0);
            } else if (gamepad2.y) {
                markerDispenser.setPosition(0.5);
            }


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
}

