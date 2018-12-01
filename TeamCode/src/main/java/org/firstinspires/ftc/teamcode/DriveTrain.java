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
    hMap robot = new hMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
        //Gamepad 1 Drive Train Controller
            //field centric drive
            //because of joystick positions, change yValue to reverse, then Quadrants will be in position
            //for Jae purposes . . .
            double yValue=gamepad1.left_stick_y;
            double xValue=gamepad1.left_stick_x;
            double r = Math.hypot(xValue, yValue);
            double robotAngle = Math.atan2(yValue, xValue) - Math.PI / 4;
            double rotationalPower = gamepad1.right_stick_x;

            //calculate direction of voltages to max value with sine cosine functions
            //voltages are without magnitude(joystick) nor rotational Power yet
            double v1 = r * Math.cos(robotAngle) + rotationalPower;
            double v2 = r *  Math.sin(robotAngle) - rotationalPower;
            double v3 = r *  Math.sin(robotAngle) + rotationalPower;
            double v4 = r * Math.cos(robotAngle) - rotationalPower;
            double[] vArray={v1,v2,v3,v4};

            telemetry.update();
            telemetry.addData("right_bumper",gamepad1.right_bumper);
            if(gamepad1.right_bumper==true) {
                robot.frontLeft.setPower(vArray[0] * 0.25);
                robot.frontRight.setPower(vArray[1] * 0.25);
                robot.backLeft.setPower(vArray[2] * 0.25);
                robot.backRight.setPower(vArray[3] * 0.25);
            }
            else{
                robot.frontLeft.setPower(vArray[0]);
                robot.frontRight.setPower(vArray[1]);
                robot.backLeft.setPower(vArray[2]);
                robot.backRight.setPower(vArray[3]);
            }

            /*
            double v1 = Math.cos(robotAngle);
            double v2 = Math.sin(robotAngle);
            double v3 = Math.sin(robotAngle);
            double v4 = Math.cos(robotAngle);

            double vMax=0;
            double[] vArray={v1,v2,v3,v4};
            for(double voltage: vArray){
                if(Math.abs(voltage)>vMax){
                    vMax=Math.abs(voltage);
                }
            }
            v1=v1/vMax;
            v2=v2/vMax;
            v3=v3/vMax;
            v4=v4/vMax;

            //voltages implemented with magnitude(joystick) and rotational Power
            v1 = r * v1 + rotationalPower;
            v2 = r * v2 - rotationalPower;
            v3 = r * v3 + rotationalPower;
            v4 = r * v4 - rotationalPower;

            robot.frontLeft.setPower(v1);
            robot.frontRight.setPower(v2);
            robot.backLeft.setPower(v3);
            robot.backRight.setPower(v4);
            */

/*
            telemetry.update();
            telemetry.addData("leftstick_y",gamepad2.left_stick_y);
            telemetry.addData("rightstick_y", gamepad2.right_stick_y);
            telemetry.addData("leftstick_x",gamepad2.left_stick_x);
            telemetry.addData("rightstick_x", gamepad2.right_stick_x);
            telemetry.addData("PfrontLeft:", robot.frontLeft.getPower());
            telemetry.addData("PfrontRight:", robot.frontRight.getPower());
            telemetry.addData("PbackLeft:", robot.backLeft.getPower());
            telemetry.addData("PbackRight:", robot.backRight.getPower());
            */


         //Gamepad 2 Robot Controller

            //lift with limit
            if(gamepad2.y && !robot.topLimit.isPressed()){
                //while y button is pressed and top limit is not pressed, extend lift
                robot.lift.setPower(-1);
            }
            else if(gamepad2.x && !robot.topLimit.isPressed()){
                robot.lift.setPower(1);
            }
            else if(gamepad2.x==false && gamepad2.y==false){
                robot.lift.setPower(gamepad2.right_stick_y);
            }
            else{
                robot.lift.setPower(0);
            }

            //intake arm
            //joystick down
            if(gamepad2.left_stick_y>1.0){
                robot.intakeArm.setPower(0.0);
            }else{
                robot.intakeArm.setPower(gamepad2.left_stick_y*0.5);
            }

            //intake servo
            if (gamepad2.a == true) {
                //intake.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.intake.setPower(1.0);
            } else if (gamepad2.b == true) {
                robot.intake.setPower(-1.0);
                // intake.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                robot.intake.setPower(0.0);
            }

            //marker dispenser
            robot.markerDispenser.setPosition(1);

            //wait for hardware to catch up
            idle();
        }

    }
    public void EncoderDetractLift(double power, int distance){

        //Restart Encoders
        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position

        robot.lift.setTargetPosition(robot.lift.getCurrentPosition()+distance);

        //set RUN_TO_POSITION mode
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lift.setPower(power);

        while(robot.lift.isBusy()){
            //wait until target position is reached
            telemetry.update();
            telemetry.addData("Lift Encoder Position", robot.lift.getCurrentPosition());
            telemetry.addData("get Target Position", robot.lift.getTargetPosition());

        }
        robot.lift.setPower(0.0);
        sleep(300);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    int positiveNegative(double x){
        int result=1;
        if(x<0){
            result=-1;
        }
        return result;
    }
}

