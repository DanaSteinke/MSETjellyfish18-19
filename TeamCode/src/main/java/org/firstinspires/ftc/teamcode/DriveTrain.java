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
            //tank drive
            //field centric drive
            //because of joystick positions, change yValue to reverse, then Quadrants will be in position
            double yValue=-gamepad1.left_stick_y;
            double xValue=gamepad1.left_stick_x;
            double r = Math.hypot(xValue, yValue);
            double robotAngle = Math.atan2(yValue, xValue) - Math.PI / 4;
            double rotationalPower = gamepad1.right_stick_x;

            //calculate direction of voltages to max value with sine cosine functions
            //voltages are without magnitude(joystick) nor rotational Power yet
            double v1 = 2*r * Math.cos(robotAngle) + rotationalPower;
            double v2 = 2*r *  Math.sin(robotAngle) - rotationalPower;
            double v3 = 2*r *  Math.sin(robotAngle) + rotationalPower;
            double v4 = 2*r * Math.cos(robotAngle) - rotationalPower;
            double[] vArray={v1,v2,v3,v4};
            for(int i=0; i<vArray.length; i++){
                if(Math.abs(vArray[i])>1) {
                    vArray[i]=positiveNegative(vArray[i]);
                }
            }
            robot.frontLeft.setPower(vArray[0]);
            robot.frontRight.setPower(vArray[1]);
            robot.backLeft.setPower(vArray[2]);
            robot.backRight.setPower(vArray[3]);

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
*/

            robot.frontLeft.setPower(v1);
            robot.frontRight.setPower(v2);
            robot.backLeft.setPower(v3);
            robot.backRight.setPower(v4);


            telemetry.update();
            telemetry.addData("leftstick_y",gamepad2.left_stick_y);
            telemetry.addData("rightstick_y", gamepad2.right_stick_y);
            telemetry.addData("leftstick_x",gamepad2.left_stick_x);
            telemetry.addData("rightstick_x", gamepad2.right_stick_x);
            telemetry.addData("PfrontLeft:", robot.frontLeft.getPower());
            telemetry.addData("PfrontRight:", robot.frontRight.getPower());
            telemetry.addData("PbackLeft:", robot.backLeft.getPower());
            telemetry.addData("PbackRight:", robot.backRight.getPower());

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
            //robot.lift.setPower(gamepad2.left_stick_y);
            //marker disposer

             yValue=-gamepad2.left_stick_y;
             xValue=gamepad2.left_stick_x;
             r = Math.hypot(xValue, yValue);
             robotAngle = Math.atan2(yValue, xValue) - Math.PI / 4;
             rotationalPower = gamepad1.right_stick_x;

            //voltages implemented with magnitude(joystick) and rotational Power
            v1 = r * Math.cos(robotAngle) + rotationalPower;
            v2 = r *  Math.sin(robotAngle) - rotationalPower;
            v3 = r *  Math.sin(robotAngle) + rotationalPower;
            v4 = r * Math.cos(robotAngle) - rotationalPower;


            robot.frontLeft.setPower(v1);
            robot.frontRight.setPower(v2);
            robot.backLeft.setPower(v3);
            robot.backRight.setPower(v4);
            robot.markerDispenser.setPosition(gamepad2.right_stick_y);
            telemetry.addData("markerPosition:",robot.markerDispenser.getPosition());



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
    int positiveNegative(double x){
        int result=1;
        if(x<0){
            result=-1;
        }
        return result;
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

