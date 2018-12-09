package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode{
    //public Robot robot;
    public CRServo intake;
    HardwareMap hardwareMap;

    teleop(){

    }
    public void runOpMode() throws InterruptedException{
        //robot = new Robot(this);
        init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            intake.setPower(-gamepad2.right_stick_y);
            //Gamepad 1 Drive Train Controller
            //field centric drive
            //because of joystick positions, change yValue to reverse, then Quadrants will be in position
            //for Jae purposes . . . not right now
            /*
            double yValue=-gamepad1.left_stick_y;
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
                robot.driveBase.frontLeft.setPower(v1 * 0.35);
                robot.driveBase.frontRight.setPower(v2 * 0.35);
                robot.driveBase.backLeft.setPower(v3 * 0.35);
                robot.driveBase.backRight.setPower(v4 * 0.35);
            }
            else{
                robot.driveBase.frontLeft.setPower(v1);
                robot.driveBase.frontRight.setPower(v2);
                robot.driveBase.backLeft.setPower(v3);
                robot.driveBase.backRight.setPower(v4);
            }
            */

        }
    }
    public void init(HardwareMap hwMap){
        this.hardwareMap = hwMap;
        intake = this.hardwareMap.crservo.get("intake");
    }
}
