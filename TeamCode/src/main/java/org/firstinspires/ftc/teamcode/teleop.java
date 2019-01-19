package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode{
    public Robot robot;
    public CRServo intake;
    HardwareMap hwMap;

    public void runOpMode() throws InterruptedException{
        robot = new Robot(this);
        //init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){

         //GAMEPAD 1
            //intake.setPower(-gamepad2.right_stick_y);
            //Gamepad 1 Drive Train Controller
            //field centric drive
            //because of joystick positions, change yValue to reverse, then Quadrants will be in position
            //for Jae purposes . . . not right now

            double yValue=-gamepad1.right_stick_y;
            double xValue=gamepad1.right_stick_x;
            double r = Math.hypot(xValue, yValue);
            double robotAngle = Math.atan2(yValue, xValue) - Math.PI / 4;
            double rotationalPower = gamepad1.left_stick_x;

            //calculate direction of voltages to max value with sine cosine functions
            //voltages are without magnitude(joystick) nor rotational Power yet
            double v1 = r * Math.cos(robotAngle) + rotationalPower;
            double v2 = r *  Math.sin(robotAngle) - rotationalPower;
            double v3 = r *  Math.sin(robotAngle) + rotationalPower;
            double v4 = r * Math.cos(robotAngle) - rotationalPower;

            if(gamepad1.right_bumper==true) {
                robot.driveBase.frontLeft.setPower(v1 * 0.4);
                robot.driveBase.frontRight.setPower(v2 * 0.4);
                robot.driveBase.backLeft.setPower(v3 * 0.4);
                robot.driveBase.backRight.setPower(v4 * 0.4);
            }
            else{
                robot.driveBase.frontLeft.setPower(v1);
                robot.driveBase.frontRight.setPower(v2);
                robot.driveBase.backLeft.setPower(v3);
                robot.driveBase.backRight.setPower(v4);
            }

         //GAMEPAD 2
            //lift with limit
            if(gamepad2.y && !robot.Lift.topLimit.isPressed()){
                //while y button is pressed and top limit is not pressed, extend lift
                robot.Lift.lift.setPower(-1);
            }
            else if(gamepad2.x && !robot.Lift.topLimit.isPressed()){
                robot.Lift.lift.setPower(1);
            }
            else if(gamepad2.x==false || gamepad2.y==false){
                if(gamepad2.dpad_down == true) {
                    robot.Lift.lift.setPower(-1);
                } else if(gamepad2.dpad_up == true){
                    robot.Lift.lift.setPower(1);
                } else {
                    robot.Lift.lift.setPower(0);
                    robot.Lift.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }


            //intake pivot
            robot.intakePivot.setPower(gamepad2.right_stick_y*0.85);

            //intake
            if(gamepad2.a == true){
                robot.intake.setPower(1);
            } else if(gamepad2.b == true){
                robot.intake.setPower(-1);
            } else{
                robot.intake.setPower(0.0);
            }

            //elevator
            if(gamepad2.dpad_left == true){
                robot.elevator.setPower(0.8);
            } else if(gamepad2.dpad_right){
                robot.elevator.setPower(-0.4);
            } else{
                robot.elevator.setPower(0.0);
                robot.Lift.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
         //Stationary
            // marker dispenser
            robot.markerDispenser.setPosition(0.55);
            telemetry.update();
            telemetry.addData("liftCurrentPosition", robot.Lift.lift.getCurrentPosition());
            telemetry.addData("elevatorPower", robot.elevator.getPower());
            telemetry.addData("gamepad2.x", gamepad2.x);
            telemetry.addData("gamepad2.y", gamepad2.y);




        }
    }
    /*
    public void init(HardwareMap ahwMap){
        this.hwMap = ahwMap;
        //intake = this.hwMap.crservo.get("intake");
    }
    */
}
