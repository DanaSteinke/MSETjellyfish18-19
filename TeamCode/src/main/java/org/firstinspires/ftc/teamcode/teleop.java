package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {
    public Robot robot;
    public CRServo intake;
    HardwareMap hwMap;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        //init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

         //GAMEPAD 1
            //Gamepad 1 Drive Train Controller
            //field centric drive
            //because of joystick positions, change yValue to reverse, then Quadrants will be in position
            //for Jae purposes . . . not right now

            double yValue = -gamepad1.left_stick_y;
            double xValue = gamepad1.left_stick_x;
            double r = Math.hypot(xValue, yValue);
            double robotAngle = Math.atan2(yValue, xValue) - Math.PI / 4;
            double rotationalPower = -gamepad1.right_stick_x;
            //multiplier
            double m = 1.41;

            //calculate direction of voltages to max value with sine cosine functions
            //voltages are without magnitude(joystick) nor rotational Power yet
            double v1 = m * r * Math.cos(robotAngle) + rotationalPower;
            double v2 = m * r * Math.sin(robotAngle) - rotationalPower;
            double v3 = m * r * Math.sin(robotAngle) + rotationalPower;
            double v4 = m * r * Math.cos(robotAngle) - rotationalPower;


            if (gamepad1.right_bumper == true) {
                robot.driveBase.frontLeft.setPower(v1 * 0.5);
                robot.driveBase.frontRight.setPower(v2 * 0.5);
                robot.driveBase.backLeft.setPower(v3 * 0.5);
                robot.driveBase.backRight.setPower(v4 * 0.5);
            } else {
                robot.driveBase.frontLeft.setPower(v1);
                robot.driveBase.frontRight.setPower(v2);
                robot.driveBase.backLeft.setPower(v3);
                robot.driveBase.backRight.setPower(v4);
            }
            /*
            telemetry.update();
            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);
            telemetry.addData("v4", v4);

            telemetry.addData("frontLeft: ", robot.driveBase.frontLeft.getPower());
            telemetry.addData("backLeft: ", robot.driveBase.backLeft.getPower());
            telemetry.addData("frontRight: ", robot.driveBase.frontRight.getPower());
            telemetry.addData("backRight: ", robot.driveBase.backRight);
            */

         //GAMEPAD 2
            //lift with limits
            if (gamepad2.y && !robot.Lift.topLimit.isPressed()) {
                //while y button is pressed and top limit is not pressed, extend lift
                robot.Lift.lift.setPower(1);
            } else if (gamepad2.x && !robot.Lift.botLimit.isPressed()) {
                robot.Lift.lift.setPower(-1);
            } else {
                robot.Lift.lift.setPower(0);
                robot.Lift.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //intake system
            robot.intakePivot.setPower(gamepad2.left_stick_y * 0.75);
            if(robot.intakePivot.getPower() == 0.0){
                robot.intakePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //intake
            if (gamepad2.a == true) {
                robot.intake.setPower(1);
            } else if (gamepad2.b == true) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0.0);
            }

            //elevator
            if (gamepad2.dpad_left == true) {
                robot.elevator.setPower(0.8);
            } else if (gamepad2.dpad_right) {
                robot.elevator.setPower(-1);
            } else {
                robot.elevator.setPower(0.0);
                robot.Lift.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            //outake
            robot.outake.setPosition(gamepad2.right_trigger);

            telemetry.addData("outake", robot.outake.getPosition());
            telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.update();

         //Stationary
            // marker dispenser
            robot.markerDispenser.setPosition(0.55);

        }
    }
    /*
    public void init(HardwareMap ahwMap){
        this.hwMap = ahwMap;
        //intake = this.hwMap.crservo.get("intake");
    }
    */
}
