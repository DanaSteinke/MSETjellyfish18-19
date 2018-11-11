package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by FerannoDad on 9/23/18.
 */

@TeleOp(name = "DriveTrainTest", group = "TeleOp")
public class DriveTrainTest extends LinearOpMode {
    //Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Declare Servos
    /*
    private CRServo intake;
    private ColorSensor color_sensor;
    */

    //private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        //initialize Servos
        /*
        intake = hardwareMap.crservo.get("intake");
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        */


        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
            telemetry.update();
         //Gamepad 1 DriveTrain Controller
            //drivetrain motors
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



         //Gamepad 2 Robot Controller

            //intake
            /*
            if (gamepad2.a == true) {
                //intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1.0);
            } else if (gamepad2.b == true) {
                intake.setPower(-1.0);
                // intake.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                intake.setPower(gamepad2.right_stick_y);
                telemetry.addData("intake power", gamepad2.right_stick_y);
            }
            */


            //color sensor
            /*
            telemetry.addData("Red", color_sensor.red());
            telemetry.addData("Green", color_sensor.green());
            telemetry.addData("Blue", color_sensor.blue());
            telemetry.addData("Color", tapeColor());
            */
        }

    }

/*
    public String tapeColor() {
        if(color_sensor.red()>200 ){
            return "red";
        }
        else if(color_sensor.blue()>200){
            return "blue";
        }
        else{
            return "no color";
        }

    }
   */
}

