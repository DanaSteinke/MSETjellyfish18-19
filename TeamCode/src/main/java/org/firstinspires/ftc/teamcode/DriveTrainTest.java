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
    private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //initialize Servos
        intake = hardwareMap.crservo.get("intake");

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
            //tankDrive
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //intkae servo
            if (gamepad1.a) {
                intake.setPower(0.7);
            }
            if (gamepad1.y) {
                intake.setPower(0.5);
            }



            //wait for hardware to catch up
            idle();
        }

    }
}
