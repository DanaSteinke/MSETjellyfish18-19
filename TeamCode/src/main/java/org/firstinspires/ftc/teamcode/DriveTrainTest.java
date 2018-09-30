package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FerannoDad on 9/23/18.
 */

@TeleOp(name = "DriveTrainTest", group = "TeleOp")
public class DriveTrainTest extends LinearOpMode {
    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //press start button
        waitForStart();
        //while running, before pressing stop button
        while (opModeIsActive()) {
            //tankDrive
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //wait for hardware to catch up
            idle();
        }

    }
}
