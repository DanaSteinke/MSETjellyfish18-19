package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hMap;

/**
 * Created by FerannoDad on 9/29/18.
 */
@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {
    hMap robot = new hMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        //Start Auto
        DriveForwardTime(0.7, 1500);
        sleep(300);
        DriveBackwardTime(0.7, 1500);
        sleep(300);
        TurnLeftTime(0.5,3000);
        sleep(300);
        TurnRightTime(0.5, 3000);



    }

    //Basic Drives
    public void DriveForward(double power){
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
    }
    public void DriveBackward(double power){
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(-power);
    }
    public void TurnLeft(double power){
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(-power);
    }
    public void TurnRight(double power){
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(power);
    }
    public void StopDriving(){
        DriveForward(0.0);
    }

    //Encoders

    //Timed Drive

    //Drive Forward Time
    public void DriveForwardTime(double power, long time) throws InterruptedException{
        DriveForward(power);
        Thread.sleep(time);
        StopDriving();

    }
    //Drive Backward Time
    public void DriveBackwardTime(double power, long time) throws InterruptedException{
        DriveBackward(power);
        Thread.sleep(time);
        StopDriving();

    }
    //Turn Right Time
    public void TurnRightTime(double power, long time) throws InterruptedException{
        TurnRight(power);
        Thread.sleep(time);
        StopDriving();

    }
    //Turn Left Time
    public void TurnLeftTime(double power, long time) throws InterruptedException{
        TurnLeft(power);
        Thread.sleep(time);
        StopDriving();

    }
}
