package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.Locale;




import org.firstinspires.ftc.teamcode.hMap;

/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {

    //Declare Gyro
    double heading;
    Orientation angles;
    int run360=5476;


    hMap robot = new hMap();


    ColorSensor color_sensor;







    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        color_sensor = hardwareMap.colorSensor.get("color");
        waitForStart();

        //Start Auto
        gyroToGo(90);


    }













    //Driving Power Functions
    public void DriveForward(double power){
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
    }
    public void DriveBackward(double power){
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(-power);
    }
    public void rotateLeft(double power){
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(-power);
    }
    public void rotateRight(double power){
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(power);
    }
    public void StopDriving(){
        DriveForward(0.0);
    }

    //Encoder Functions
    public void DriveForwardDistance(double power, int distance){

        //Restart Encoders
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position
        robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition()+ distance);
        robot.motorRight.setTargetPosition(robot.motorRight.getCurrentPosition() + distance);

        //set RUN_TO_POSITION mode
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(power);

        while(robot.motorLeft.isBusy() && robot.motorRight.isBusy()){

            //wait until target position is reached

        }
        StopDriving();
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void RotateDistance(double power, int distance) throws InterruptedException {
        {
            //reset encoders
            robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorLeft.setTargetPosition(distance);
            robot.motorRight.setTargetPosition(-distance);


            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rotateRight(power);

            while (robot.motorLeft.isBusy() && robot.motorRight.isBusy()) {
                //wait until robot stops
            }

            StopDriving();
        }
    }

    //Timed Drive

    public void DriveForwardTime(double power, long time) throws InterruptedException{
        DriveForward(power);
        Thread.sleep(time);
        StopDriving();

    }
    public void DriveBackwardTime(double power, long time) throws InterruptedException{
        DriveBackward(power);
        Thread.sleep(time);
        StopDriving();

    }
    public void TurnRightTime(double power, long time) throws InterruptedException{
        rotateRight(power);
        Thread.sleep(time);
        StopDriving();

    }
    public void TurnLeftTime(double power, long time) throws InterruptedException{
        rotateLeft(power);
        Thread.sleep(time);
        StopDriving();

    }













    //Gyro
    public void waitUntilStable() throws InterruptedException {
        telemetry.update();
        double degree = heading;
        double previousreading = 0;
        boolean stable = false;
        while (stable == false) {
            sleep(10);
            previousreading = heading;
            if (Math.abs(degree - previousreading) < 0.1) {
                stable = true;
            }
        }
    }
    static class RangeResult {
        public double distance;
        public int position;
    }

    RangeResult inRange(double angle, double offset) {
        RangeResult range = new RangeResult();
        telemetry.update();
        double degree = heading;
        range.distance = Math.abs(angle - degree);
        double right = angle - offset;
        double left = angle + offset;
        if (right < 0) {
            right = right + 360;
            if (degree > right || degree < left) {
                range.position = 0;
            } else {
                range.position = 1;
            }
        } else if (left >= 360) {
            left = left - 360;
            if (degree < left || degree > right) {
                range.position = 0;
            } else {
                range.position = -1;
            }
        } else {
            if (degree > left) {
                range.position = 1;
            } else if (degree < right) {
                range.position = -1;
            } else {

                range.position = 0;
            }
        }
        if (range.distance > 180) {
            range.distance = range.distance - 180;
            if (range.position == -1) {
                range.position = 1;
            } else {
                range.position = -1;
            }
        }
        return range;
    }

    //turn left when -1
    //turn right when 1
    public void gyroToGo(double angle) throws InterruptedException {
        double angleoffset = 4;
        RangeResult rangeresult = inRange(angle, angleoffset);
        int position = rangeresult.position;
        int previousposition = rangeresult.position;
        double distance = rangeresult.distance;
        double previouspower = 0.5;
        double powerlevel = 0.5;
        double k=0.7;
        while (true) {
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //adjust power level
            if (distance > 40) {
                powerlevel = 0.7;
            }
            else{
                powerlevel = k-3;
            }

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    break;
                }
                //position is left of heading, rotate right
            } else if (position == 1) {
                if (previouspower != powerlevel || previousposition != position) {
                    int deg= Math.round((float) (run360/360)*(float)(distance));
                    RotateDistance(powerlevel, deg);
                    previousposition = position;
                    previouspower = powerlevel;
                }
                //position is right of heading, rotate left
            } else if (position == -1) {
                if (previouspower != powerlevel || previousposition != position) {
                    int deg= Math.round((float) (run360/360)*(float)(distance));
                    RotateDistance(-powerlevel, -deg);
                    previousposition = position;
                    previouspower = powerlevel;

                }
            }
        }
    }

    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //robot.gravity = robot.imu.getGravity();
            }
        });

        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });
                */

        telemetry.addLine()
                //rotating left adds to the heading, while rotating right makes the heading go down.
                //when heading reaches 180 it'll become negative and start going down.

                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {

                        //heading is a string, so the below code makes it a long so it can actually be used
                        heading = Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
                        if (heading < 0) {
                            heading = heading + 360;
                        }

                        return formatAngle(robot.angles.angleUnit, heading);

                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    //The two functions below are for gyro
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    void setHeadingToZero() {
        robot.gyroInit();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void colorSense(){
        DriveForward(power);
        if(color_sensor.red()>20){ 
            StopDriving();
        }
    }

}
