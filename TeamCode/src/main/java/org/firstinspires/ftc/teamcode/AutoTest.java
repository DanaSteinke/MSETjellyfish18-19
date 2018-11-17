package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.Locale;
import java.util.Vector;


import org.firstinspires.ftc.teamcode.hMap;

    /**
     * Created by Feranno on 9/29/18.
     */
    @Autonomous(name = "AutoTest")
    public class AutoTest extends LinearOpMode {

        //Declare Gyro
        double heading;
        public Orientation angles;
        public Acceleration gravity;
        final int run360=6962;

        // The IMU sensor object
        public BNO055IMU imu;
        int rev=1120;
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;



        //ColorSensor color_sensor;



        @Override
        public void runOpMode() throws InterruptedException {
            composeTelemetry();
            gyroInit();
            setHeadingToZero();

            //color_sensor = hardwareMap.colorSensor.get("color");
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareMap.dcMotor.get("backRight");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);


            waitForStart();
            //Start Auto

/*
            while(tapeColor().equals("red")==false){
                telemetry.update();
                String.format("red: ", color_sensor.red());
                String.format("green: ", color_sensor.green());
                String.format("blue: ", color_sensor.blue());
                DriveForward(0.3);
            }
            StopDriving();
            sleep(300);
            */

           VectorDistance(0, run360, 0, 1);
           sleep(300);
           VectorDistance(0,run360,0,0.5);
           sleep(300);
           VectorDistance(0,run360/2,0,-1);
           sleep(300);
           VectorDistance(0.5,run360/2,0,-0.7);

           sleep(300);
           VectorDistance(0.7,1500,90,0);
           sleep(300);
           VectorDistance(0.7,1500,180,0);
           sleep(300);
           VectorDistance(0.7,1500,270,0);
           sleep(300);
           VectorDistance(0.7,1500,0,0);



        }
        //Driving Power Functions
        public void DriveForward(double power){
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
        public void DriveBackward(double power){
            DriveForward(-power);
        }
        public void rotateLeft(double power){
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
            backLeft.setPower(power);
        }
        public void rotateRight(double power){
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(-power);
        }
        public void StopDriving(){
            DriveForward(0.0);
        }

        //Encoder Functions
        public void DriveVeriticalDistance(double power, int distance){

            //Restart Encoders
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Start Target Position
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+ distance);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + distance);
            frontRight.setTargetPosition(backLeft.getCurrentPosition() + distance);
            backRight.setTargetPosition(backLeft.getCurrentPosition() + distance);

            //set RUN_TO_POSITION mode
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward(power);

            while(frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()){
                //wait until target position is reached
                telemetry.update();
                telemetry.addData("frontLeft:",frontLeft.getPower());
                telemetry.addData("frontRight:",frontLeft.getPower());
                telemetry.addData("backLeft:",frontLeft.getPower());
                telemetry.addData("backRight:",frontLeft.getPower());

            }

            StopDriving();
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int positiveNegative(double x){
            int result=1;
            if(x<0){
                result=-1;
            }
            return result;
        }

        //if rotationalPower, insert power:0 and directionalAngle:0
        //power and rotational power between -1 and 1
        void VectorDistance(double power, int distance, double directionalAngle, double rotationalPower){
            double r = power;
            double robotAngle = Math.toRadians(directionalAngle) - Math.PI / 4;
            double rotateAngle = rotationalPower;
            //calculate voltage for each motor
            double v1 = r * Math.cos(robotAngle)+rotateAngle;
            double v2 = r * Math.sin(robotAngle)-rotateAngle;
            double v3 = r * Math.sin(robotAngle)+rotateAngle;
            double v4 = r * Math.cos(robotAngle)-rotateAngle;

            //calculate distance for each motor
            double vMax=0;
            double[] vArray={v1,v2,v3,v4};
            for(double voltage: vArray){
                if(voltage>vMax){
                    vMax=voltage;
                }
            }
            //unique motor distance= percentage(+/-) * distance
            /*
                int d1 = (int) (v1 / vMax) * distance;
                int d2 = (int) (v2 / vMax) * distance;
                int d3 = (int) (v3 / vMax) * distance;
                int d4 = (int) (v4 / vMax) * distance;
                */


            //reset encoders
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //check that motor(frontLeft,backLeft,frontRight,backRight) corresponds with voltage and unique motor distance
                //MOTOR ORDER MATTERS
                //ex: frontLeft=>v1 and d1
                frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (distance * positiveNegative(v1)));
                frontRight.setTargetPosition(frontRight.getCurrentPosition() + (distance * positiveNegative(v2)));
                backLeft.setTargetPosition(backLeft.getCurrentPosition() + (distance * positiveNegative(v3)));
                backRight.setTargetPosition(backRight.getCurrentPosition() + (distance * positiveNegative(v4)));


            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                //wait until robot stops
                telemetry.update();
                telemetry.addData("v1",v1);
                telemetry.addData("v2",v2);
                telemetry.addData("v3",v3);
                telemetry.addData("v4",v4);
/*
                telemetry.addData("d1",d1);
                telemetry.addData("d2",d2);
                telemetry.addData("d3",d3);
                telemetry.addData("d4",d4);
                */

                telemetry.addData("frontLeft:", frontLeft.getPower());
                telemetry.addData("frontRight:",frontRight.getPower());
                telemetry.addData("backLeft:", backLeft.getPower());
                telemetry.addData("backRight:",backRight.getPower());

            }

            StopDriving();
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        void RotateDistance(double power, int distance) throws InterruptedException {

            //reset encoders
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+distance);
            backLeft.setTargetPosition(backLeft.getCurrentPosition()+distance);
            frontRight.setTargetPosition(frontRight.getCurrentPosition()-distance);
            backRight.setTargetPosition(backRight.getCurrentPosition()-distance);



            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rotateRight(power);

            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                //wait until robot stops
            }

            StopDriving();
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        //position 0= in range
        //position 1= degree is greater than left
        //position -1= degree is less than right
        RangeResult inRange(double angle, double offset) {
           RangeResult range = new RangeResult();
            telemetry.update();
            double degree = heading;
            //find distance from angle to degree
            range.distance = Math.abs(angle - degree);
            double right = angle - offset;
            double left = angle + offset;
            //if right is less than 0 degrees; out of bounds
            if (right < 0) {
                right = right + 360;
                if (degree > right || degree < left) {
                    range.position = 0;
                } else {
                    range.position = 1;
                }
                //if left is greater than 360 degrees; out of bounds
            } else if (left >= 360) {
                left = left - 360;
                if (degree < left || degree > right) {
                    range.position = 0;
                } else {
                    range.position = -1;
                }
                //normal conditions: if degree is greater than left, set position to 1
                //else, if degree is less than right, set position to -1
                //else, if not greater than left nor less than right, in range, set position to 0

            } else {
                if (degree > left) {
                    range.position = 1;
                } else if (degree < right) {
                    range.position = -1;
                } else {

                    range.position = 0;
                }
            }
            //find shortest distance: if greater than 180 degrees, change direction and subtract distance by 180
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
            double powerlevel =0.5;
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
                    powerlevel = 0.5;
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
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
                            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                            if (heading < 0) {
                                heading = heading + 360;
                            }

                            return formatAngle(angles.angleUnit, heading);

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
            gyroInit();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }

        public void gyroInit() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
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

