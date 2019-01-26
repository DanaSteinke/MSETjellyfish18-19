package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import android.util.Log;


import java.util.Locale;

public class DriveBase {
    LinearOpMode opMode;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //gyro
    double heading;
    long startTime = System.nanoTime();
    long timeElapse;

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;


    public DriveBase(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;

        //initialize motors
        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }


    //Driving Power Functions
    public void driveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void driveBackward(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
        backLeft.setPower(-power);
    }

    public void rotateLeft(double power) {

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

    }

    public void rotateRight(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void StopDriving() {
        driveForward(0.0);
    }

    //Encoder Functions
    public void rotateRightDistance(double power, int distance) {

        //Restart Encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - distance);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - distance);
        frontRight.setTargetPosition(backLeft.getCurrentPosition() + distance);
        backRight.setTargetPosition(backLeft.getCurrentPosition() + distance);

        //set RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotateRight(power);

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            //wait until target position is reached
        }
        StopDriving();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    int positiveNegative(double x) {
        int result = 1;
        if (x < 0) {
            result = -1;
        }
        return result;
    }

    public void compassDrive(double power, double angle) {
        RangeResult rangeresult = new RangeResult();
        rangeresult = inRange(angle,5);

        //convert degrees(0-360) to radians(0.0-3.14) to diff(0.0 to 0.1)
        double diff = (Math.toRadians(rangeresult.distance)/3.14)/10;
        double position = rangeresult.position;
        diff = diff * position;

        double pfl = scaleDiffPower(power, diff, 1.0);
        double pbl = scaleDiffPower(power, diff, 1.0);
        double pfr = scaleDiffPower(power, diff, -1.0);
        double pbr = scaleDiffPower(power, diff, -1.0);
        safeDrive(pfl, pbl, pfr, pbr);
    }

    public void safeDrive(double pfl, double pbl, double pfr, double pbr) {
        // Stagger the order to reduce left/right front/back bias at start
        frontLeft.setPower(safePower(pfl));
        backRight.setPower(safePower(pbr));
        backLeft.setPower(safePower(pbl));
        frontRight.setPower(safePower(pfr));
    }

    public double safePower(double power) {
        if (power < -1.0) {
            power = -1.0;
        }
        if (power > 1.0) {
            power = 1.0;
        }
        return power;
    }

    public double scaleDiffPower(double power, double diff, double sign) {
        double p = power + (sign * diff);
        return safePower(p);
    }


    //if rotationalPower, insert power:0 and directionalAngle:0
    //power and rotational power between -1 and 1
    void VectorDistance(double power, int distance, double directionalAngle) throws InterruptedException {
        //match gyro direction
        directionalAngle += 90;
        //multiplier
        double m = 1;
        double r = power;
        double robotAngle = Math.toRadians(directionalAngle) - Math.PI / 4;
        //calculate voltage for each motor
        double v1 = m * r * Math.cos(robotAngle);
        double v2 = m * r * Math.sin(robotAngle);
        double v3 = m * r * Math.sin(robotAngle);
        double v4 = m * r * Math.cos(robotAngle);
        //safe power

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


        //correcting for Vector Distance with gyro
        //RangeResult rangeResult10 = inRange(directionalAngle, 10);
        //int v1distance, v2distance, v3distance, v4distance;

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //wait until robot stops


            opMode.telemetry.addData("v1", v1);
            opMode.telemetry.addData("v2", v2);
            opMode.telemetry.addData("v3", v3);
            opMode.telemetry.addData("v4", v4);

            opMode.telemetry.addData("frontLeftP:", frontLeft.getPower());
            opMode.telemetry.addData("frontRightP:", frontRight.getPower());
            opMode.telemetry.addData("backLeftP:", backLeft.getPower());
            opMode.telemetry.addData("backRightP:", backRight.getPower());

            opMode.telemetry.addData("frontLeftE:", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("frontRightE:", frontRight.getCurrentPosition());
            opMode.telemetry.addData("backLeftE:", backLeft.getCurrentPosition());
            opMode.telemetry.addData("backRightE:", backRight.getCurrentPosition());

        }

        StopDriving();
        opMode.sleep(300);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //GYRO
    public void waitUntilStable() throws InterruptedException {
        opMode.telemetry.update();
        double degree = heading;
        double previousreading = 0;
        boolean stable = false;
        while (stable == false) {
            opMode.sleep(10);
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

    public double getDegree() {
        opMode.telemetry.update();
        double degree = heading;
        if (degree >= 180 || degree <= -180) {
            degree = 0;
        }
        if (degree < 0) {
            degree = degree + 360;
        }
        if (degree == 360) {
            degree = 0;
        }
        return degree;
    }


    public RangeResult inRange(double angle, double offset) {
        if (angle == 360) {
            angle = 0;
        }
        opMode.telemetry.addData("ExecutionTimeinMilliseconds", timeElapse / 1000000);
        RangeResult range = new RangeResult();
        double degree = getDegree();


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
                if(range.distance > 180) {
                    range.distance = (360 - degree) + angle;
                    range.position = -1;
                }
            }
            //if left is greater than 360 degrees; out of bounds
        } else if (left > 360) {
            left = left - 360;
            if (degree < left || degree > right) {
                range.position = 0;
            } else {
                range.position = -1;
                if(range.distance > 180) {
                    range.distance = degree + (360 - angle);
                    range.position = 1;
                }
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

        opMode.telemetry.addData("degree", degree);
        opMode.telemetry.addData("angle", angle);
        opMode.telemetry.addData("range.distance = ", range.distance);

        return range;
    }

    /*
     * @param position -1, turn left
     * @param position 1, turn right
     */
    public void gyroToGo(double angle) throws InterruptedException {
        gyroToGo(angle, 5);
    }

    public void gyroToGo(double angle, double offset) throws InterruptedException {
        double angleoffset = offset;
        RangeResult rangeresult = inRange(angle, angleoffset);
        double distance = rangeresult.distance;
        int position = rangeresult.position;
        int previousposition = -10;
        double previouspower = 0.5;
        double powerlevel = 0.5;
        boolean forceLowPower = false;

        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("power: ", powerlevel);
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //change power to 0.5 first time
            if (distance > 45) {
                powerlevel = 0.5;
            } else {
                powerlevel = 0.25;
                forceLowPower = true;
            }

            if (forceLowPower) {
                powerlevel = 0.25;
            }

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    opMode.sleep(300);
                    break;
                }
                //position is left of heading, rotate right
            } else if (position == 1) {
                if ((previouspower != powerlevel || previousposition != position)) {
                    rotateRight(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                }
                //position is right of heading, rotate left
            } else if (position == -1) {
                if (previouspower != powerlevel || previousposition != position) {
                    rotateLeft(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                }
            }

        }
    }


    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        opMode.telemetry.addAction(new Runnable() {
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

        opMode.telemetry.addLine()
                //rotating left adds to the heading, while rotating right makes the heading go down.
                //when heading reaches 180 it'll become negative and start going down.

                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {

                        //heading is a string, so the below code makes it a long so it can actually be used
                        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

                        long endTime = System.nanoTime();
                        timeElapse = endTime - startTime;
                        startTime = endTime;
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


    void setHeadingToZero() throws InterruptedException {
        gyroInit();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void gyroInit() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //The REV Expansion hub is mounted vertically, so we have to flip the y and z axes.
        byte AXIS_MAP_CONFIG_BYTE = 0x18;
        byte AXIS_MAP_SIGN_BYTE = 0x1;

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        /*
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
        */
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        /*
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
        */
    }
}
