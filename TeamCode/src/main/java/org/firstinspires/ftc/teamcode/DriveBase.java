package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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


    public void backLeftDistance(double power, int distance) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + distance);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(power);
        while (backLeft.isBusy()) {

        }
        backLeft.setPower(0);
        opMode.sleep(300);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void backRightDistance(double power) {
        backRight.setPower(power);
    }

    //Driving Power Functions
    public void DriveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void DriveBackward(double power) {
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
        DriveForward(0.0);
    }

    //Encoder Functions
    public void DriveForwardDistance(double power, int distance) {

        //Restart Encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start Target Position
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + distance);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + distance);
        frontRight.setTargetPosition(backLeft.getCurrentPosition() + distance);
        backRight.setTargetPosition(backLeft.getCurrentPosition() + distance);

        //set RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(power);

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


    //if rotationalPower, insert power:0 and directionalAngle:0
    //power and rotational power between -1 and 1
    void VectorDistance(double power, int distance, double directionalAngle) throws InterruptedException {
        //match gyro direction
        directionalAngle += 90;

        double r = power;
        double robotAngle = Math.toRadians(directionalAngle) - Math.PI / 4;
        //calculate voltage for each motor
        double v1 = r * Math.cos(robotAngle);
        double v2 = r * Math.sin(robotAngle);
        double v3 = r * Math.sin(robotAngle);
        double v4 = r * Math.cos(robotAngle);

        //calculate max power for each motor
        /*
        double[] vArray={v1,v2,v3,v4};
        for(int i=0; i<vArray.length; i++){
            if(Math.abs(vArray[i])>1) {
                vArray[i]=positiveNegative(vArray[i]);
            }
        }
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


        //tune for Vector Distance
        //RangeResult rangeResult10 = inRange(directionalAngle, 10);
        //int v1distance, v2distance, v3distance, v4distance;

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //wait until robot stops
            /*
            if (rangeResult10.position != 0) {

                v1distance = frontLeft.getTargetPosition() - frontLeft.getCurrentPosition();
                v2distance = frontRight.getTargetPosition() - frontRight.getCurrentPosition();
                v3distance = backLeft.getTargetPosition() - backLeft.getCurrentPosition();
                v4distance = backRight.getTargetPosition() - backRight.getCurrentPosition();

//                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                gyroToGo(directionalAngle, 10);


                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeft.setTargetPosition(v1distance);
                frontRight.setTargetPosition(v2distance);
                backLeft.setTargetPosition(v3distance);
                backRight.setTargetPosition(v4distance);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                */

                /*
                try{
                   gyroToGo(directionalAngle, 10);
                }
                catch(InterruptedException ex){
                    System.out.print(ex.getMessage());
                }


            }

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);


            rangeResult10 = inRange(directionalAngle, 10);
            */


            opMode.telemetry.update();
            opMode.telemetry.addData("v1", v1);
            opMode.telemetry.addData("v2", v2);
            opMode.telemetry.addData("v3", v3);
            opMode.telemetry.addData("v4", v4);

            opMode.telemetry.update();

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

    void RotateRight(double power, int distance) throws InterruptedException {

        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + distance);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - distance);
        backRight.setTargetPosition(backRight.getCurrentPosition() - distance);


        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotateRight(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //wait until robot stops
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

    //position 0= in range
    //position 1= degree is greater than left
    //position -1= degree is less than right
    public RangeResult inRange(double angle, double offset) {
        opMode.telemetry.addData("ExecutionTimeinMilliseconds", timeElapse / 1000000);
        RangeResult range = new RangeResult();
        opMode.telemetry.update();
        double degree = heading;

        if (degree < 0) {
            degree = degree + 360;
        }

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
        double angleoffset = 5;
        RangeResult rangeresult = inRange(angle, angleoffset);
        int position = rangeresult.position;
        int previousposition = -10;
        double distance = rangeresult.distance;
        double previouspower = 0.3;
        double powerlevel = 0.5;
        double k = 0.3;
        int count = 0;
        while (opMode.opModeIsActive()) {
            opMode.telemetry.update();
            opMode.telemetry.addData("distance", rangeresult.distance);
            opMode.telemetry.addData("powerlevel", powerlevel);
            opMode.telemetry.addData("position", rangeresult.position);
            opMode.telemetry.addData("countTurn", count);
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //adjust power level\
            powerlevel = 0.3;

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    opMode.telemetry.update();
                    opMode.telemetry.addData("Finished with gyro to go-position", rangeresult.position);
                    opMode.telemetry.addData("position:", rangeresult.position);
                    opMode.sleep(300);
                    break;
                }
                //position is left of heading, rotate right
            } else if (position == 1) {
                if ((previouspower != powerlevel || previousposition != position)) {
                    //int deg= Math.round((float) (run360/360)*(float)(distance));
                    //RotateDistance(powerlevel, deg);
                    rotateRight(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                    count++;
                }
                //position is right of heading, rotate left
            } else if (position == -1) {
                if (previouspower != powerlevel || previousposition != position) {
                    //int deg= Math.round((float) (run360/360)*(float)(distance));
                    //RotateDistance(-powerlevel, -deg);
                    rotateLeft(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                    count++;
                }
            }
        }
    }

    public void gyroToGo(double angle, double offset) throws InterruptedException {
        double angleoffset = offset;
        RangeResult rangeresult = inRange(angle, angleoffset);
        int position = rangeresult.position;
        int previousposition = rangeresult.position;
        double distance = rangeresult.distance;
        double previouspower = 0.3;
        double powerlevel = 0.5;
        double k = 0.3;

        while (opMode.opModeIsActive()) {
            opMode.telemetry.update();
            opMode.telemetry.addData("distance", rangeresult.distance);
            opMode.telemetry.addData("angle", angle);
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //adjust power level\
            powerlevel = 0.3;

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    //opMode.sleep(300);
                    break;
                }
                //position is left of heading, rotate right
            } else if (position == 1) {
                if ((previouspower != powerlevel || previousposition != position) || (previouspower != powerlevel && previousposition != position)) {
                    rotateRight(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                }
                //position is right of heading, rotate left
            } else if (position == -1) {
                if ((previouspower != powerlevel || previousposition != position) || (previouspower != powerlevel && previousposition != position)) {
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


    void setHeadingToZero() throws InterruptedException{
        gyroInit();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void gyroInit() throws InterruptedException{

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
