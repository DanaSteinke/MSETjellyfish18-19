package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by FerannoDad on 9/29/18.
 */

public class hMap {
    //Declare motors
    public DcMotor motorLeft;
    public DcMotor motorRight;

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;


    HardwareMap hwMap;



    hMap() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight = hwMap.dcMotor.get("motorRight");

        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}
