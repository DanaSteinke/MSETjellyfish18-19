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

import java.util.Vector;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.hMap;

/**
 * Created by Feranno on 9/29/18.
 */
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.driveBase.composeTelemetry();
        robot.driveBase.setHeadingToZero();


        waitForStart();

        //Start Auto
        robot.driveBase.VectorDistance(0.5,2000,90);
        robot.driveBase.VectorDistance(0.5,2000, 180);
        robot.driveBase.VectorDistance(0.5, 2000, 270);
        robot.driveBase.VectorDistance(0.5, 2000, 0);
        robot.driveBase.gyroToGo(90);
        robot.driveBase.gyroToGo(180);
        robot.driveBase.gyroToGo(270);
        robot.driveBase.gyroToGo(0);



    }
}
