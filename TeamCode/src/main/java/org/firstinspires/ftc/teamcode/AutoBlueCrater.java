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
    @Autonomous(name = "AutoBlueCrater")
    public class AutoBlueCrater extends LinearOpMode {

        Robot robot;

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(this);
            robot.driveBase.composeTelemetry();
            robot.markerDispenser.setPosition(0.55);


            waitForStart();
            robot.driveBase.setHeadingToZero();

            //Start Auto
            //extend lift
            robot.Lift.ExtendingLift();

            //turn to prep position after extending lift
            robot.driveBase.VectorDistance(0.5, 500, 180);
            robot.driveBase.VectorDistance(1, 200, 270);
            robot.driveBase.VectorDistance(1, 300, 90);
            robot.driveBase.gyroToGo(320);

            //drive to the marker depot
            robot.driveBase.VectorDistance(1, 1500, 180);
            robot.driveBase.gyroToGo(0);
            robot.driveBase.VectorDistance(1, 6000, 180);

            //dispense marker and drive to crater
            robot.markerDispenser.setPosition(0);
            sleep(700);
            robot.markerDispenser.setPosition(0.55);
            sleep(700);
            //robot.driveBase.VectorDistance()


        }

}
