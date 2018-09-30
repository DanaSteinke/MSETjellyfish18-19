package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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



        //Encoders



    }
}
