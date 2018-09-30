package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FerannoDad on 9/29/18.
 */

public class hMap {
    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;

    HardwareMap hwMap;


    hMap() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight = hwMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }
}
