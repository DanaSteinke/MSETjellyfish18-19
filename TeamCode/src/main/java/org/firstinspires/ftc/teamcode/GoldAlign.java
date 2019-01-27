package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;

public class GoldAlign {
    public LinearOpMode opMode;
    private GoldAlignDetector goldD;

    public GoldAlign(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;
    }

    public void enable() throws InterruptedException{
        //GOLD MINERAL
        // Set up detector
        goldD = new GoldAlignDetector(); // Create detector
        goldD.init(this.opMode.hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        goldD.useDefaults(); // Set detector to use default settings

        //Alignment
        goldD.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldD.alignPosOffset = 0; // How far from center frame to offset this alignment zone.

        // Optional tuning
        goldD.downscale = 0.4; // How much to downscale the input frames

        goldD.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldD.maxAreaScorer.weight = 0.005;
        goldD.ratioScorer.weight = 5;
        goldD.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        goldD.enable();
    }

    public void disable() throws InterruptedException{
        goldD.disable();
    }

    //return
    public boolean isFound() throws InterruptedException{
        return goldD.isFound();
    }
    public double getGoldXPosition() throws InterruptedException{
        return goldD.getXPosition();
    }
    public boolean getAligned(){
        return goldD.getAligned();
    }

}
