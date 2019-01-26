package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



public class SamplingOrder{
    // Detector object
    private SamplingOrderDetector detector;
    public LinearOpMode opMode;

    public SamplingOrder(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;

    }

    public void enable() throws InterruptedException{

        // Setup detector
        detector = new SamplingOrderDetector(); // Create the detector
        detector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable(); // Start detector
    }

    public String getGoldLocation(){
        String result = "UNKNOWN";
        int leftcount = 0;
        int centercount = 0;
        int rightcount = 0;
        String goldLocation;

        for(int i = 0; i < 5; i++){
            goldLocation = detector.getCurrentOrder().toString();
            if(goldLocation == "LEFT"){
                leftcount++;
            } else if(goldLocation == "CENTER"){
                centercount++;
            } else if(goldLocation == "RIGHT"){
                rightcount++;
            }
            opMode.telemetry.update();
            opMode.sleep(20);
        }
        //get max value
        int[] arr = {leftcount, centercount, rightcount};
        int max = 0;
        for(int i = 0; i < 3; i++){
            if(arr[i] > max){
                max = arr[i];
            }
        }
        if(leftcount == max) {
            result = "LEFT";
        } else if(centercount == max){
            result = "CENTER";
        } else if(rightcount == max){
            result = "RIGHT";
        }
        return result;
    }

    public String getCurrentOrder() throws InterruptedException{
        return detector.getCurrentOrder().toString(); // The current result for the frame
    }

    public String getLastOrder() throws InterruptedException{
        return detector.getLastOrder().toString(); // The last known result
    }


    public void disable() throws InterruptedException{
        detector.disable();
    }
}
