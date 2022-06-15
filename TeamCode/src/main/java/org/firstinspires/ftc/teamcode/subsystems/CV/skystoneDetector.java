package org.firstinspires.ftc.teamcode.subsystems.CV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


public class skystoneDetector implements CV {
    private int valMid = -1;
    private int valLeft = -1;
    private int valRight = -1;

    private int RUNTIME;

    //Camera
    private OpenCvInternalCamera.CameraDirection CAMERA_DIRECTION = OpenCvInternalCamera.CameraDirection.FRONT;
    private OpenCvCameraRotation ROTATION = OpenCvCameraRotation.SIDEWAYS_RIGHT;

    private static float rectHeight = 1.0f/8f;
    private static float rectWidth = 1.5f/8f;

    private float[] leftPos = {1.5f/8f, 7.2f/8f};//0 = col, 1 = row
    private float[] midPos = {3.5f/8f, 7.2f/8f};
    private float[] rightPos = {5.5f/8f, 7.2f/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    //3 Stones
    private stone left = stone.UNKNOWN;
    private stone mid = stone.UNKNOWN;
    private stone right = stone.UNKNOWN;

    private Boolean isRed;

    private location skystoneLocation = location.MID;

    public Boolean isFound = false;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    OpenCvCamera phoneCam;


    public skystoneDetector(HardwareMap myHardwareMap, Telemetry myTelemetry, Boolean myIsRed) {
        hardwareMap = myHardwareMap;
        telemetry = myTelemetry;

        isRed = myIsRed;

        if (!isRed){
            leftPos[0] = 2.0f/8f;//0 = col, 1 = row
            midPos[0] = 4.0f/8f;
            rightPos[0] = 6.0f/8f;
        }

        camSetup();
        





    }

    public void setOffset(float offsetX, float offsetY) {
        midPos[0] += offsetX;
        midPos[1] += offsetY;
        leftPos[0] += offsetX;
        leftPos[1] += offsetY;
        rightPos[0] += offsetX;
        rightPos[1] += offsetY;
   }

    public void camSetup () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(CAMERA_DIRECTION, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, ROTATION);//display on RC
    }

    public void camClose() {
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
    }

    public stone type(int val) {
        //Stone val = 255
        if (val == 255) { return stone.STONE; }

        //Skystone val = 0
        if (val == 0) { return stone.SKY; }

        return stone.UNKNOWN;
    }


    public void lookForSkystone(){
        //Update Stones
        left = type(valLeft);
        mid = type(valMid);
        right = type(valRight);

        //Skystone found at Left
        if (left.equals(stone.SKY) && mid.equals(stone.STONE) && right.equals(stone.STONE)){
            if(isRed){
                skystoneLocation = location.LEFT;
            }
            skystoneLocation = location.LEFT;
            isFound = true;
        }
        //Skystone found at Mid
        if (left.equals(stone.STONE) && mid.equals(stone.SKY) && right.equals(stone.STONE)){
            skystoneLocation = location.MID;
            isFound = true;
        }
        //Skystone found at Right
        if (left.equals(stone.STONE) && mid.equals(stone.STONE) && right.equals(stone.SKY)){
            if(isRed){
                skystoneLocation = location.RIGHT;
            }
            skystoneLocation = location.RIGHT;
            isFound = true;
        }
    }

    public Boolean getIsFound(){
        return isFound;
    }


    public location getSkystoneTime(int milliseconds) throws InterruptedException{
        RUNTIME /= 10;
        double count = 0;

        while (count < RUNTIME && !isFound) {

            //Look for Skystone
            lookForSkystone();

            //Log what CV is detecting
            telemetry.addData("Values", left + "   " + mid + "   " + right);
            telemetry.update();

            //Check if Skystone was found
            isFound = getIsFound();

            Thread.sleep(10);
            count++;
        }
        telemetry.addData("Location", skystoneLocation);
        telemetry.update();
//        camClose();
        return skystoneLocation;
    }


    public location getSkystoneInfinite() throws InterruptedException{
       //Look for Skystone
            lookForSkystone();

            //Log what CV is detecting
            telemetry.addData("Values", left + "   " + mid + "   " + right);
            telemetry.update();

            //Check if Skystone was found
            isFound = getIsFound();


        telemetry.addData("Location", skystoneLocation);
        telemetry.update();
//        camClose();
        return skystoneLocation;
    }


    //detection pipeline
    public class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {

            //color diff cb.
            //lower cb = more blue = skystoneLocation = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            yCbCrChan2Mat.copyTo(all);//copies mat object

            updateVals(input);

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            return all;
        }

        public void updateVals(Mat input) {
            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

        }
    }
}
