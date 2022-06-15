package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class slides implements LinearSlides{

    private DcMotor spoolLeft, spoolRight;

    HardwareMap hardwareMap;

    int start;

    //Add variables for distance needed to turn to go up 1 skystone distance

    public slides(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        spoolLeft = hardwareMap.dcMotor.get("spoolLeft");
        spoolRight = hardwareMap.dcMotor.get("spoolRight");

        //Reverse right spool motor, both turn in same direction
        spoolLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        spoolLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        start = spoolLeft.getCurrentPosition();
    }

    @Override
    public void moveSpool(double spoolPower) throws InterruptedException{
        spoolLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spoolRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spoolLeft.setPower(spoolPower);
        spoolRight.setPower(spoolPower);
    }

    public void stop() throws InterruptedException{
        spoolLeft.setPower(0.0);
        spoolRight.setPower(0.0);
    }

    public DcMotor getSpoolLeft(){
        return spoolLeft;
    }

    public DcMotor getSpoolRight(){
        return spoolRight;
    }

    public void spoolEncoder(double power, int degrees) throws InterruptedException{
//        spoolLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int initial = spoolLeft.getCurrentPosition();

        int target = initial - degrees;

        ElapsedTime runTime = new ElapsedTime();

        runTime.reset();

        if (target > initial){
            while ((spoolLeft.getCurrentPosition() < target) && runTime.seconds() < 0.75)
            {
                moveSpool(power);
            }
        }

        if (initial > target){
            while ((spoolLeft.getCurrentPosition() > target) && runTime.seconds() < 0.75){
                moveSpool(power);
            }
        }
        stop();
    }

    public void spoolReturn(double power) throws InterruptedException{

        spoolLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int initial = spoolLeft.getCurrentPosition();

        int target = start;

        ElapsedTime runTime = new ElapsedTime();

        runTime.reset();

        if (target > initial){
            while ((spoolLeft.getCurrentPosition() < target) && runTime.seconds() < 3.0){
                moveSpool(power);
            }
        }

        if (initial > target){
            while ((spoolLeft.getCurrentPosition() > target) && runTime.seconds() < 3.0){
                moveSpool(power);
            }
        }
        stop();
    }



}
