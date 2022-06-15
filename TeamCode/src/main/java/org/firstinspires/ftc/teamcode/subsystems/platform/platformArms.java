package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class platformArms implements Platform{

    private HardwareMap hardwareMap;

    //Left and right servo arms to grab platform
    private Servo platformLeft;
    private Servo platformRight;

    //Servo positions for init
    private double LEFT_INIT = 0.0;
    private double RIGHT_INIT = 1.0;

    //Servo positions when up
    private double LEFT_GRAB = 0.11;
    private double RIGHT_GRAB = 0.91;

    //Servo positions when latched onto platform
    private double LEFT_UP = 0.35;
    private double RIGHT_UP = 0.69;

    public platformArms(HardwareMap hardwareMap) throws InterruptedException {
        this.hardwareMap = hardwareMap;

        platformLeft = hardwareMap.servo.get("platLeft");
        platformRight = hardwareMap.servo.get("platRight");

        init();

    }

    @Override
    public void init() throws InterruptedException
    {
        platformLeft.setPosition(LEFT_INIT);
        platformRight.setPosition(RIGHT_INIT);
    }


    @Override
    public void up() throws InterruptedException{
        platformLeft.setPosition(LEFT_UP);
        platformRight.setPosition(RIGHT_UP);
        Thread.sleep(400);
    }

    @Override
    public void grab() throws InterruptedException
    {
        platformLeft.setPosition(LEFT_GRAB);
        platformRight.setPosition(RIGHT_GRAB);
        Thread.sleep(1000);
    }

    @Override
    public Servo getPlatformLeft(){
        return platformLeft;
    }

    @Override
    public Servo getPlatformRight(){
        return platformRight;
    }
}