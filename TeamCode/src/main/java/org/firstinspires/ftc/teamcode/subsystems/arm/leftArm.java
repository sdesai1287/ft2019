package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class leftArm implements Arm {
    private Servo sideLift;
    private Servo sideGrab;

    private HardwareMap hardwareMap;

    //Positions for Lifter
    private final double UP_POS= 0.4;
    private final double LIFT_POS = 0.55;
    private final double DOWN_POS = 0.8;
    private final double LOWEST_POS = 0.9;
    private final double telePos = 0.6;
    ;

    //Positions for Grabber
    private final double  OPEN_POS= 0.7;
    private final double GRAB_POS = 0.07;


    public leftArm(HardwareMap hardwareMap) throws InterruptedException
    {
        this.hardwareMap = hardwareMap;

        sideLift = hardwareMap.servo.get("sideLiftLeft");
        sideGrab = hardwareMap.servo.get("sideGrabLeft");

        init();
    }


    @Override
    public void init() throws InterruptedException
    {
        up();
        grab();
    }

    @Override
    public void up() throws InterruptedException
    {
        sideLift.setPosition(UP_POS);
        Thread.sleep(500);
    }

    @Override
    public void down()
    {
        sideLift.setPosition(DOWN_POS);
    }

    public void lift()
    {
        sideLift.setPosition(LIFT_POS);
    }


    @Override
    public void grab() throws InterruptedException
    {
        sideGrab.setPosition(GRAB_POS);
        Thread.sleep(500);
    }

    @Override
    public void open() throws InterruptedException
    {
        sideGrab.setPosition(OPEN_POS);
        Thread.sleep(500);
    }

    @Override
    public void grabAuto() throws InterruptedException
    {
        down();
        sideLift.setPosition(LOWEST_POS);
        grab();
    }

    @Override
    public void releaseAuto() throws InterruptedException
    {
        down();
        open();
    }

    @Override
    public void tele()
    {
        sideLift.setPosition(telePos);
    }


}
