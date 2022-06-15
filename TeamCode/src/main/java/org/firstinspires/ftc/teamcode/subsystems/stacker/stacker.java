package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

public class stacker
{

    //Gripper
    private Servo gripper;
    private double OPEN_POS = 0.25;
    private double RELEASE_POS = 0.1;
    private double CLOSED_POS = 0.0;

    //Cantilever left
    private Servo cantileverLeft;
    private double EXTEND_POS_LEFT = 0.05;
    private double INTAKE_POS_LEFT = 0.7;
    private double GRAB_POS_LEFT = 0.8;

    //Cantilever right
    private Servo cantileverRight;
    private double EXTEND_POS_RIGHT = 0.95;
    private double INTAKE_POS_RIGHT = 0.33;
    private double GRAB_POS_RIGHT = 0.23;


    //Capstone
    private Servo capstone;
    private double INIT_POS = 0.45;
    private double CAP_POS = 0.0;

    private HardwareMap hardwareMap;

    private LinearSlides slides;
    private skystoneChassis chassis;

    Gamepad gamepad1, gamepad2;


    public stacker(HardwareMap hardwareMap) throws InterruptedException {

        this.hardwareMap = hardwareMap;

        gripper = hardwareMap.servo.get("gripper");
        cantileverLeft = hardwareMap.servo.get("cantileverLeft");
        cantileverRight = hardwareMap.servo.get("cantileverRight");
        capstone = hardwareMap.servo.get("capstone");

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

        init();

    }

    public void init()
    {
        open();
        resetCantilever();
        capstone.setPosition(INIT_POS);
    }

    public void open()
    {
        gripper.setPosition(OPEN_POS);
    }

    public void grab()
    {
        cantileverLeft.setPosition(GRAB_POS_LEFT);
        cantileverRight.setPosition(GRAB_POS_RIGHT);
        gripper.setPosition(CLOSED_POS);
    }

    public void ungrab()
    {
        gripper.setPosition(RELEASE_POS);
    }

    public boolean clearSensor()
    {
        if ((cantileverLeft.getPosition() < INTAKE_POS_LEFT) && (cantileverRight.getPosition() > INTAKE_POS_RIGHT))
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    public void resetCantilever()
    {
        cantileverLeft.setPosition(INTAKE_POS_LEFT);
        cantileverRight.setPosition(INTAKE_POS_RIGHT);
    }

    public void extend()
    {
        cantileverLeft.setPosition(EXTEND_POS_LEFT);
        cantileverRight.setPosition(EXTEND_POS_RIGHT);
    }

    public void retract() throws InterruptedException
    {
        ungrab();
        slides.moveSpool(1.0);
        Thread.sleep(200);
        slides.stop();
        resetCantilever();
        chassis.driveForwardsAutonomous(0.3, 50);
        Thread.sleep(500);
        open();
    }

    public void cap()
    {
        grab();
        capstone.setPosition(CAP_POS);
    }
}