package org.firstinspires.ftc.teamcode.subsystems.arm;

public interface Arm
{

    //Put the arm up
    void up() throws InterruptedException;
    //Put the arm init
    void down() throws InterruptedException;
    //Put the arm between up and init
    void lift() throws InterruptedException;

    //Grab the block
    void grab() throws InterruptedException;

    void grabAuto() throws InterruptedException;

    void releaseAuto() throws InterruptedException;
    //Open the grabber
    void open() throws InterruptedException;

    //Init the arm for teleOp
    void init() throws InterruptedException;

    void tele() throws InterruptedException;
    //Init the arm for autonomous
}
