package org.firstinspires.ftc.teamcode.subsystems.distanceSensor;

public interface Distance
{
    void distLeftShift(double power, double distance) throws InterruptedException;
    void distRightShift(double power, double distance) throws InterruptedException;

    void stoneShiftLeft() throws InterruptedException;
    void stoneShiftRight() throws InterruptedException;
    void platformReverse() throws InterruptedException;

    boolean intakeStone();

}
