package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface LinearSlides {

    void moveSpool(double spoolPower) throws InterruptedException;
    void stop() throws InterruptedException;
    void spoolEncoder(double power, int degrees) throws InterruptedException;
    void spoolReturn(double power) throws InterruptedException;
    DcMotor getSpoolLeft();
    DcMotor getSpoolRight();
}
