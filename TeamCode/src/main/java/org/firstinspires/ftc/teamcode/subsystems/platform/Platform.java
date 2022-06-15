package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.Servo;

public interface Platform
{
    void up() throws InterruptedException;
    void init() throws InterruptedException;
    void grab() throws InterruptedException;

    Servo getPlatformLeft();
    Servo getPlatformRight();
}
