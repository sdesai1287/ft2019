package org.firstinspires.ftc.teamcode.subsystems.CV;

public interface CV {


    enum stone { SKY, STONE, UNKNOWN }

    public stone left = stone.UNKNOWN;
    public stone mid = stone.UNKNOWN;
    public stone right = stone.UNKNOWN;

    enum location { LEFT, MID, RIGHT}
}
