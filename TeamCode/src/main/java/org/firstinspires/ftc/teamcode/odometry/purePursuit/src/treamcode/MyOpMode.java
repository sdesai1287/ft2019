package org.firstinspires.ftc.teamcode.odometry.purePursuit.src.treamcode;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.odometry.purePursuit.src.treamcode.RobotMovement.followCurve;

public class MyOpMode extends OpMode
{
    double ms = 0.8;
    double ts = 0.8;
    double fd = 50;
    double sdta = 1.0;
    @Override
    public void init() {

    }

    @Override
    public void loop()
    {
//        RobotMovement.goToPosition(358/2, 358/2, 0.3, Math.toRadians(90), 0.3);
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, ms, ts, fd, Math.toRadians(50), sdta));
        allPoints.add(new CurvePoint(180, 180, ms, ts, fd, Math.toRadians(50), sdta));
        allPoints.add(new CurvePoint(220, 180, ms, ts, fd, Math.toRadians(50), sdta));
        allPoints.add(new CurvePoint(280, 50, ms, ts, fd, Math.toRadians(50), sdta));

        followCurve(allPoints, 0);
    }
}
