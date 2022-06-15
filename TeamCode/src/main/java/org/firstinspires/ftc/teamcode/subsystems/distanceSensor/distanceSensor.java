package org.firstinspires.ftc.teamcode.subsystems.distanceSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

public class distanceSensor implements Distance
{
    private DistanceSensor distRight;
    private DistanceSensor stoneDistLeft;
    private DistanceSensor stoneDistRight;
    private DistanceSensor stoneDistLow;
    private DistanceSensor stoneIntake;

    private HardwareMap hardwareMap;

    private skystoneChassis chassis;
    private IIMU imu;

    double power = 0.3;

    public distanceSensor(HardwareMap hardwareMap)
    {
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");

        stoneDistLeft = hardwareMap.get(DistanceSensor.class, "stoneDistLeft");
        stoneDistRight = hardwareMap.get(DistanceSensor.class, "stoneDistRight");
        stoneDistLow = hardwareMap.get(DistanceSensor.class, "stoneDistLow");
        stoneIntake = hardwareMap.get(DistanceSensor.class, "stoneIntake");

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        imu = new BoschIMU(hardwareMap);
    }

    @Override
    public void distLeftShift(double power, double distance) throws InterruptedException{
//        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//
//        runTime.reset();
//
//        double timeOut = distance / 2;
//        double startAngle = 0;
//        double COEFF = 0.94;
//        while (!(distLeft.getDistance(DistanceUnit.INCH)<distance) && (runTime.time()<timeOut))
//        {
//            chassis.shiftTeleop(power);
//            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
//            {
//                if (imu.getZAngle() > startAngle)
//                {
//                    chassis.setDriveMotorPowers(-COEFF * power, power, COEFF * power, -power);
//                }
//
//                if (imu.getZAngle() < startAngle)
//                {
//                    chassis.setDriveMotorPowers(-power, COEFF * power, power, -COEFF * power);
//                }
//            }
//        }
//        chassis.stopDriving();
    }

    @Override
    public void distRightShift(double power, double distance) throws InterruptedException{
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        double startAngle = 0;
        double timeOut = distance / 2;
        double COEFF = 0.94;
        while (!(distRight.getDistance(DistanceUnit.INCH)<distance) && (runTime.time()<timeOut))
        {
            chassis.shiftTeleop(-power);
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                if (imu.getZAngle() > startAngle)
                {
                    chassis.setDriveMotorPowers(COEFF * power, -power, -COEFF * power, power);
                }

                if (imu.getZAngle() < startAngle)
                {
                    chassis.setDriveMotorPowers(power, -COEFF * power, -power, COEFF * power);
                }
            }
        }
        chassis.stopDriving();
    }

    @Override
    public void stoneShiftLeft() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        runTime.reset();

        while (!(stoneDistLeft.getDistance(DistanceUnit.INCH) < 5) && (runTime.seconds() < 2))
        {
            chassis.shiftTeleop(-power);
        }
        chassis.leftShiftAutonomous(power, 40);
        platformReverse();
    }

    @Override
    public void stoneShiftRight() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        runTime.reset();

        while (!(stoneDistRight.getDistance(DistanceUnit.INCH) < 5) && (runTime.seconds() < 2))
        {
            chassis.shiftTeleop(power);
        }
        chassis.rightShiftAutonomous(power, 40);
        platformReverse();
    }

    @Override
    public void platformReverse() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        runTime.reset();

        while (!(stoneDistLow.getDistance(DistanceUnit.INCH) < 7) && (runTime.seconds() < 3 ))
        {
            chassis.driveTeleop(-power);
        }
        chassis.stopDriving();
    }

    @Override
    public boolean intakeStone()
    {
        if (stoneIntake.getDistance(DistanceUnit.INCH) < 3)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
