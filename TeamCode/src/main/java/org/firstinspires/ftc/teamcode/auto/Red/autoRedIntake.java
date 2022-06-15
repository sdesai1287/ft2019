//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Red;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.CV.CV;
import org.firstinspires.ftc.teamcode.subsystems.CV.skystoneDetector;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.Distance;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.distanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@Autonomous(name="AutoRed Intake", group = "Red") //Name the class
public class autoRedIntake extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.5;
    private float TURN_POWER = (float) 0.3;
    private float SHIFT_POWER = (float) 0.5;

    private float DRIFT_POWER = (float) 0.5;

    int driveDistance;
    int driveDistance2;
    int shiftDistance;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private skystoneChassis chassis;
    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;
    private IntakeWheels intake;
    private stacker stacker;
    private Distance distanceSensor;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        detector = new skystoneDetector(hardwareMap, telemetry, true);
        intake = new intake(hardwareMap);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        platform = new platformArms(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);
        imu = new BoschIMU(hardwareMap);
        stacker = new stacker(hardwareMap);

        //Look for Skystone until play is pressed
        while(!isStarted()){ skystoneLocation = detector.getSkystoneInfinite(); }

        //Wait for start button to be clicked
        waitForStart();

        telemetry.addData("Skystone", skystoneLocation);

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            intake.intake();

            if (skystoneLocation == CV.location.LEFT)
            {
                shiftDistance = 1100;
                driveDistance = 3850;
            }

            if (skystoneLocation == CV.location.MID)
            {
                shiftDistance = 800;
                driveDistance = 1000;
                driveDistance2 = 500;
            }

            if (skystoneLocation == CV.location.RIGHT)
            {
                shiftDistance = 550;
                driveDistance = 2600;
            }

            chassis.driveForwardsAutonomousPID(1050);

            chassis.leftShiftAutonomous(SHIFT_POWER, shiftDistance);

            while (imu.getZAngle() < 35)
            {
                chassis.setDriveMotorPowers(0, 0, TURN_POWER, TURN_POWER);
            }

            platform.up();

            if (distanceSensor.intakeStone())
            {
                intake.eject();
            }

            chassis.driveBackwardsAutonomousPID(-900);

            stacker.grab();

            chassis.leftTurnIMUPID(84);

            chassis.driveBackwardsAutonomousPID(-driveDistance);

            chassis.leftTurnIMUPID(17);

            while (imu.getZAngle() > 0)
            {
                chassis.leftTurnTeleop(TURN_POWER / 2);
            }

            distanceSensor.platformReverse();

            Thread.sleep(200);

            platform.grab();

            Thread.sleep(200);

            chassis.setDriveMotorPowers(1.0,1.0, 0.1, 0.1);
            Thread.sleep(100);

            //drift turn
            while (imu.getZAngle() > 150)
            {
                stacker.extend();
                chassis.setDriveMotorPowers(1.0,1.0, 0.1, 0.1);
            }
            chassis.stopDriving();

            chassis.rightTurnIMU(TURN_POWER * 3, 90);

            stacker.retract();

            intake.intake();

            chassis.driveForwardsAutonomousPID(900);

            platform.up();

            chassis.driveForwardsAutonomousPID(700);

            chassis.rightTurnIMUPID(35);

            chassis.driveForwardsAutonomous(DRIVE_POWER / 1.5, 900);

            chassis.driveBackwardsAutonomousPID(-900);
//
//            stacker.grab();
//
//            chassis.leftTurnIMUPID(90);
//
//            chassis.driveBackwardsAutonomousPID(-500);
//
//            stacker.extend();
//
//            stacker.ungrab();
//
//            chassis.driveForwardsAutonomousPID(600);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
