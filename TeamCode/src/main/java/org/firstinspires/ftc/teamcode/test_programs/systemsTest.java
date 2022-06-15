package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.leftArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.rightArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@Disabled
@Autonomous(name="Systems Test") //Name the class
public class systemsTest extends LinearOpMode {

    //Define floats to be used as joystick inputs and trigger inputs
    private double drivePower, shiftPower, leftTurnPower, rightTurnPower, spoolPower;

    private final double MOTOR_POWER = 0.5;
    private final double CHASSIS_POWER = 0.7;
    private final double STOP_POWER = 0.0;

    private final double ERROR_MARGIN = 0.1;

    private final int LIFT_DISTANCE = 420;

    private final int DRIVE_DISTANCE = 500;

    //    private String team = "red";
    private String team = "blue";

    int two = 2;

    private skystoneChassis chassis;
    private Platform platform;
    private IntakeWheels intake;
    private LinearSlides slides;
    private org.firstinspires.ftc.teamcode.subsystems.stacker.stacker stacker;
    private Arm leftArm;
    private Arm rightArm;
    private IIMU imu;

    private Servo saber;
    private Servo shortSaber;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        saber = hardwareMap.servo.get("saber");
        shortSaber = hardwareMap.servo.get("shortSaber");

        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap);
        leftArm = new leftArm(hardwareMap);
        rightArm = new rightArm(hardwareMap);
        imu = new BoschIMU(hardwareMap);

        platform.up();
        shortSaber.setPosition(0.45);

        telemetry.addData("init complete", two);
        telemetry.update();

        //Wait for start button to be clicked
        waitForStart();
        //***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            chassis.driveForwardsAutonomous(MOTOR_POWER, DRIVE_DISTANCE);

            waitAndPrint("Drive forward");

            chassis.driveForwardsAutonomous(-MOTOR_POWER, -DRIVE_DISTANCE);

            waitAndPrint("Drive backwards");

            chassis.rightShiftAutonomous(MOTOR_POWER, DRIVE_DISTANCE);

            waitAndPrint("Left shift");

            chassis.leftShiftAutonomous(MOTOR_POWER, DRIVE_DISTANCE);

            waitAndPrint("Right shift");

            intake.intake();

            waitAndPrint("Intake start");

            intake.stop();

            waitAndPrint("Intake stop");

            slides.moveSpool(MOTOR_POWER);

            waitAndPrint("Slides up");

            slides.stop();

            waitAndPrint("Slides stop");

            slides.moveSpool(-MOTOR_POWER);

            waitAndPrint("Slides init");



            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main

    public void waitAndPrint(String print) throws InterruptedException
    {
        Thread.sleep(500);
        telemetry.addData(print, ": test complete");
        telemetry.update();
    }
}//Close class and end program



