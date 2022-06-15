//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.Distance;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.distanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@TeleOp(name="teleOp") //Name the class
public class teleOp extends LinearOpMode
{

    //Define floats to be used as joystick inputs and trigger inputs
    private double drivePower, shiftPower, leftTurnPower, rightTurnPower, turnPower, spoolPower;

    private final double MAX_POWER = 1.0;
    private final double CHASSIS_POWER = 0.6;
    private final double STOP_POWER = 0.0;

    private final double ERROR_MARGIN = 0.1;

    private final int LIFT_DISTANCE = 420;

    private final double SEGMENT_ONE = ERROR_MARGIN;
    private final double SEGMENT_TWO = 0.3;
    private final double SEGMENT_THREE = 0.8;

    private boolean intakeOn = true;

    private skystoneChassis chassis;
    private Platform platform;
    private IntakeWheels intake;
    private LinearSlides slides;
    private stacker stacker;

    private Servo saber;

    private Distance distanceSensor;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        saber = hardwareMap.servo.get("saber");

        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);


        //Wait for start button to be clicked
        waitForStart();

        intake.intake();

    //***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            spoolPower = -(gamepad1.right_stick_y + gamepad2.right_stick_y);
            if (gamepad1.guide)
            {
                drivePower = gamepad1.right_trigger - gamepad1.left_trigger;
                turnPower = gamepad1.left_stick_x;
                chassis.mecanumKinematics2(drivePower, 0, turnPower);
            }
            else
            {
                drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y);
                shiftPower = (gamepad1.left_stick_x + gamepad2.left_stick_x);
                turnPower = (gamepad1.left_trigger + gamepad2.left_trigger - gamepad1.right_trigger - gamepad2.right_trigger) * 0.6;
                chassis.mecanumKinematics1(drivePower, shiftPower, turnPower);
            }

            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) + Math.abs(turnPower) < ERROR_MARGIN)
            {
                chassis.stopDriving();
            }

            if (spoolPower > ERROR_MARGIN)
            {
                intakeOn = false;
                intake.stop();
                slides.moveSpool(spoolPower);
            }
            else if (spoolPower < -ERROR_MARGIN)
            {
                slides.moveSpool(spoolPower);
            }
            else
            {
                if (!gamepad2.left_bumper || !gamepad2.right_bumper)
                {
                    slides.stop();
                }
            }

            if(gamepad1.y)
            {
                platform.up();
            }

            if (gamepad1.x)
            {
                platform.init();
            }

            if(gamepad1.a)
            {
                platform.grab();
            }

            //Saber red
            if (gamepad2.y)
            {
                saber.setPosition(0.25);
            }

            //Saber blue
            if (gamepad2.a)
            {
                saber.setPosition(1.0);
            }

            if (gamepad1.dpad_left)
            {
                distanceSensor.stoneShiftLeft();
            }

            if (gamepad1.dpad_right)
            {
                distanceSensor.stoneShiftRight();
            }

            if (gamepad1.dpad_down)
            {
                distanceSensor.platformReverse();
            }

            if (gamepad2.right_bumper)
            {
                slides.spoolEncoder(MAX_POWER, LIFT_DISTANCE);
            }

            if (gamepad2.left_bumper)
            {
                slides.spoolEncoder(-MAX_POWER, -LIFT_DISTANCE);
            }

            //Eject
            if (gamepad1.left_bumper)
            {
                intake.eject();
                chassis.setDriveMotorPowers(-CHASSIS_POWER, -CHASSIS_POWER, -CHASSIS_POWER, -CHASSIS_POWER);
                Thread.sleep(500);
                chassis.stopDriving();
            }
            //Intake
            else if (intakeOn)
            {
                intake.intake();
            }

            telemetry.addData("Intake: ", intake.getIntakeState());

            //Stacker System
            if (gamepad2.dpad_right)
            {
                stacker.extend();
                intakeOn = false;
                intake.stop();
            }
            else if (distanceSensor.intakeStone() && (!stacker.clearSensor()))
            {
                stacker.grab();
                intakeOn = false;
                intake.stop();
            }

            if (gamepad2.dpad_left)
            {
                stacker.retract();
                intakeOn = true;
                intake.intake();
            }

            if (gamepad2.x)
            {
                stacker.grab();
            }

            if (gamepad2.b)
            {
                stacker.open();
            }

            //Add cap here
            if(gamepad2.back)
            {
                stacker.cap();
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
}//Close class and end program