//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Flipper Test") //Name the class
public class flipTest extends LinearOpMode {

    private Servo flipper;

    private DcMotor LF, RF, LB, RB;

    private double TUCK = 0.3;
    private double DOWN = 1.0;
    private double UP = 0.5;

    //Define floats to be used as joystick inputs and trigger inputs
    private double drivePower, shiftPower, leftTurnPower, rightTurnPower;



    //Define a function to use to set motor powers
    public void setDriveMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        LF.setPower(leftFrontPower);
        LB.setPower(leftBackPower);
        RF.setPower(rightFrontPower);
        RB.setPower(rightBackPower);
    }

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {

        flipper = hardwareMap.servo.get("flipper");

        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);


        //Wait for start button to be clicked
        waitForStart();

    //***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()){

            if (gamepad1.a){
                flipper.setPosition(TUCK);
            }
            if (gamepad1.y){
                flipper.setPosition(DOWN);
            }
            if (gamepad1.x){
                flipper.setPosition(UP);
            }

            drivePower = -((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.3);
            shiftPower = -((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.3);
            leftTurnPower = ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.3);
            rightTurnPower = ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.3);


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            }


            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                setDriveMotorPowers(-shiftPower, shiftPower, shiftPower, -shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
                setDriveMotorPowers(-leftTurnPower, -leftTurnPower, leftTurnPower, leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
}//Close class and end program