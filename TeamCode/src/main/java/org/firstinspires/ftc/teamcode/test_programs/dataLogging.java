//Run from the package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "Data Logging Program") //Name the program
public class dataLogging extends LinearOpMode {
    //Define drive motors
    private DcMotor LF, LB, RF, RB;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

//***************************************************************************************************************************
        //While the op mode is active, loop and read the RGB data.
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //If b is pressed, reset the encoders
            if (gamepad1.b)
            {
                //Reset the encoders
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //Show all the encoder values on the driver station
            telemetry.addData("LEFT FRONT", LF.getCurrentPosition());
            telemetry.addData("LEFT BACK", LB.getCurrentPosition());
            telemetry.addData("RIGHT FRONT", RF.getCurrentPosition());
            telemetry.addData("RIGHT BACK", RB.getCurrentPosition());

            //Update the data if/when it changes
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while(opModeIsActive())" loop
    } //Close "run Opmode" loop
} //Close class and end program