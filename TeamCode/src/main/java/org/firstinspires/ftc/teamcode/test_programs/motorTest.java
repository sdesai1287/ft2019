package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="Motor Test ") //Name the class
public class motorTest extends LinearOpMode {

    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

//    //Define other motors
//    DcMotor dumper;
//    DcMotor intakeLeft;
//    DcMotor intakeRight;

    Boolean aPressed=false;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("LF");
        rightMotorFront = hardwareMap.dcMotor.get("RF");
        leftMotorBack = hardwareMap.dcMotor.get("LB");
        rightMotorBack = hardwareMap.dcMotor.get("RB");


        leftMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior type = DcMotor.ZeroPowerBehavior.FLOAT;

        leftMotorFront.setZeroPowerBehavior(type);
        rightMotorFront.setZeroPowerBehavior(type);
        leftMotorBack.setZeroPowerBehavior(type);
        rightMotorBack.setZeroPowerBehavior(type);

//        dumper = hardwareMap.dcMotor.get("dumper");
//        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
//        intakeRight = hardwareMap.dcMotor.get("intakeRight");


        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {

            int LFStart = leftMotorFront.getCurrentPosition();
            int RFStart = rightMotorFront.getCurrentPosition();
            int LBStart = leftMotorBack.getCurrentPosition();
            int RBStart = rightMotorBack.getCurrentPosition();

            while (gamepad1.y){

                leftMotorFront.setPower(0.6);
                telemetry.addData("LMF: ", leftMotorFront.getCurrentPosition()-LFStart);

                rightMotorFront.setPower(0.6);
                telemetry.addData("RMF: ", rightMotorFront.getCurrentPosition()-RFStart);

                leftMotorBack.setPower(0.6);
                telemetry.addData("LMB: ", leftMotorBack.getCurrentPosition()-LBStart);

                rightMotorBack.setPower(0.6);
                telemetry.addData("RMB: ", rightMotorBack.getCurrentPosition()-RBStart);

                telemetry.update();
            }
            while (gamepad1.x){

                leftMotorFront.setPower(-0.6);
                telemetry.addData("LMF: ", leftMotorFront.getCurrentPosition()-LFStart);

                rightMotorFront.setPower(0.6);
                telemetry.addData("RMF: ", rightMotorFront.getCurrentPosition()-RFStart);

                leftMotorBack.setPower(0.6);
                telemetry.addData("LMB: ", leftMotorBack.getCurrentPosition()-LBStart);

                rightMotorBack.setPower(-0.6);
                telemetry.addData("RMB: ", rightMotorBack.getCurrentPosition()-RBStart);

                telemetry.update();
            }
            while (gamepad1.b){

                leftMotorFront.setPower(0.6);
                telemetry.addData("LMF: ", leftMotorFront.getCurrentPosition()-LFStart);

                rightMotorFront.setPower(-0.6);
                telemetry.addData("RMF: ", rightMotorFront.getCurrentPosition()-RFStart);

                leftMotorBack.setPower(-0.6);
                telemetry.addData("LMB: ", leftMotorBack.getCurrentPosition()-LBStart);

                rightMotorBack.setPower(0.6);
                telemetry.addData("RMB: ", rightMotorBack.getCurrentPosition()-RBStart);

                telemetry.update();
            }
            while (gamepad1.a){

                leftMotorFront.setPower(-0.6);
                telemetry.addData("LMF: ", leftMotorFront.getCurrentPosition()-LFStart);

                rightMotorFront.setPower(-0.6);
                telemetry.addData("RMF: ", rightMotorFront.getCurrentPosition()-RFStart);

                leftMotorBack.setPower(-0.6);
                telemetry.addData("LMB: ", leftMotorBack.getCurrentPosition()-LBStart);

                rightMotorBack.setPower(-0.6);
                telemetry.addData("RMB: ", rightMotorBack.getCurrentPosition()-RBStart);

                telemetry.update();
            }

            leftMotorFront.setPower(0.0);
            rightMotorFront.setPower(0.0);
            rightMotorBack.setPower(0.0);
            leftMotorBack.setPower(0.0);







            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program