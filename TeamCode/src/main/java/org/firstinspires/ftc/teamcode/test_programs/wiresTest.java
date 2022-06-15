package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis.oneMotorEncoder;

@Disabled
@TeleOp(name="Wires Test ") //Name the class
public class wiresTest extends LinearOpMode {
    //Define drive motors
    private DcMotor LF, LB, RF, RB;

    private DcMotor intakeLeft, intakeRight;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

//        RF.setDirection(DcMotor.Direction.REVERSE);
//        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            if (gamepad1.y)
            {
                oneMotorEncoder(LF, power, degrees);
                telemetry.addData("left front", "forwards");
            }
            if (gamepad1.a)
            {
                oneMotorEncoder(LB, power, degrees);
                telemetry.addData("left back", "forwards");
            }
            if (gamepad1.b)
            {
                oneMotorEncoder(RF, power, degrees);
                telemetry.addData("right front", "forwards");
            }
            if (gamepad1.x)
            {
                oneMotorEncoder(RB, power, degrees);
                telemetry.addData("right back", "forwards");
            }

            if (gamepad1.dpad_up)
            {
                oneMotorEncoder(LF, -power, -degrees);
                telemetry.addData("left front", "backwards");
            }
            if (gamepad1.dpad_right)
            {
                oneMotorEncoder(RF, -power, -degrees);
                telemetry.addData("right front", "backwards");
            }
            if (gamepad1.dpad_down)
            {
                oneMotorEncoder(LB, -power, -degrees);
                telemetry.addData("left back", "backwards");
            }
            if (gamepad1.dpad_left)
            {
                oneMotorEncoder(RB, -power, -degrees);
                telemetry.addData("right back", "backwards");
            }

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
