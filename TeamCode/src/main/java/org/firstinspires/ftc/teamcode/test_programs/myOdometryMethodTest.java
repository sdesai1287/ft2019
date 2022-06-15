//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

@Disabled
@TeleOp(name="My odometry method test") //Name the class
public class myOdometryMethodTest extends LinearOpMode
{
    //Drivetrain
    private DcMotor LF, RF, LB, RB, backOdometer;

    private IIMU imu;

    private double slowPower = 0.5;


    //Define a function to use to set motor powers
    public void setDriveMotorPowers(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        //Use the entered powers and feed them to the motors
        LF.setPower(LFPower);
        LB.setPower(LBPower);
        RF.setPower(RFPower);
        RB.setPower(RBPower);
    }

    public void stopDriving()
    {
        setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    public void goToIMU(double power, double degrees, Telemetry telemetry) throws InterruptedException
    {
        if (abs(imu.getZAngle() - degrees) > 2.0)
        {
            while (imu.getZAngle() < degrees)
            {
                telemetry.addData("Angle", imu.getZAngle());
                telemetry.update();
                setDriveMotorPowers(-power, -power, power, power);
            }
            while (imu.getZAngle() > degrees)
            {
                telemetry.addData("Angle", imu.getZAngle());
                telemetry.update();
                setDriveMotorPowers(power, power, -power, -power);
            }
        }
        stopDriving();
    }

    public void goToPosition(DcMotor yMotor, DcMotor xMotor, double targetYPos, double targetXPos, double power, Telemetry telemetry) throws InterruptedException
    {
        double startAngle = imu.getZAngle();
        double distanceToY = targetYPos - yMotor.getCurrentPosition();
        double distanceToX = targetXPos - xMotor.getCurrentPosition();
        double COEFF;
        while ((distanceToY > 20) || (distanceToX > 20))
        {
            distanceToY = targetYPos - yMotor.getCurrentPosition();
            distanceToX = targetXPos - xMotor.getCurrentPosition();
            COEFF = 1 - (distanceToX / sqrt(pow(distanceToX, 2) + pow(distanceToY, 2)));

            if (abs(targetYPos) > abs(targetXPos))
            {
                if (targetYPos > yMotor.getCurrentPosition() && targetXPos > xMotor.getCurrentPosition())
                {
                    setDriveMotorPowers(-power, -power, -COEFF * power, -COEFF * power);
                }
                if (targetYPos > yMotor.getCurrentPosition() && targetXPos < xMotor.getCurrentPosition())
                {
                    setDriveMotorPowers(-COEFF * power, -COEFF * power, -power, -power);
                }
                if (targetYPos < yMotor.getCurrentPosition() && targetXPos < xMotor.getCurrentPosition())
                {
                    setDriveMotorPowers(COEFF * power, COEFF * power, power, power);
                }
                if (targetYPos < yMotor.getCurrentPosition() && targetXPos > xMotor.getCurrentPosition())
                {
                    setDriveMotorPowers(power, power, COEFF * power, COEFF * power);
                }
            }

            telemetry.addData("Y encoder position: ", yMotor.getCurrentPosition());
            telemetry.addData("X encoder position: ", xMotor.getCurrentPosition());
            telemetry.addData("Angle: ", imu.getZAngle());
            telemetry.update();
        }

        stopDriving();
        Thread.sleep(5000);

        goToIMU(power, startAngle, telemetry);
    }

    public void odometryMotion(DcMotor motor1, DcMotor motor2, double LFPower, double LBPower, double RFPower, double RBPower, int degrees, Telemetry telemetry)
    {
        //Empty while loop while the motors are moving
        while ((abs(motor1.getCurrentPosition() - degrees) > 20) && (abs(motor2.getCurrentPosition() - degrees) > 20))
        {
            telemetry.addData("enc 1", motor1.getCurrentPosition());
            telemetry.addData("enc 2", motor2.getCurrentPosition());
            setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
            telemetry.update();
        }

        //Stop driving
        stopDriving();
    }

    public void odometryDrive(DcMotor motor1, DcMotor motor2, double power, int degrees, Telemetry telemetry) throws InterruptedException
    {
        odometryMotion(motor1, motor2, power, power, power, power, -degrees, telemetry);
    }

    public void odometryLeftShift(DcMotor motor, double power, int degrees, Telemetry telemetry) throws InterruptedException
    {
        odometryMotion(motor, motor, -power, power, power, -power, -degrees, telemetry);
    }

    public void odometryRightShift(DcMotor motor, double power, int degrees, Telemetry telemetry) throws InterruptedException
    {
        odometryMotion(motor, motor, power, -power, -power, power, degrees, telemetry);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        backOdometer = hardwareMap.dcMotor.get("backOdometer");

        //Reverse left side motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = new BoschIMU(hardwareMap);

        //Wait for start button to be clicked
        waitForStart();


//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            if (gamepad1.b)
            {
                backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.a)
            {
                goToPosition(LF, backOdometer,3000, 1200, slowPower, telemetry);
            }

            telemetry.addData("Y encoder position: ", LF.getCurrentPosition());
            telemetry.addData("X encoder position: ", backOdometer.getCurrentPosition());
            telemetry.addData("Angle: ", imu.getZAngle());
            telemetry.update();

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
