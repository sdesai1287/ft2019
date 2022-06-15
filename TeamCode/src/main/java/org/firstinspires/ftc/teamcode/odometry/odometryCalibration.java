package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.io.File;

/**
 * Created by Samedh on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
//@Disabled
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class odometryCalibration extends LinearOpMode
{
    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;

    DcMotor intakeLeft;

    //IMU Sensor
    IIMU imu;

    final float PIVOT_SPEED = (float) 0.4;

    //NEED TO UPDATE THIS VALUE
    final double COUNTS_PER_INCH = 200;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");

        //Get references to the boschIMU Motors from the hardware map
        imu = new BoschIMU(hardwareMap);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Reverse left side motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "boschIMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.init();
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(imu.getZAngle() < 90 && opModeIsActive()){
            LF.setPower(PIVOT_SPEED);
            LB.setPower(PIVOT_SPEED);
            RF.setPower(-PIVOT_SPEED);
            RB.setPower(-PIVOT_SPEED);
            if(imu.getZAngle() < 60) {
                LF.setPower(PIVOT_SPEED);
                LB.setPower(PIVOT_SPEED);
                RF.setPower(-PIVOT_SPEED);
                RB.setPower(-PIVOT_SPEED);
            }else{
                LF.setPower(PIVOT_SPEED / 2);
                LB.setPower(PIVOT_SPEED / 2);
                RF.setPower(-PIVOT_SPEED / 2);
                RB.setPower(-PIVOT_SPEED / 2);
            }

            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = imu.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(LF.getCurrentPosition()) + (Math.abs(RB.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = intakeLeft.getCurrentPosition()/Math.toRadians(imu.getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Vertical Left Position", LF.getCurrentPosition());
            telemetry.addData("Vertical Right Position", RB.getCurrentPosition());
            telemetry.addData("Horizontal Position", intakeLeft.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }
}