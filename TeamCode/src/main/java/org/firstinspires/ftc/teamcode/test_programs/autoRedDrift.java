//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;

@Disabled
@Autonomous(name="Auto Red Drift", group = "Red") //Name the class
public class autoRedDrift extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.4;
    private float TURN_POWER = (float) 0.2;
    private float SHIFT_POWER = (float) 0.3;

    private DcMotor LF, RF, LB, RB;

//    private DistanceSensor distLeft;

//    int driveDistance;

    //Skystone location variable
//    private CV.location skystoneLocation = CV.location.MID;

//    private Arm arm;
//    private Arm arm2;
    private IntakeWheels intake;
//    private LinearSlides slides;
//    private skystoneChassis chassis;
//    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;
//    private stacker stacker;

//    private int STONE_SPACE = 300;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {

        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        platform = new platformArms(hardwareMap);




        intake = new intake(hardwareMap);
        imu = new BoschIMU(hardwareMap);
        imu.init();

        intake.intake();

        //Wait for start button to be clicked
        waitForStart();


//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
//            platform.middle();



            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





            while(LF.getCurrentPosition()<100){
                LF.setPower(0.5);
                RF.setPower(0.5);
                LB.setPower(0.5);
                RB.setPower(0.5);
            }


            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while(RF.getCurrentPosition()<1300){
                LF.setPower(0.2);
                RF.setPower(0.8);
                LB.setPower(0.2);
                RB.setPower(0.8);
            }


            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
////            Thread.sleep(700);
//
            while(LF.getCurrentPosition()<900){
                LF.setPower(0.6);
                RF.setPower(-0.2);
                LB.setPower(-0.2);
                RB.setPower(0.6);
            }

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while(LF.getCurrentPosition()>-400){
                LF.setPower(-0.6);
                RF.setPower(0.2);
                LB.setPower(0.2);
                RB.setPower(-0.6);
            }

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while(imu.getZAngle()<90){
                LF.setPower(-0.6);
                RF.setPower(-0.2);
                LB.setPower(-0.6);
                RB.setPower(-0.2);
            }

//            if (imu.getZAngle()>90){
            while(imu.getZAngle()>90){
                LF.setPower(0.2);
                RF.setPower(-0.2);
                LB.setPower(0.2);
                RB.setPower(-0.2);
            }
//            }

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//

//
            while(LF.getCurrentPosition()>-2500){
                LF.setPower(-0.4);
                RF.setPower(-0.4);
                LB.setPower(-0.4);
                RB.setPower(-0.4);
            }

            platform.up();


            while(imu.getZAngle()>0){
                LF.setPower(-0.2);
                RF.setPower(0.2);
                LB.setPower(-0.2);
                RB.setPower(0.2);
            }

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(LF.getCurrentPosition()>-600){
                LF.setPower(-0.4);
                RF.setPower(-0.4);
                LB.setPower(-0.4);
                RB.setPower(-0.4);
            }

            LF.setPower(0.0);
            RF.setPower(0.0);
            LB.setPower(0.0);
            RB.setPower(0.0);


            platform.grab();

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while(LF.getCurrentPosition()<500){
                LF.setPower(0.4);
                RF.setPower(0.4);
                LB.setPower(0.4);
                RB.setPower(0.4);
            }

            LF.setPower(0.0);
            RF.setPower(0.0);
            LB.setPower(0.0);
            RB.setPower(0.0);








            stop();


//
//            Thread.sleep(700);
//
//            LF.setPower(-0.4);
//            RF.setPower(0.1);
//            LB.setPower(0.1);
//            RB.setPower(-0.4);
//
//            Thread.sleep(500);
//
//            while(imu.getZAngle()<90){
//                LF.setPower(-0.6);
//                RF.setPower(-0.2);
//                LB.setPower(-0.6);
//                RB.setPower(-0.2);
//            }
//
////
//////            Thread.sleep(300);
//
//            LF.setPower(0.0);
//            RF.setPower(0.0);
//            LB.setPower(0.0);
//            RB.setPower(0.0);
////
////
//            LF.setPower(-0.3);
//            RF.setPower(-0.3);
//            LB.setPower(-0.3);
//            RB.setPower(-0.3);
//
//
//            Thread.sleep(3000);
//
//            while(imu.getZAngle()>0){
//                LF.setPower(-0.4);
//                RF.setPower(0.4);
//                LB.setPower(-0.4);
//                RB.setPower(0.4);
//            }
//

//            LF.setPower(0.0);
//            RF.setPower(0.0);
//            LB.setPower(0.0);
//            RB.setPower(0.0);




//            chassis.leftShiftAutonomous(0.5, 250);



//            chassis.driveForwardsAutonomous(0.5, 1000);





//
//            double startAngle = imu.getZAngle();
//            double COEFF = 0.94;
//
//            while((!(distLeft.getDistance(DistanceUnit.INCH)<8.75)))
//            {
//                chassis.shiftTeleop(0.5);
//                if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
//                {
//                    if (imu.getZAngle() > startAngle) {
//                        chassis.setDriveMotorPowers(-COEFF * 0.5, 0.5, COEFF * 0.5, - 0.5);
//                    }
//
//                    if (imu.getZAngle() < startAngle) {
//                        chassis.setDriveMotorPowers(-0.5, COEFF * 0.5, 0.5, -COEFF * 0.5);
//                    }
//                }
//            }
//            chassis.stopDriving();
//
//            Thread.sleep(200);
//
//            if (skystoneLocation== CV.location.LEFT) {
//                driveDistance = 0;
//            }
//            if (skystoneLocation== CV.location.MID) {
//                driveDistance = 225;
//            }
//            if (skystoneLocation== CV.location.RIGHT) {
//                driveDistance = 470;
//            }
//
//            chassis.driveForwardsAutonomous(DRIVE_POWER, driveDistance);
//
//
//            arm.grabAuto();
//
//            Thread.sleep(500);
//
//            arm.lift();
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 200);
//
//
//            chassis.driveForwardsAutonomous(DRIVE_POWER, 2950 - driveDistance);
//
//            Thread.sleep(300);
//
//            stacker.platformLeftShift();
//
//            arm.init();
//
//            Thread.sleep(200);
//            arm.open();
//            arm.up();
//            arm.grab();
//
//            chassis.rightShiftAutonomous(SHIFT_POWER,200);
//
//            Thread.sleep(300);
//
//            chassis.rightTurnIMU(0.5,-90);
//
////            chassis.driveForwardsAutonomous(-DRIVE_POWER, -200);
//
//            stacker.stoneReverseAuto();
//
//            platform.grab();
//
//            Thread.sleep(200);
//
////                        chassis.driveForwardsAutonomous(DRIVE_POWER / 1.2, 1200);
////            chassis.rightShiftAutonomous(SHIFT_POWER, 1000);
////            chassis.driveForwardsAutonomous(-DRIVE_POWER, 500);
////            chassis.rightShiftAutonomous(SHIFT_POWER, 500);
//
//            imu.init();
//
//            while (imu.getZAngle() > -90)
//            {
//                chassis.setDriveMotorPowers(DRIVE_POWER* 1.5, DRIVE_POWER * 1.5, DRIVE_POWER / 3, DRIVE_POWER / 3);
//            }
//
//            chassis.stopDriving();
//
//            imu.init();
//
//            platform.up();
//
//            chassis.rightShiftAutonomous(SHIFT_POWER,225);
//
//
//            chassis.driveForwardsAutonomous(-DRIVE_POWER, -650);
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 250);
//
//            chassis.driveForwardsAutonomous(DRIVE_POWER, 1200);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
