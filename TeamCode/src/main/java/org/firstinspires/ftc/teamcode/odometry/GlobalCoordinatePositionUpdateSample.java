package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.odometry.purePursuit.src.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.odometry.purePursuit.src.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.odometry.purePursuit.src.treamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.odometry.purePursuit.src.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.odometry.purePursuit.src.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.odometry.purePursuit.src.RobotUtilities.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.odometry.purePursuit.src.treamcode.MathFunctions.lineCircleIntersection;

/**
 * Created by Samedh on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    DcMotor LF, LB, RF, RB, intakeLeft;

    double ms = 0.8;
    double ts = 0.8;
    double fd = 50;
    double sdta = 1.0;

    IIMU imu;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 200;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "LF", verticalRightEncoderName = "RB", horizontalEncoderName = "intakeLeft";

    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        LF = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get(verticalRightEncoderName);
        intakeLeft = hardwareMap.dcMotor.get(horizontalEncoderName);

        imu = new BoschIMU(hardwareMap);

        //Reset the encoders
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the LF and RB encoders spin forward, they return positive values, and when the
        backOdometer encoder travels to the right, it returns positive value
        */

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reverse left side motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
         globalPositionUpdate = new OdometryGlobalCoordinatePosition(LF, RB, intakeLeft, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

//        goToPosition(20 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.4, -90, 1 * COUNTS_PER_INCH);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0, 0, ms, ts, fd, Math.toRadians(50), sdta));
//        allPoints.add(new CurvePoint(180, 180, ms, ts, fd, Math.toRadians(50), sdta));
//        allPoints.add(new CurvePoint(220, 180, ms, ts, fd, Math.toRadians(50), sdta));
//        allPoints.add(new CurvePoint(280, 50, ms, ts, fd, Math.toRadians(50), sdta));
//
//        followCurve(allPoints, 0);

        while(opModeIsActive())
        {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Orientation (IMU)", imu.getZAngle());

            telemetry.addData("Vertical Left Encoder Position", LF.getCurrentPosition());
            telemetry.addData("Vertical Right Encoder Position", RB.getCurrentPosition());
            telemetry.addData("Horizontal Encoder Position", intakeLeft.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    public void goToPosition(double targetYPosition, double targetXPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while (opModeIsActive() && distance > allowableDistanceError)
        {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double pivotCorrection = AngleWrap(robotMovementAngle - globalPositionUpdate.returnOrientation());
            double relativeTurnAngle1 = pivotCorrection + desiredRobotOrientation;

            double robotMovementTurnComponent = Range.clip(relativeTurnAngle1 / 30, -1, 1) * robotPower;

            //These are the power variables to feed into mecanum kinematics
            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            //Call mecanum kinematics 1 or 2 here
            mecanumKinematics1(robotMovementYComponent, robotMovementXComponent, robotMovementTurnComponent);
        }
        setDriveMotorPowers(0,0,0,0);
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public void setDriveMotorPowers(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        LF.setPower(-LFPower);
        LB.setPower(-LBPower);
        RF.setPower(-RFPower * 0.95);
        RB.setPower(-RBPower * 0.95);
    }

    public void mecanumKinematics1(double yPower, double xPower, double turnPower)
    {
        double LFPower = yPower + xPower - turnPower;
        double LBPower = yPower - xPower - turnPower;
        double RFPower = yPower - xPower + turnPower;
        double RBPower = yPower + xPower + turnPower;
        setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
    }

    public void mecanumKinematics2(double yPower, double xPower, double turnPower)
    {
        double LFPower = yPower + xPower + turnPower;
        double LBPower = yPower - xPower + turnPower;
        double RFPower = yPower - xPower - turnPower;
        double RBPower = yPower + xPower - turnPower;
        setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
    }

    public double AngleWrap(double angle)
    {
        while (angle < -180)
        {
            angle += 360;
        }

        while (angle > 180)
        {
            angle -= 360;
        }
        return angle;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle)
    {
        for (int i = 0; i < allPoints.size() - 1; i++)
        {
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y), new FloatPoint(allPoints.get(i+1).x, allPoints.get(i+1).y));
        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()), allPoints.get(0).followDistance);

        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPoint(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius)
    {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size() - 1; i++)
        {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000000;

            for (Point thisIntersection : intersections)
            {
                double angle = Math.atan2(thisIntersection.y - globalPositionUpdate.returnYCoordinate(), thisIntersection.x - globalPositionUpdate.returnXCoordinate());
                double deltaAngle = Math.abs(AngleWrap(angle - Math.toRadians(globalPositionUpdate.returnOrientation())));

                if (deltaAngle < closestAngle)
                {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void goToPoint(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed)
    {
        double distanceToTarget = Math.hypot(x-globalPositionUpdate.returnXCoordinate(), y-globalPositionUpdate.returnYCoordinate());

        double absoluteAngleToTarget = Math.atan2(y - globalPositionUpdate.returnYCoordinate(), x - globalPositionUpdate.returnYCoordinate());

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (Math.toRadians(globalPositionUpdate.returnOrientation()) - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10)
        {
            movement_turn = 0;
        }

        mecanumKinematics1(movement_y, movement_x, movement_turn);
    }

}