/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

/**
 * {@link imuTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

@Disabled
@TeleOp(name = "IMU Test")
public class imuTest extends LinearOpMode {

    private DcMotor LF, RF, LB, RB;

    // The IMU sensor object
    IIMU inertialMeasurementUnit;
    BNO055IMU bosch;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Acceleration acceleration;
    Velocity velocity;
    Position position;

//    ElapsedTime time;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        inertialMeasurementUnit = new BoschIMU(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        LF = hardwareMap.dcMotor.get("LF");
        RF = hardwareMap.dcMotor.get("RF");
        LB = hardwareMap.dcMotor.get("LB");
        RB = hardwareMap.dcMotor.get("RB");



//        bosch = hardwareMap.get(BNO055IMU.class, "boschIMU");


        inertialMeasurementUnit.init();
//        inertialMeasurementUnit.calibrate();

//        bosch.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Wait until we're told to go
        waitForStart();


        // Loop and update the dashboard
        while (opModeIsActive())
        {
            telemetry.addData("x angle", inertialMeasurementUnit.getXAngle());
            telemetry.addData("y angle", inertialMeasurementUnit.getYAngle());
            telemetry.addData("z angle", inertialMeasurementUnit.getZAngle());
            telemetry.addData("x accel", inertialMeasurementUnit.getXAcc());
            telemetry.addData("y accel", inertialMeasurementUnit.getYAcc());
            telemetry.addData("z accel", inertialMeasurementUnit.getZAcc());
            telemetry.addData("x velo", inertialMeasurementUnit.getXVelo());
            telemetry.addData("y velo", inertialMeasurementUnit.getYVelo());
            telemetry.addData("z velo", inertialMeasurementUnit.getZVelo());
            telemetry.addData("x pos", inertialMeasurementUnit.getXPos());
            telemetry.addData("y pos", inertialMeasurementUnit.getYPos());
            telemetry.addData("z pos", inertialMeasurementUnit.getZPos());
            telemetry.update();
        }
    }
}