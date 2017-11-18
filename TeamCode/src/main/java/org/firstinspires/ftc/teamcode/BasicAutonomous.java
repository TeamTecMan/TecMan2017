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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Tournament Autonomous", group="Linear Opmode")
//@Disabled
public class BasicAutonomous extends LinearOpMode {

    // Declare OpMode members.
    HardwareTecbot2 tecbot2 = new HardwareTecbot2();
    private ElapsedTime runtime = new ElapsedTime(); //Time is in seconds
    //Methods methods = new Methods();

    String teamColor = "blue";

    double servoUpPos = 0.25; // Range of 0-1 (0.25 best position)
    double servoDownPos = 0.95; // Range of 0-1 (1 best position)
    double jewelServoPos;
    ModernRoboticsI2cGyro tecbot2Gyro = null; // Additional Gyro device

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        tecbot2.frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        tecbot2.backLeft = hardwareMap.get(DcMotor.class, "back_left");
        tecbot2.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        tecbot2.backRight = hardwareMap.get(DcMotor.class, "back_right");
        tecbot2.lift1 = hardwareMap.get(DcMotor.class, "lift_1");
        tecbot2.lift2 = hardwareMap.get(DcMotor.class, "lift_2");
        tecbot2.grabber = hardwareMap.get(DcMotor.class, "grabber");
        tecbot2.jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        tecbot2.jewelSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewel_sensor");
        tecbot2Gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        // tecbot2Gyro requires a different form of defining
        // because ModernRobotics gyro has more options when defined this way

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        tecbot2.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.backLeft.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.frontRight.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.backRight.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.lift1.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift2.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.grabber.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.jewelServo.setPosition(servoUpPos);

        // initialize gyro
        telemetry.addData("Status: ", "preparing to calibrate gyro");
        telemetry.update();

        tecbot2Gyro.calibrate();
        while (tecbot2Gyro.isCalibrating()) {
            sleep(50);
            telemetry.addData("Status: ", "Calibrating Gyro");
            telemetry.update();
        }
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();


        while (!isStarted()) {
            //Setting team color
            if (gamepad1.x) {
                teamColor = "blue";
            }
            if (gamepad1.b) {
                teamColor = "red";
            }
            jewelServoPos = tecbot2.jewelServo.getPosition();
            telemetry.addData("Jewel Servo Pos: ", jewelServoPos);
            telemetry.addData("Team Color: ", teamColor);
            telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        gyroDriveByTime(0.25, 120, 0, 0.025);

        //todo grab glyph
        //todo lift glyph

        knockJewels();

        sleep(5000);

        //Split code into 2 teams
        if (teamColor.equals("blue")) {
            // turn to cryptobox
            pivotRobotByGyro("cClockwise", 85, 0.1);
            //todo drive off platform
            //todo drop glyph
            //todo push glyph into cryptobox
            //todo back away from glyph
        }

        if (teamColor.equals("red")) {
            // turn to cryptobox
            pivotRobotByGyro("clockwise", -85, 0.1);

            //todo drive off platform
            //todo drop glyph
            //todo push glyph into cryptobox
            //todo back away from glyph
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status: ", "Run Time: " + runtime.toString());
            telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        tecbot2.frontLeft.setPower(0);
        tecbot2.backLeft.setPower(0);
        tecbot2.frontRight.setPower(0);
        tecbot2.backRight.setPower(0);
        tecbot2.lift1.setPower(0);
        tecbot2.lift2.setPower(0);
        tecbot2.grabber.setPower(0);
    }

    public void knockJewels() {
        double pivotTime = 0.15; //In seconds
        double pivotPower = 0.2;

        tecbot2.jewelSensor.enableLed(true);
        if (opModeIsActive()) {
            tecbot2.jewelServo.setPosition(servoDownPos);

            sleep(5000);

            while ((tecbot2.jewelSensor.red() > tecbot2.jewelSensor.blue()) == false &&
                    (tecbot2.jewelSensor.blue() > tecbot2.jewelSensor.red()) == false) {
                pivotRobotByTime("clockwise", 0.005, 0.1);
            }

            sleep(5000);

            if (tecbot2.jewelSensor.red() > tecbot2.jewelSensor.blue()) {
                if (teamColor.equals("blue")) {
                    pivotRobotByTime("clockwise", pivotTime, pivotPower);
                    telemetry.addData("Sensor Color: ", "blue");
                }
                if (teamColor.equals("red")) {
                    pivotRobotByTime("cClockwise", pivotTime, pivotPower);
                    telemetry.addData("Sensor Color: ", "blue");
                }
            } else if (tecbot2.jewelSensor.blue() > tecbot2.jewelSensor.red()) {
                if (teamColor.equals("blue")) {
                    pivotRobotByTime("cClockwise", pivotTime, pivotPower);
                    telemetry.addData("Sensor Color: ", "red");
                }
                if (teamColor.equals("red")) {
                    pivotRobotByTime("clockwise", pivotTime, pivotPower);
                    telemetry.addData("Sensor Color: ", "red");
                }
            }
        }

        sleep(5000);

        if (opModeIsActive()) {
            tecbot2.jewelServo.setPosition(servoUpPos);
        }
    }

    public void pivotRobotByTime(String turnDirection, double time, double power) {
        double startTime = getRuntime();
        double currentTime = getRuntime();

        if (opModeIsActive()) {
            if (turnDirection.equals("clockwise")) {
                tecbot2.frontLeft.setPower(power);
                tecbot2.backLeft.setPower(power);
                tecbot2.frontRight.setPower(-power);
                tecbot2.backRight.setPower(-power);
            } else if (turnDirection.equals("cClockwise")) {
                tecbot2.frontLeft.setPower(-power);
                tecbot2.backLeft.setPower(-power);
                tecbot2.frontRight.setPower(power);
                tecbot2.backRight.setPower(power);
            }
        }

        while ((currentTime - startTime < time) && opModeIsActive()) {
            currentTime = getRuntime();

            sleep(50);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Turn direction: ", turnDirection);
            telemetry.update();
        }
        tecbot2.frontLeft.setPower(0);
        tecbot2.backLeft.setPower(0);
        tecbot2.frontRight.setPower(0);
        tecbot2.backRight.setPower(0);
    }

    public void pivotRobotByGyro(String turnDirection, double targetGyroHeading, double power) {

        double currentGyroHeading = tecbot2Gyro.getIntegratedZValue();

        if (turnDirection.equals("clockwise")) {
            telemetry.addData("Status: ", "turningCW");
            setDriveMotorPower(power, power, -power, -power);
            while ((currentGyroHeading >= targetGyroHeading) && opModeIsActive()) {
                sleep(50);
                currentGyroHeading = tecbot2Gyro.getIntegratedZValue();
                telemetry.addData("Gyro Heading: ", currentGyroHeading);
                telemetry.update();
            }
        } else if (turnDirection.equals("cClockwise")) {
            telemetry.addData("Status: ", "turningCW");
            setDriveMotorPower(-power, -power, power, power);
            while ((currentGyroHeading <= targetGyroHeading) && opModeIsActive()) {
                sleep(50);
                currentGyroHeading = tecbot2Gyro.getIntegratedZValue();
                telemetry.addData("Gyro Heading: ", currentGyroHeading);
                telemetry.update();
            }

        }

        setAllDriveMotorPower(0);

    }

    public void driveByTime(double time, double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        double startTime = getRuntime();
        double currentTime = getRuntime();

        while ((currentTime - startTime < time) && opModeIsActive()) {
            currentTime = getRuntime();
            tecbot2.frontLeft.setPower(leftFrontPower);
            tecbot2.backLeft.setPower(leftBackPower);
            tecbot2.frontRight.setPower(rightFrontPower);
            tecbot2.backRight.setPower(rightBackPower);
        }
    }

    public void setAllDriveMotorPower(double allMotorPowers) {
        tecbot2.frontLeft.setPower(allMotorPowers);
        tecbot2.backLeft.setPower(allMotorPowers);
        tecbot2.frontRight.setPower(allMotorPowers);
        tecbot2.backRight.setPower(allMotorPowers);
    }

    public void correctPower(double frontLeftPower, double backLeftPower,
                             double frontRightPower, double backRightPower) {

        while (Math.round(Math.abs(tecbot2.frontLeft.getPower() * 10)) != Math.round(Math.abs(frontLeftPower * 10))) {
            tecbot2.frontLeft.setPower(frontLeftPower);
        }
        while (Math.round(Math.abs(tecbot2.backLeft.getPower() * 10)) != Math.round(Math.abs(backLeftPower * 10))) {
            tecbot2.backLeft.setPower(backLeftPower);
        }
        while (Math.round(Math.abs(tecbot2.frontRight.getPower() * 10)) != Math.round(Math.abs(frontRightPower * 10))) {
            tecbot2.frontRight.setPower(frontRightPower);
        }
        while (Math.round(Math.abs(tecbot2.backRight.getPower() * 10)) != Math.round(Math.abs(backRightPower * 10))) {
            tecbot2.backRight.setPower(backRightPower);
        }
    }

    public void setDriveMotorPower(double frontLeftPower, double backLeftPower,
                                   double frontRightPower, double backRightPower) {
        tecbot2.frontLeft.setPower(frontLeftPower);
        tecbot2.backLeft.setPower(backLeftPower);
        tecbot2.frontRight.setPower(frontRightPower);
        tecbot2.backRight.setPower(backRightPower);
    }

    public void gyroDriveByTime(double power,
                                double time,
                                double targetHeading,
                                double propConst) {

//        int newLeftTarget;
//        int newRightTarget;
//        int moveCounts;
//        double max;
//        double error;
//        double steer;
        double leftPower = power;
        double rightPower = power;

        double startTime = getRuntime();
        double currentTime = getRuntime();
        double stopTime = startTime + time;

        double error;
        double steer;

        // Ensure that the opmode is still active
        while (opModeIsActive() && (currentTime < stopTime)) {

            // start motion.
            //power = Range.clip(Math.abs(power), 0.0, 1.0);
            setDriveMotorPower(leftPower,leftPower,rightPower, rightPower);
            //correctPower(power, power, power, power);

            error = targetHeading - tecbot2Gyro.getIntegratedZValue();
            steer = error * propConst;


            if (error > 0) {
                telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
                telemetry.addData("error: ", error);
                telemetry.update();

                rightPower = power;
                leftPower = power - Math.abs(steer);

                if (leftPower < 0) {
                    leftPower = 0;
                }
                setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
            }

            else if (error < 0) {
                telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
                telemetry.addData("error: ", error);
                telemetry.update();

                rightPower = power - Math.abs(steer);
                leftPower = power;

                if (rightPower < 0) {
                    rightPower = 0;
                }
                setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
            }

            else {
                telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
                telemetry.addData("error: ", error);
                telemetry.update();

                rightPower = power;
                leftPower = power;

                setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
            }


            currentTime = getRuntime();

        }
    }
}