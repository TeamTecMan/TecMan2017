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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Tournament TeleOp", group = "Tecbot 2017-18")
//@Disabled
public class TournamentTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareTecbot2 tecbot2 = new HardwareTecbot2();
    ModernRoboticsI2cGyro tecbot2Gyro = null; // Additional Gyro device

    Methods methods = new Methods();

    static final double STRAFE_MULTIPLIER = 1.5;
    static final double TURN_MULTIPLIER = 0.75;
    double servoUpPos = 0; // Range of 0-1

    double strafeX;
    double strafeY;
    double turn;
    boolean driveFast;
    boolean driveSlow;
    double driveSpeed = 0.7;

    double lift1;
    double lift2;
    boolean grabberOpen;
    boolean grabberClose;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned durin g the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        tecbot2.frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        tecbot2.backLeft = hardwareMap.get(DcMotor.class, "back_left");
        tecbot2.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        tecbot2.backRight = hardwareMap.get(DcMotor.class, "back_right");
        tecbot2.lift1 = hardwareMap.get(DcMotor.class, "lift_1");
        tecbot2.lift2 = hardwareMap.get(DcMotor.class, "lift_2");
        tecbot2.grabber = hardwareMap.get(DcMotor.class, "grabber");
        tecbot2.jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        tecbot2Gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        // tecbot2Gyro requires a different form of defining
        // because ModernRobotics gyro has more options when defined this way


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        tecbot2.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.backLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.frontRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.backRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift1.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift2.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.grabber.setDirection(DcMotor.Direction.REVERSE);
//       tecbot2.jewelServo.setPosition(); // might be redundant, used in autonomous

        tecbot2Gyro.calibrate();

        while (tecbot2Gyro.isCalibrating() && !isStarted()) {
            sleep(50);
            telemetry.addData("Status: ", "Calibrating Gyro");
            telemetry.update();
        }

        if (!tecbot2Gyro.isCalibrating()){
            telemetry.addData("Status: ", "Done calibrating");
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (tecbot2Gyro.isCalibrating()){
            sleep(50);
            telemetry.addData("Status: ", "Calibrating Gyro");
            telemetry.update();
        }

        if (!tecbot2Gyro.isCalibrating()){
            telemetry.addData("Status: ", "Done calibrating");
            telemetry.update();
        }


        tecbot2.frontLeft.setPower(0);
        tecbot2.backLeft.setPower(0);
        tecbot2.frontRight.setPower(0);
        tecbot2.backRight.setPower(0);
        tecbot2.lift1.setPower(0);
        tecbot2.lift2.setPower(0);
        tecbot2.grabber.setPower(0);

        tecbot2.jewelServo.setPosition(servoUpPos);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            lift1 = -gamepad2.left_stick_y / 2;
            lift2 = -gamepad2.right_stick_y / 2;
            grabberOpen = gamepad2.right_bumper;
            grabberClose = gamepad2.left_bumper;

            driveFast = gamepad1.right_bumper;
            driveSlow = gamepad1.left_bumper;

            if (driveFast && !driveSlow) {
                driveSpeed = 0.7;
            }
            else if (!driveFast && driveSlow) {
                driveSpeed = 0.3;
            }

            strafeX = gamepad1.left_stick_x; // Side to side
            strafeY = gamepad1.left_stick_y; // Forward and backward
            turn = -gamepad1.right_stick_x;

            if (grabberOpen && !grabberClose) {
                tecbot2.grabber.setPower(0.25);
            } else if (!grabberOpen && grabberClose) {
                tecbot2.grabber.setPower(-0.25);
            } else {
                tecbot2.grabber.setPower(0);
            }

            if (gamepad1.dpad_up) {
                followGyroHeading(0.7, 0.1);
            }

            // Send calculated power to wheels
            frontLeftPower =  ((strafeY * STRAFE_MULTIPLIER) - (strafeX * STRAFE_MULTIPLIER) + (turn * TURN_MULTIPLIER)) * driveSpeed;
            frontRightPower = ((strafeY * STRAFE_MULTIPLIER) + (strafeX * STRAFE_MULTIPLIER) - (turn * TURN_MULTIPLIER)) * driveSpeed;
            backLeftPower =   ((strafeY * STRAFE_MULTIPLIER) + (strafeX * STRAFE_MULTIPLIER) + (turn * TURN_MULTIPLIER)) * driveSpeed;
            backRightPower =  ((strafeY * STRAFE_MULTIPLIER) - (strafeX * STRAFE_MULTIPLIER) - (turn * TURN_MULTIPLIER)) * driveSpeed;

            tecbot2.frontLeft.setPower(frontLeftPower);
            tecbot2.frontRight.setPower(frontRightPower);
            tecbot2.backLeft.setPower(backLeftPower);
            tecbot2.backRight.setPower(backRightPower);

            correctPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            tecbot2.lift1.setPower(lift1);
            tecbot2.lift2.setPower(lift2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontLeft (%.2f), backLeft (%.2f)", strafeY + strafeX + turn, strafeY - strafeX + turn);
            telemetry.addData("Motors", "frontRight (%.2f), backRight (%.2f)", strafeY - strafeX - turn, strafeY + strafeX - turn);
            telemetry.update();
        }
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

    public void followGyroHeading(double power,
                                  double propConst) {

        double leftPower = power;
        double rightPower = power;

        double startTime = getRuntime();
        double currentTime = getRuntime();
        double stopTime = startTime + time;

        double error;
        double steer;

        double targetHeading = tecbot2Gyro.getIntegratedZValue();

        // Ensure that the opmode is still active
        while (opModeIsActive() && gamepad1.dpad_up) {

            // start motion.
            //power = Range.clip(Math.abs(power), 0.0, 1.0);
            setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
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
            } else if (error < 0) {
                telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
                telemetry.addData("error: ", error);
                telemetry.update();

                rightPower = power - Math.abs(steer);
                leftPower = power;

                if (rightPower < 0) {
                    rightPower = 0;
                }
                setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
            } else {
                telemetry.addData("Gyro Heading: ", tecbot2Gyro.getIntegratedZValue());
                telemetry.addData("error: ", error);
                telemetry.update();

                rightPower = power;
                leftPower = power;

                setDriveMotorPower(leftPower, leftPower, rightPower, rightPower);
            }


            currentTime = getRuntime();

        }

        setDriveMotorPower(0, 0, 0, 0);
    }
}
