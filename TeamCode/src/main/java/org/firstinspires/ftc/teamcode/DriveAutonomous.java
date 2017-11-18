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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="drive Autonomous", group="Linear Opmode")
@Disabled
public class DriveAutonomous extends LinearOpMode {

    // Declare OpMode members.
    HardwareTecbot2 tecbot2 = new HardwareTecbot2();
    private ElapsedTime runtime = new ElapsedTime(); //Time is in seconds
    Methods methods = new Methods();

    String teamColor = "blue";

    double servoUpPos   = 0.25; // Range of 0-1 (0.25 best position)
    double servoDownPos = 1; // Range of 0-1 (1 best position)
    double jewelServoPos;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tecbot2.frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        tecbot2.backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        tecbot2.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        tecbot2.backRight  = hardwareMap.get(DcMotor.class, "back_right");
        tecbot2.lift1      = hardwareMap.get(DcMotor.class, "lift_1");
        tecbot2.lift2      = hardwareMap.get(DcMotor.class, "lift_2");
        tecbot2.grabber    = hardwareMap.get(DcMotor.class, "grabber");
        tecbot2.jewelServo = hardwareMap.get(Servo.class,   "jewel_servo");
        tecbot2.jewelSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewel_sensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        tecbot2.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.backLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.frontRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.backRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift1.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift2.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.grabber.setDirection(DcMotor.Direction.REVERSE);
        //tecbot2.jewelServo.setPosition(servoUpPos);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        tecbot2.frontLeft.setPower(0.25);
        tecbot2.backLeft.setPower(0.25);
        tecbot2.frontRight.setPower(0.25);
        tecbot2.backRight.setPower(0.25);
        tecbot2.lift1.setPower(0);
        tecbot2.lift2.setPower(0);
        tecbot2.grabber.setPower(0);
        // run until the end of the match (driver presses STOP)
        sleep(3000);
        tecbot2.frontLeft.setPower(0);
        tecbot2.backLeft.setPower(0);
        tecbot2.frontRight.setPower(0);
        tecbot2.backRight.setPower(0);
        tecbot2.lift1.setPower(0);
        tecbot2.lift2.setPower(0);
        tecbot2.grabber.setPower(0);
        sleep(1000);
        tecbot2.frontLeft.setPower(-0.25);
        tecbot2.backLeft.setPower(-0.25);
        tecbot2.frontRight.setPower(-0.25);
        tecbot2.backRight.setPower(-0.25);
        sleep(250);
        tecbot2.frontLeft.setPower(0);
        tecbot2.backLeft.setPower(0);
        tecbot2.frontRight.setPower(0);
        tecbot2.backRight.setPower(0);
        tecbot2.lift1.setPower(0);
        tecbot2.lift2.setPower(0);
        tecbot2.grabber.setPower(0);
    }

    public void knockJewels(){
        double pivotTime  = 2; //In seconds
        double pivotPower = 0.2;

        tecbot2.jewelSensor.enableLed(true);
        if(opModeIsActive()) {
            tecbot2.jewelServo.setPosition(servoDownPos);

            if (tecbot2.jewelSensor.red() > tecbot2.jewelSensor.blue()) {
                if (teamColor.equals("blue")) {
                    pivotRobotByTime("left", pivotTime, pivotPower);
                }
                if (teamColor.equals("red")) {
                    pivotRobotByTime("right", pivotTime, pivotPower);
                }
            } else if (tecbot2.jewelSensor.blue() > tecbot2.jewelSensor.red()) {
                if (teamColor.equals("blue")) {
                    pivotRobotByTime("right", pivotTime, pivotPower);
                }
                if (teamColor.equals("red")) {
                    pivotRobotByTime("left", pivotTime, pivotPower);
                }
            }
        }
        if(opModeIsActive()) {
            tecbot2.jewelServo.setPosition(servoUpPos);
        }
    }

    public void pivotRobotByTime(String turnDirection, double time, double power){
        double startTime = getRuntime();
        double currentTime = getRuntime();

        if(opModeIsActive()) {
            if (turnDirection == "right") {
                tecbot2.frontLeft.setPower(power);
                tecbot2.backLeft.setPower(power);
                tecbot2.frontRight.setPower(-power);
                tecbot2.backRight.setPower(-power);
            }
            if (turnDirection == "left") {
                tecbot2.frontLeft.setPower(-power);
                tecbot2.backLeft.setPower(-power);
                tecbot2.frontRight.setPower(power);
                tecbot2.backRight.setPower(power);
            }
        }

        while((currentTime - startTime < time) && opModeIsActive()) {
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

    public void driveByTime(double time, double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        double startTime = getRuntime();
        double currentTime = getRuntime();

        while((currentTime - startTime < time) && opModeIsActive()) {
            currentTime = getRuntime();
            tecbot2.frontLeft.setPower(leftFrontPower);
            tecbot2.backLeft.setPower(leftBackPower);
            tecbot2.frontRight.setPower(rightFrontPower);
            tecbot2.backRight.setPower(rightBackPower);
        }
    }
}