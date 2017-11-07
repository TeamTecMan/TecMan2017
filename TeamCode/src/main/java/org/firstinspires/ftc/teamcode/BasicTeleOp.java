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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name = "Basic TeleOp", group = "Tecbot 2017-18")
//@Disabled
public class BasicTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareTecbot2 tecbot2 = new HardwareTecbot2();
    Methods methods = new Methods();

    static final double STRAFE_MULTIPLIER = 1.5;
    static final double TURN_MULTIPLIER   = 0.75;

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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned durin g the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        tecbot2.frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        tecbot2.backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        tecbot2.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        tecbot2.backRight  = hardwareMap.get(DcMotor.class, "back_right");
        tecbot2.lift1      = hardwareMap.get(DcMotor.class, "lift_1");
        tecbot2.lift2      = hardwareMap.get(DcMotor.class, "lift_2");
        tecbot2.grabber    = hardwareMap.get(DcMotor.class, "grabber");
        tecbot2.jewelServo = hardwareMap.get(Servo.class,   "jewel_servo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        tecbot2.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.backLeft.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.frontRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.backRight.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift1.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.lift2.setDirection(DcMotor.Direction.FORWARD);
        tecbot2.grabber.setDirection(DcMotor.Direction.REVERSE);
        tecbot2.jewelServo.setPosition(0); // might be redundant, used in autonomous



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            lift1 = -gamepad2.left_stick_y/2;
            lift2 = -gamepad2.right_stick_y/2;
            grabberOpen = gamepad2.right_bumper;
            grabberClose = gamepad2.left_bumper;

            driveFast = gamepad1.right_bumper;
            driveSlow = gamepad1.left_bumper;

            if     (driveFast && !driveSlow){
                driveSpeed = 0.7;
            }
            else if(!driveFast && driveSlow){
                driveSpeed = 0.3;
            }
            else if(driveFast && driveSlow || !driveFast && !grabberClose){}

            strafeX = gamepad1.left_stick_x; // Forward and backward
            strafeY = gamepad1.left_stick_y; // Side to side
            turn = gamepad1.right_stick_x;

            if     (grabberOpen && !grabberClose){
                tecbot2.grabber.setPower(0.25);
            }
            else if(!grabberOpen && grabberClose){
                tecbot2.grabber.setPower(-0.25);
            }
            else if(grabberOpen && grabberClose || !grabberOpen && !grabberClose){
                tecbot2.grabber.setPower(0);
            }

            // Send calculated power to wheels
            tecbot2.frontLeft.setPower (((strafeY * STRAFE_MULTIPLIER) - (strafeX * STRAFE_MULTIPLIER) + (turn * TURN_MULTIPLIER)) * driveSpeed);
            tecbot2.frontRight.setPower(((strafeY * STRAFE_MULTIPLIER) + (strafeX * STRAFE_MULTIPLIER) - (turn * TURN_MULTIPLIER)) * driveSpeed);
            tecbot2.backLeft.setPower  (((strafeY * STRAFE_MULTIPLIER) + (strafeX * STRAFE_MULTIPLIER) + (turn * TURN_MULTIPLIER)) * driveSpeed);
            tecbot2.backRight.setPower (((strafeY * STRAFE_MULTIPLIER) - (strafeX * STRAFE_MULTIPLIER) - (turn * TURN_MULTIPLIER)) * driveSpeed);

            tecbot2.lift1.setPower(lift1);
            tecbot2.lift2.setPower(lift2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontLeft (%.2f), backLeft (%.2f)", strafeY + strafeX + turn, strafeY - strafeX + turn);
            telemetry.addData("Motors", "frontRight (%.2f), backRight (%.2f)", strafeY - strafeX - turn, strafeY + strafeX - turn);
            telemetry.update();
        }
    }
}
