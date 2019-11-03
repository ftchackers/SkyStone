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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="TeleAll", group="Linear Opmode")
//@Disabled
public class TeleAll extends LinearOpMode {

    private double leftServoUp = 0;
    private double leftServoDown = 1;
    private double rightServoUp = 1;
    private double rightServoDown = 0;
    private double leftServoHalfUp = 0.5;
    private double rightServoHalfUp = 0.5;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;  //
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftRevMotor = null;
    private DcMotor rightRevMotor = null;
    private DcMotor clampRevMotor = null;

    private Servo leftServo = null;
    private Servo rightServo = null;
    static final double     COUNTS_PER_MOTOR_REV    = 14000/7.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LFdrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFdrive");

        leftBackDrive  = hardwareMap.get(DcMotor.class, "LBdrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBdrive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);




        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);



        leftRevMotor = hardwareMap.get(DcMotor.class, "Lrev");
        rightRevMotor = hardwareMap.get(DcMotor.class, "Rrev");
        clampRevMotor = hardwareMap.get(DcMotor.class, "Backrev");
        leftRevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the

        leftRevMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRevMotor.setDirection(DcMotor.Direction.REVERSE);

        rightRevMotor.setPower(0);
        leftRevMotor.setPower(0);
        clampRevMotor.setPower(0);

        leftServo  = hardwareMap.get(Servo.class, "Lservo");
        rightServo = hardwareMap.get(Servo.class, "Rservo");

        leftServo.setPosition(leftServoUp);
        rightServo.setPosition(rightServoUp);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //code from https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example  dmssargent
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;


            leftFrontDrive.setPower(v1);
            rightFrontDrive.setPower(v2);
            leftBackDrive.setPower(v3);
            rightBackDrive.setPower(v4);




            if(gamepad1.left_bumper) {
                telemetry.addData("left bumper pressed", ""+ leftServo.getPosition());
                telemetry.update();
                leftServo.setPosition(leftServoDown);
                rightServo.setPosition(rightServoDown);
            }

            telemetry.addData("left servo: ", ""+leftServo.getPosition());
            telemetry.addData("right servo: ", ""+rightServo.getPosition());
            telemetry.update();
            if(gamepad1.right_bumper) {
                telemetry.addData("right bumper pressed", ""+leftServo.getPosition());
                telemetry.update();
                leftServo.setPosition(leftServoUp);
                rightServo.setPosition(rightServoUp);
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double elevationPower;
            double clampPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            elevationPower = -gamepad2.left_stick_y ;
            clampPower =  -gamepad2.right_stick_y ;

            //elevationPower    = Range.clip(drive, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;



            // Send calculated power to wheels
            leftRevMotor.setPower(elevationPower);
            rightRevMotor.setPower(elevationPower);

            clampRevMotor.setPower(clampPower);


            if(gamepad2.left_bumper) {
                encoderRightDrive(DRIVE_SPEED,  4,  4, 10.0);
                sleep(100);
                leftServo.setPosition(leftServoDown);
                rightServo.setPosition(rightServoDown);
                sleep(100);
                encoderDrive(DRIVE_SPEED,  35,  35, 10.0);
                sleep(250);
                leftServo.setPosition(leftServoUp);
                rightServo.setPosition(rightServoUp);
                sleep(200);
                encoderDrive(DRIVE_SPEED,  9,  9, 10.0);
                sleep(100);

                leftServo.setPosition(leftServoDown);
                rightServo.setPosition(rightServoDown);
                sleep(100);
                /*
                leftServo.setPosition(leftServoHalfUp);
                rightServo.setPosition(rightServoHalfUp);
                sleep(100);
                */
                encoderDrive(DRIVE_SPEED,  -35,  -35, 10.0);
                sleep(200);
                encoderLeftDrive(DRIVE_SPEED,  60,  60, 10.0);
                sleep(200);
                leftServo.setPosition(leftServoUp);
                rightServo.setPosition(rightServoUp);
                sleep(200);
                encoderDrive(DRIVE_SPEED,  -3,  -3, 10.0);
                sleep(200);
                encoderRightDrive(DRIVE_SPEED,  20,  20, 10.0);

            }

            if(gamepad2.right_bumper) {

                encoderLeftDrive(DRIVE_SPEED,  10,  10, 10.0);
            }




            telemetry.addData("left servo: ", ""+leftServo.getPosition());
            telemetry.addData("right servo: ", ""+rightServo.getPosition());
            telemetry.update();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));

            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));

            rightBackDrive.setPower(Math.abs(speed));

            /*newLeftFrontTarget = leftFrontDrive.getCurrentPosition() -2000;
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() -2000;
            newLeftBackTarget = leftBackDrive.getCurrentPosition() -2000;
            newRightBackTarget = rightBackDrive.getCurrentPosition() -2000;
            */

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);

            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

               /*telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                /*
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            sleep(250);   // optional pause after each move
        }
    }

    public void encoderLeftDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));

            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));

            rightBackDrive.setPower(Math.abs(speed));

            /*newLeftFrontTarget = leftFrontDrive.getCurrentPosition() -2000;
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() -2000;
            newLeftBackTarget = leftBackDrive.getCurrentPosition() -2000;
            newRightBackTarget = rightBackDrive.getCurrentPosition() -2000;
            */

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);

            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

               /*telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                /*
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            sleep(250);   // optional pause after each move
        }
    }

    public void encoderRightDrive(double speed,
                                  double leftInches, double rightInches,
                                  double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));

            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));

            rightBackDrive.setPower(Math.abs(speed));

            /*newLeftFrontTarget = leftFrontDrive.getCurrentPosition() -2000;
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() -2000;
            newLeftBackTarget = leftBackDrive.getCurrentPosition() -2000;
            newRightBackTarget = rightBackDrive.getCurrentPosition() -2000;
            */

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);

            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

               /*telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                /*
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
                */
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            sleep(250);   // optional pause after each move
        }
    }
}
