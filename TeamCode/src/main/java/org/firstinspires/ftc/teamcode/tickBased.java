package org.firstinspires.ftc.teamcode;

import java.util.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class tickBased extends LinearOpMode {
    DcMotorEx leftFront = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftBack = null;
    DcMotorEx rightBack = null;


    DcMotorEx arm = null;
    DcMotorEx slides = null;

    Servo frontClaw;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    double COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    double     DRIVE_GEAR_REDUCTION    = 20 ;     // No External Gearing.
    double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

//initialize and setup the motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);

        slides = hardwareMap.get(DcMotorEx.class, "arm");
        slides.setDirection(DcMotorEx.Direction.FORWARD);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(0);


        frontClaw = hardwareMap.servo.get("frontClaw");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

    }
        public void encoderDrive(double speed,
        double leftBackInches, double leftFrontInches, double rightBackInches, double rightFrontInches,
        double timeoutS) {
            int newLeftBackTarget;
            int newLeftFrontTarget;
            int newRightBackTarget;
            int newRightFrontTarget;

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
                newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
                newRightBackTarget = leftBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
                newRightFrontTarget = leftFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
                leftFront.setTargetPosition(newLeftFrontTarget);

                leftBack.setTargetPosition(newLeftBackTarget);
                rightFront.setTargetPosition(newRightFrontTarget);
                rightBack.setTargetPosition(newRightBackTarget);

                // Turn On RUN_TO_POSITION
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFront.setVelocity(speed);
                leftBack.setVelocity(speed);
                rightFront.setVelocity(speed);
                rightBack.setVelocity(speed);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                    telemetry.addData("Currently at",  " at %7d :%7d",
                            leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftBack.setVelocity(0);
                leftFront.setVelocity(0);
                rightFront.setVelocity(0);
                rightBack.setVelocity(0);

                // Turn off RUN_TO_POSITION
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

    }




