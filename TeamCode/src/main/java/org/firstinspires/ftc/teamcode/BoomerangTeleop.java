package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.PIDFcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "BoomerangTeleop")
public class BoomerangTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Gamepad currGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();
        //init drivetrain
        DriveTrain driveTrain = new DriveTrain(hardwareMap,
                new String[]{"frontRight", "frontLeft", "backRight", "backLeft"},
                org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Reverse.RevLeft,
                "imu",
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                ),
                x -> x
        );
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "Arm");
        DcMotorEx slides = hardwareMap.get(DcMotorEx.class, "Slides");
        PIDFcontroller armPidf = new PIDFcontroller(0, 0, 0, 0, 0, 10, 0.8);
        LinearSlide armController = new LinearSlide(arm, new int []{ 0, -300 }, 0, armPidf);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        waitForStart();

        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


//        boolean claw_up = false;
        boolean arm_up = false;
        int sample = 0;

        int currentSlidePos = 0;

        while (opModeIsActive()) {
            prevGamepad.copy(currGamepad);
            currGamepad.copy(gamepad1);

            driveTrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);
            armController.update(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.a, gamepad1.b);

            currentSlidePos = slides.getCurrentPosition();

                //programs A button for claw
//               if (gamepad1.dpad_up){
//                        claw.setPosition(1);
//               } else if (gamepad1.dpad_down) {
//                        claw.setPosition(0);
//                    }

            //checks to see if the arm is up. Then brings it down or takes it down.
            //programs B button for arm
            if (gamepad1.b){
                arm.setTargetPosition(1);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(0.015);
                slides.setTargetPosition(5);
                slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slides.setPower(0.015);
                //wrist.setPosition(1);
//                claw.setPosition(1);
            }

            if (gamepad1.x) {
                slides.setTargetPosition(0);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(-0.015);

                //arm.setTargetPosition(0);
                //arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //wrist.setPosition(0);
                //arm.setPower(-0.015);
                //currentArmPos = arm.getCurrentPosition();
              }



            if (gamepad1.right_trigger >= 0.3 && slides.getCurrentPosition() < 5) {
                // move the slides up
                slides.setPower(0.05);
                slides.setTargetPosition(150);
            } else if (gamepad1.left_trigger >= 0.3 && slides.getCurrentPosition() > 0) {
                // move the slides down
                slides.setPower(-0.05);
                slides.setTargetPosition(0);
            } else {
                // maintain current position
                slides.setTargetPosition(currentSlidePos);
                slides.setPower(0);
            }
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }
}


