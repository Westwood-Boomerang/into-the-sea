package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BoomerangTeleop")
public class BoomerangTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        //init drivetrain
        driveTrain DriveTrain = new driveTrain(hardwareMap, driveTrain.Reverse.RevLeft,  new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)

        ),(x) -> {return x;});

        DcMotorEx arm;
        DcMotorEx slides;
        Servo claw;
        Servo wrist;

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        slides = hardwareMap.get(DcMotorEx.class, "Slides");
        claw = hardwareMap.get(Servo.class, "Claw");
        wrist = hardwareMap.get(Servo.class, "Wrist");

        boolean claw_up = false;
        boolean arm_up = false;
        int sample = 0;
        waitForStart();


        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                DriveTrain.update(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x, gamepad1.start);
                // code goes here!,

                int lift = slides.getCurrentPosition();

                //programs A button for claw
                if (gamepad1.a){
                    if (!claw_up) {
                        claw.setPosition(1);
                        claw_up = true;
                    }
                    else{
                        claw.setPosition(0);
                        claw_up = false;
                    }
                }

                //checks to see if the arm is up. Then brings it down or takes it down.
                //programs B button for arm
                if (gamepad1.b){
                    if (!arm_up){
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(500);
                        arm.setPower(0.5);
                        wrist.setPosition(1);
                        claw.setPosition(1);
                        arm_up = true;
                        claw_up = true;
                    }
                    else if (arm_up){
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(0);
                        arm.setPower(0.5);
                        wrist.setPosition(0.5);
                        arm_up = false;
                    }
                }

                if (gamepad1.x) {
                    sample ++;
                    if (sample % 2 != 0) {
                        //arm.setDirection(DcMotorEx.Direction.REVERSE);
                        arm.setPower(-0.5);
                        arm.setTargetPosition(500);
                        wrist.setPosition(0);}
                    else {
                        arm.setPower(0.5);
                        arm.setTargetPosition(0);
                        wrist.setPosition(0.5);
                        claw.setPosition(0);
                    }
                }
                if (gamepad1.y){
                    if (!arm_up){
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.5);
                        arm.setTargetPosition(500);
                        arm_up = true;
                    }
                    else {
                        arm.setPower(0.5);
                        arm.setTargetPosition(0);
                        arm_up = false;
                    }
                }
                //keeps the slides while left trigger is not pressed
                if (gamepad1.right_trigger <= 0.3){
                    slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    slides.setTargetPosition(lift);
                }
                //moves the slides up while right trigger is pressed
                else {
                    slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (slides.getCurrentPosition() < 495) {
                        slides.setPower(0.5);
                        lift = slides.getCurrentPosition();
                    }
                    else{
                        slides.setPower(-0.01);
                        slides.setTargetPosition(495);
                    }
                }
                //keeps the slides while left trigger is not pressed
                if (gamepad1.left_trigger <= 0.3){
                    slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    slides.setTargetPosition(lift);
                }
                //moves the slides down while left trigger is pressed
                else {

                    slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (slides.getCurrentPosition() > 0) {
                        slides.setPower(-0.5);
                        lift = slides.getCurrentPosition();
                    }
                    else {
                        slides.setPower(0.01);
                        slides.setTargetPosition(0);
                    }
                }

            }
        }



    }
        }
