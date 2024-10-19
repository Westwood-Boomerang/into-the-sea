package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                ),
                x -> x
        );
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();


        DcMotorEx arm;
        DcMotorEx slides;
        Servo claw;
        Servo wrist;

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        slides = hardwareMap.get(DcMotorEx.class, "Slides");
//        claw = hardwareMap.get(Servo.class, "Claw");
        wrist = hardwareMap.get(Servo.class, "Wrist");

//        boolean claw_up = false;
        boolean arm_up = false;
        int sample = 0;

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();



        while (opModeIsActive()) {
            prevGamepad.copy(currGamepad);
            currGamepad.copy(gamepad1);

            driveTrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);
            //arm.update(currGamepad.dpad_up && !prevGamepad.dpad_up, false, false, false);
            //if (gamepad1.dpad_up)
            //foo.setPower(1.0);
            //else foo.setPower(0.0);


            // code goes here!,
            /*
            int lift = slides.getCurrentPosition();

                //programs A button for claw
//                if (gamepad1.a){
//                    if (!claw_up) {
//                        claw.setPosition(1);
//                        claw_up = true;
//                    }
//                    else{
//                        claw.setPosition(0);
//                        claw_up = false;
//                    }
//                }

                //checks to see if the arm is up. Then brings it down or takes it down.
                //programs B button for arm
                if (gamepad1.b){
                    if (!arm_up){
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(600);
                        arm.setPower(0.5);
                        wrist.setPosition(1);
//                        claw.setPosition(1);
                        arm_up = true;
//                        claw_up = true;
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
                        arm.setPower(0.5);
                        arm.setTargetPosition(0);
                        wrist.setPosition(0);
                    arm_up = false;}
                    else {
                        arm.setPower(0.5);
                        arm.setTargetPosition(600);
                        wrist.setPosition(0.5);
//                        claw.setPosition(0);
                        arm_up = true;
                    }
                }
                if (gamepad1.y){
                    if (!arm_up){
                        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.5);
                        arm.setTargetPosition(600);
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

             */
        }
    }
}


