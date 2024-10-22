package org.firstinspires.ftc.teamcode;
/*
TODO: FIX RETURN STRING IT LITERALLY SNAPPED

TODO: ADD CLAW ITEMS
- ritvij has been working on the claw for 1 hour...
TODO: ADD FTC Dash so that we can do this on the spot so it is faster
- done
TODO: Get rid of my crappy subsystems
- after first comp

TODO: better macros w/ claw
TODO: Tune RR
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.PIDFcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "BoomerangTeleop")
public class BoomerangTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
        //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));

        waitForStart();

        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int targetSlidePos = 0;
        int targetArmPos = 0;
        double UpPower = 0.2;
        double DownPower = 0.1;
        double CurrPower = 0.0;
        arm.setTargetPosition(targetArmPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setTargetPosition(targetSlidePos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("position", arm.getCurrentPosition());
            telemetry.addData("velocity", arm.getPower());
            telemetry.addData("targetArm", arm.getTargetPosition());
            telemetry.addData("slide Pos", slides.getCurrentPosition());
            telemetry.addData("slide Pow", slides.getPower());
            telemetry.addData("targetSlides", slides.getTargetPosition());
            telemetry.update();

            driveTrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);

            if (gamepad1.dpad_up) {
                claw.setPosition(1);
            } else if (gamepad1.dpad_down) {
                claw.setPosition(0);
            }

            //checks to see if the arm is up. Then brings it down or takes it down.
            //programs B button for arm
            if (gamepad1.b) {
                targetSlidePos = 10000;
                targetArmPos = -250;
//                wrist.setPosition(1);
//                claw.setPosition(1);
            }
// TODO: Someone tell me whats this supposed to do
            // Who knows what this does... idk

            else if (gamepad1.a) {
                targetSlidePos = 0;

                targetArmPos = 0;
                //currentArmPos = arm.getCurrentPosition();
            } else if (gamepad1.left_bumper) {
                targetArmPos = Math.min(arm.getCurrentPosition() + 10, 0);
            } else if (gamepad1.right_bumper) {
                targetArmPos = Math.max(-250, arm.getCurrentPosition() - 10);
            }

            arm.setTargetPosition(targetArmPos);
            // TODO: adjust power - need more power on way up and when closer to horizontal (math.cos or smth)
            // we don't need very much power at the top
            arm.setPower(0.5);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            if (gamepad1.right_trigger >= 0.3) {
                // TODO: figure out what max slide position is
                targetSlidePos = Math.min(slides.getCurrentPosition() + 10, 10000);
                CurrPower = UpPower; // TODO: Someone correct me if I am wrong
            } else if (gamepad1.left_trigger >= 0.3) {
                targetSlidePos = Math.max(slides.getCurrentPosition() - 10, 0);
                CurrPower = DownPower;
            }

            slides.setTargetPosition(targetSlidePos);
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setPower(CurrPower);
        }
    }
}


