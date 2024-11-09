package org.firstinspires.ftc.teamcode;
/*
horizontal claw = Servo Axon = horclaw
horizontal slides extendo = Servo Axon = horext

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
        DcMotorEx vert = hardwareMap.get(DcMotorEx.class, "Vert");
        DcMotorEx vert2 = hardwareMap.get(DcMotorEx.class, "Vert2");
        vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
        //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));

        waitForStart();

        Servo horclaw = hardwareMap.get(Servo.class, "horclaw");
        Servo horwrist1 = hardwareMap.get(Servo.class, "horwrist1");
        //Servo horwrist2 = hardwareMap.get(Servo.class, "horwrist2");
        Servo vertclaw = hardwareMap.get(Servo.class, "vertclaw");
        Servo vertwrist = hardwareMap.get(Servo.class, "vertwrist");
        Servo horext = hardwareMap.get(Servo.class, "horext");
        Servo vertpivot = hardwareMap.get(Servo.class, "vertpivot");


        vert.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        int targetVertPos = 0;
        double UpPower = 0.4;
        double DownPower = 0.1;
        double CurrPower = 0.0;
        vert.setTargetPosition(targetVertPos);
        vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vert2.setTargetPosition(targetVertPos);
        vert2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("slide Pos", vert.getCurrentPosition());
            telemetry.addData("slide Pow", vert.getPower());
            telemetry.addData("targetSlides", vert.getTargetPosition());
            telemetry.update();

            driveTrain.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);

            if (gamepad1.dpad_up) {
                if (vertclaw.getPosition() == 1) horclaw.setPosition(1);
                else {
                    vertclaw.setPosition(1);
                }
            } else if (gamepad1.dpad_down) {
                if (vertclaw.getPosition() == 0) horclaw.setPosition(0);
                else {
                    vertclaw.setPosition(0);
                }
            }
            if (gamepad1.dpad_left) {
                if (horwrist1.getPosition() <= 0.8) {
                    horwrist1.setPosition(horwrist1.getPosition() + 0.2);
                }
            }
            else if (gamepad1.dpad_right) {
                if (horwrist1.getPosition() >= 0.2) horwrist1.setPosition(horwrist1.getPosition() + 0.2);
            }
            //checks to see if the arm is up. Then brings it down or takes it down.
            //programs B button for arm
            if (gamepad1.b) {
                targetVertPos = 4000;
                horext.setPosition(0); //change
//                wrist.setPosition(1);
//                claw.setPosition(1);
            }


            else if (gamepad1.a) {
                targetVertPos = 0;
                 horext.setPosition(0.5); //change
                horext.setPosition(1);
                vertpivot.setPosition(0);
                vertwrist.setPosition(0);
                horwrist1.setPosition(0);
                //horwrist2.setPosition(1);
                horclaw.setPosition(1);
                vertclaw.setPosition(0);

            }
            else if (gamepad1.x) {
                vertwrist.setPosition(0);
                vertpivot.setPosition(0);
            }
            else if (gamepad1.y) {
                vertpivot.setPosition(1);
                vertwrist.setPosition(1);
            }
            else if (gamepad1.left_bumper) {
                horext.setPosition(Math.min(horext.getPosition() + .10, 1));
            } else if (gamepad1.right_bumper) {
                horext.setPosition(Math.max(0, horext.getPosition() - .10));
            }




            if (gamepad1.right_trigger >= 0.3) {
                // TODO: figure out what max slide position is
                targetVertPos = Math.min(vert.getCurrentPosition() + 50, 4000);
                CurrPower = UpPower; // TODO: Someone correct me if I am wrong
            } else if (gamepad1.left_trigger >= 0.3) {
                targetVertPos = Math.max(vert.getCurrentPosition() - 50, 0);
                CurrPower = DownPower;
            }

            vert.setTargetPosition(targetVertPos);
            vert.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vert.setPower(CurrPower);

            vert2.setTargetPosition(targetVertPos);
            vert2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vert2.setPower(CurrPower);
        }
    }
}


