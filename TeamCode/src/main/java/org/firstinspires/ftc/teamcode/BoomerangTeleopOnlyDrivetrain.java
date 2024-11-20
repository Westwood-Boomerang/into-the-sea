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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "BoomerangTeleopOnlyDrivetrain")
public class BoomerangTeleopOnlyDrivetrain extends LinearOpMode {
    public static int maxSlidesPos = 4000;
    public static int slideWallPos = 400;
    public static int topBarSlidePos = 3085;
    public static int topBarSlidePosDown = 2100;
    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //init drivetrain
        DriveTrain driveTrain = new DriveTrain(hardwareMap,
                new String[]{"frontRight", "frontLeft", "backRight", "backLeft"},
                DriveTrain.Reverse.RevBackRight,
                "imu",
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                ),
                x -> x
        );

        //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
        //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));

        waitForStart();

//        Servo horclaw = hardwareMap.get(Servo.class, "horclaw");
//

        while (opModeIsActive()) {


            driveTrain.update(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);


        }
    }
}


