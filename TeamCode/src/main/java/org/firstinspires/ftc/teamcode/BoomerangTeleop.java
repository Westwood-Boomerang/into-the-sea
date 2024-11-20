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
@TeleOp(name = "BoomerangTeleop")
public class BoomerangTeleop extends LinearOpMode {
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
                org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Reverse.RevLeft,
                "imu",
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                ),
                x -> x
        );
/*
        DcMotorEx vert = hardwareMap.get(DcMotorEx.class, "Vert");
        DcMotorEx vert2 = hardwareMap.get(DcMotorEx.class, "Vert2");
        vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
        //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
*/
        waitForStart();

//        Servo horclaw = hardwareMap.get(Servo.class, "horclaw");
//        Servo horwrist1 = hardwareMap.get(Servo.class, "horwrist1");
        //Servo horwrist2 = hardwareMap.get(Servo.class, "horwrist2");
//        Servo vertClaw = hardwareMap.get(Servo.class, "vertclaw");
//        Servo vertwrist = hardwareMap.get(Servo.class, "vertwrist");
//        Servo horext = hardwareMap.get(Servo.class, "horext");
//        Servo vertpivot = hardwareMap.get(Servo.class, "vertpivot");

/*
        vert.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert.setDirection(DcMotorSimple.Direction.REVERSE);


        int targetVertPos = 0;
        double UpPower = 0.4;
        double DownPower = 0.1;
        double CurrPower = 0.0;
        vert.setTargetPosition(targetVertPos);
        vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vert2.setTargetPosition(targetVertPos);
        vert2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vert.setPower(1);
        vert2.setPower(1);
*/
        while (opModeIsActive()) {
/*
            telemetry.addData("slide Pos", vert.getCurrentPosition());
            telemetry.addData("slide pos 2", vert2.getCurrentPosition());
            telemetry.addData("slide Pow", vert.getPower());
            telemetry.addData("targetSlides", vert.getTargetPosition());
            telemetry.update();
*/
            driveTrain.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);
/*
            if (gamepad1.dpad_up) {
                vertClaw.setPosition(0.8);
            } else if (gamepad1.dpad_down) {
                vertClaw.setPosition(0.2);
            }

            //checks to see if the arm is up. Then brings it down or takes it down.
            //programs B button for arm
            if (gamepad1.b) {
                // takes it to the side wall height
                targetVertPos = slideWallPos;
            } else if (gamepad1.a) {
                // above top bar
                targetVertPos = topBarSlidePos;
                vertClaw.setPosition(0.8);
            } else if (gamepad1.x) {
                // below top bar
                targetVertPos = topBarSlidePosDown;
                vertClaw.setPosition(0.8);
            } else if (gamepad1.y) {
                // bottom
                targetVertPos = 0;

            }

            if (gamepad1.right_trigger >= 0.3) {
                // TODO: figure out what max slide position is
                targetVertPos = Math.min(vert.getCurrentPosition() + 200, maxSlidesPos);
                CurrPower = UpPower; // TODO: Someone correct me if I am wrong
            } else if (gamepad1.left_trigger >= 0.3) {
                targetVertPos = Math.max(vert.getCurrentPosition() - 200, 0);
                CurrPower = DownPower;
            }

            vert.setTargetPosition(targetVertPos);
            vert2.setTargetPosition(targetVertPos);
            */

        }
    }
}


