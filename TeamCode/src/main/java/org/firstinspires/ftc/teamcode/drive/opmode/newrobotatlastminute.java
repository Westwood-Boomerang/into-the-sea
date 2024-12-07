package org.firstinspires.ftc.teamcode.drive.opmode;

import android.provider.ContactsContract;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name = "newrobotatlastminute")
public class newrobotatlastminute extends OpMode {
    DcMotorEx FrontRight, FrontLeft, BackRight;
    CRServo BackLeft;
    IMU imu;
    DcMotorEx vert;
    Servo claw, arm;
    int ticksToGo;

    @Override
    public void init() {
        try {
            FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
            FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
            BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
            BackLeft = hardwareMap.get(CRServo.class, "BackLeft");

            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            //FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            //BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                    )
            ));


        } catch (
                Exception err) {
            telemetry.addLine("Failed to instantiate drivetrain. Error message: " + err.getMessage());
        }
        try {
            vert = hardwareMap.get(DcMotorEx.class, "Vert");
        } catch (
                Exception err) {
            telemetry.addLine("Failed to instantiate slides. Error message: " + err.getMessage());
        }

        try {
            arm = hardwareMap.get(Servo.class, "Arm");
            claw = hardwareMap.get(Servo.class, "Claw");
        } catch (
                Exception err) {
            telemetry.addLine("Failed to instantiate arm/claw. Error message" + err.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        dt();
        arms();
        telemetry.addData("vert pos", vert.getCurrentPosition());
        telemetry.update();
    }

    private void arms() {
        if (gamepad1.dpad_up) {
            //go up
            ticksToGo = 1000;
            vert.setPower(0.4);

        } else if (gamepad1.dpad_down) {
            //go down
            ticksToGo = 0;
            vert.setPower(-0.4);
        } else {
            if (Math.abs(vert.getTargetPosition() - ticksToGo) < 15) {
                //TODO
            }
        }
        telemetry.addData("vert pos", vert.getCurrentPosition());
        telemetry.addData("vert tar", vert.getTargetPosition());
        vert.setTargetPosition(ticksToGo);
        vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (gamepad1.x) {
            arm.setPosition(1);
        } else if (gamepad1.y) {
            arm.setPosition(0);
        }


        if (gamepad1.a) {
            claw.setPosition(1);
        } else if (gamepad1.b) {
            claw.setPosition(0);
        }
    }

    private void dt() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = strafe * Math.cos(-botHeading) - -forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + -forward * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        //double denominator = 1;
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        FrontLeft.setPower(frontLeftPower);
        BackLeft.setPower(backLeftPower);
        FrontRight.setPower(frontRightPower);
        BackRight.setPower(backRightPower);

    }
}