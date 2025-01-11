package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name = "boomerangTeleopOpMode")
public class boomerangTeleopOpMode extends OpMode {
    DriveTrain driveTrain;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo bucket1;
    Servo bucket2;
    Servo claw;
    Servo rotater; //Send help
    Servo axonrotater; // I need help
    Servo extendo1;
    Servo extendo2;
    @Override
    public void init() {
        //init drivetrain
        try {
            driveTrain = new DriveTrain(hardwareMap,
                    new String[]{"frontRight", "frontLeft", "backRight", "backLeft"},
                    org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Reverse.RevLeft,
                    "imu",
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                            )
                    )
            );
        } catch (Exception err) {
            telemetry.addLine("Failed to instantiate drivetrain. Error message: " + err.getMessage());
        }
        try {
            vert = hardwareMap.get(DcMotorEx.class, "Vert");
            vert2 = hardwareMap.get(DcMotorEx.class, "Vert2");
            vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
            //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
        } catch (Exception err) {
            telemetry.addLine("Failed to instantiate vertical slides. Error Message: " + err.getMessage());
        }
        try {
            bucket1 = hardwareMap.get(Servo.class, "Bucket1");
            bucket2 = hardwareMap.get(Servo.class, "Bucket2");
            //bucket1.setDirection(Servo.Direction.REVERSE);
            //bucket2.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err) {
            telemetry.addLine("Failed to instantiate bucket servos. Error Message: " + err.getMessage());
        }

        try {
            claw = hardwareMap.get(Servo.class, "claw");
            //claw.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err){
            telemetry.addLine("Failed in instantiate the claw. Error Message: " + err.getMessage());
        }

        try {
            rotater = hardwareMap.get(Servo.class, "rotater");
            axonrotater = hardwareMap.get(Servo.class, "axonrotater"); //Jihoon wasn't here so pray this works otherwise we cooked frfr
        } catch (Exception err){
            telemetry.addLine("Failed to instantiate the rotaters. Error message: " + err.getMessage());
        }

       try {
           extendo1 = hardwareMap.get(Servo.class, "extendo1");
           extendo2 = hardwareMap.get(Servo.class, "extendo2");

       } catch (Exception err){
           telemetry.addLine("Failed to instantiate the extendos. Error message: " + err.getMessage());
       }
        telemetry.update();
    }

    @Override
    public void loop() {

        driveTrain.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);

    }


}
