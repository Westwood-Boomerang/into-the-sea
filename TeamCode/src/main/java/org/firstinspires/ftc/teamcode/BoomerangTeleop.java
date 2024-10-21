package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "BoomerangTeleop")
public class BoomerangTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        //init drivetrain
        DriveTrain driveTrain = new DriveTrain(hardwareMap,
                new String[]{"frontRight", "frontLeft", "backRight", "backLeft"},
                org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Reverse.RevLeft,
                "imu",
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                ),
                x -> x
        );

        waitForStart();



        while (opModeIsActive()) {
            driveTrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);
        }
    }
}


