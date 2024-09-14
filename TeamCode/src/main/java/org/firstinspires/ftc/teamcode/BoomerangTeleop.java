package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                DriveTrain.update(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x, gamepad1.start);
                // code goes here!,
            }
        }
    }
}