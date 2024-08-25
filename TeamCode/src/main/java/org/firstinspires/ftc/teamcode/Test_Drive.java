package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Test_Drive")
public class Test_Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        driveTrain myTrain = new driveTrain(hardwareMap, driveTrain.Reverse.RevLeft,new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)
        ),(x)->{return x*x*x/Math.abs(x);});
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
               myTrain.update(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,false);
            }
        }
    }
}