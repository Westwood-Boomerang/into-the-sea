package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(group = "drive")
public class BoomAutonTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // init drivetrain
        DcMotorEx  fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx  fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx  br = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();

        while (opModeIsActive()) {
            fl.setPower(1);
            sleep(1000);
            fl.setPower(0);
            fr.setPower(1);
            sleep(1000);
            fr.setPower(0);
            bl.setPower(1);
            sleep(1000);
            bl.setPower(0);
            br.setPower(1);
            sleep(1000);
            br.setPower(0);

        }
    }
}
