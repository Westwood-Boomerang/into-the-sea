package org.firstinspires.ftc.teamcode;

import java.util.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class tickBasedRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

        }
    }
}