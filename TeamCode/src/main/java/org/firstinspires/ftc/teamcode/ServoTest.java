package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.robotcore.eventloop.

@TeleOp(name = "ServoTest")
@Config
public class ServoTest extends LinearOpMode {
    public static String servoName = "extendoLeft";
    public static double start = 0;
    public static double end = 1;
    public static boolean testMode = false;
    @Override
    public void runOpMode() {
        waitForStart();
        Servo s = hardwareMap.get(Servo.class, servoName);
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (testMode) {
                    s.setPosition(start);
                    sleep(1000);
                    s.setPosition(end);
                    sleep(1000);
                } else {
                    t.addData("pos", s.getPosition());
                    t.update();
                    sleep(100);
                }
            }
        }
    }
}