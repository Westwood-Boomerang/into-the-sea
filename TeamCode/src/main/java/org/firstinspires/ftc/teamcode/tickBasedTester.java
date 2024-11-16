package org.firstinspires.ftc.teamcode;
import java.util.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@Autonomous
public class tickBasedTester extends LinearOpMode {
    final double tpr = ((((1+((double)46/17))) * (1+((double)46/11))) * 28);
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo claw;
    ElapsedTime time;
    ElapsedTime runtime;
    Telemetry tel;
    @Override
    public void runOpMode() throws InterruptedException {
        tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        time = new ElapsedTime();
        runtime = new ElapsedTime();
        // horizontal encoder
        //parEnc = hardwareMap.get(Encoder.class, "frontLeft");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        // vetical encoder
        //perEnc = hardwareMap.get(Encoder.class, "backRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        vert = hardwareMap.get(DcMotorEx.class, "Vert");
        vert2 = hardwareMap.get(DcMotorEx.class, "Vert2");
        claw = hardwareMap.get(Servo.class, "vertclaw");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        waitForStart();
        vert.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //vert.setDirection(DcMotorSimple.Direction.REVERSE);


        vert.setPower(0);
        vert2.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            tel.addData("fl", fl.getCurrentPosition() / tpr);
            tel.addData("fr", fr.getCurrentPosition() / tpr);
            tel.addData("bl", bl.getCurrentPosition() / tpr);
            tel.addData("br", br.getCurrentPosition() / tpr);
            tel.addData("Vert1", vert.getCurrentPosition());
            tel.addData("Vert2", vert2.getCurrentPosition());
            tel.update();
        }
    }
}