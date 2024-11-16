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
public class tickBasedRed extends LinearOpMode {
    final double tpr = ((((1+((double)46/17))) * (1+((double)46/11))) * 28);
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx arm;
    Encoder parEnc;
    Encoder perEnc;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo wrist;
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
        vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        waitForStart();
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
        if (isStopRequested()) return;
        if (runtime.seconds() < 29.9) {
            if (opModeIsActive()) {
                drive(-1.6, -1.6, -1.6, -1.6, 3085, 0.8, 5000);
                drive(-1.8, -1.8, -1.8, -1.8,3085, 0.8, 5000);
                drive(-1.8, -1.8, -1.8, -1.8,2100, 0.8, 5000);
                drive(-1.8, -1.8, -1.8, -1.8, 2100, 0.2, 5000);


                //drive(5, -5, -5, 5, 0, 0, 5000);
                //drive(5, -5, -5, 5, 0, 1, 5000);



            }
        }
    }

    public void drive(double flpos, double frpos, double blpos, double brpos, int slidePos, double clawOpen,  double t) {


        /*/fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        fl.setTargetPosition((int) (tpr * flpos));
        fr.setTargetPosition((int) (tpr * frpos));
        bl.setTargetPosition((int) (tpr * blpos));
        br.setTargetPosition((int) (tpr * brpos));
        vert.setTargetPosition(slidePos);
        vert2.setTargetPosition(slidePos);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        fl.setPower(0.4);
        fr.setPower(0.4);
        bl.setPower(0.4);
        br.setPower(0.4);
        vert.setPower(0.2);
        vert2.setPower(0.2);

        while (
                !(
                        Math.abs(vert.getCurrentPosition() - vert.getTargetPosition()) < 5 &&
                                Math.abs(fl.getCurrentPosition() - fl.getTargetPosition()) < 5
                ) && opModeIsActive())
        {
            claw.setPosition(clawOpen);
            tel.addData("par", fl.getCurrentPosition());
            tel.addData("per", br.getCurrentPosition());
            tel.addData("slide", vert.getCurrentPosition());
            tel.addData("slide2", vert2.getCurrentPosition());
            tel.update();
        }

        time.reset();

    }
}