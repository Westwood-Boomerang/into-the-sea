package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(preselectTeleOp = "BoomerangTeleopOpMode")
public class tickBasedRed extends LinearOpMode {
    final double tpr = ((((1+((double)46/17))) * (1+((double)46/11))) * 28);
//    public static double flpos = -0.4371;
//    public static double frpos = -2.7711;
//    public static double blpos = 2.9562;
//    public static double brpos = -6.2285;
    public static double flpos = -1.5;
    public static double frpos = -1.3;
    public static double blpos = -1.5;
    public static double brpos = -1.3;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo claw;
    ElapsedTime time;
    Telemetry tel;
    @Override
    public void runOpMode() throws InterruptedException {
        tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        time = new ElapsedTime();
        time = new ElapsedTime();
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
        claw = hardwareMap.get(Servo.class, "vertClaw");
        vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        waitForStart();
        vert.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert.setDirection(DcMotorSimple.Direction.REVERSE);
        vert2.setDirection(DcMotorSimple.Direction.REVERSE);

        int targetVertPos = 0;
        vert.setTargetPosition(targetVertPos);
        vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vert2.setTargetPosition(targetVertPos);
        vert2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vert.setPower(0.5);
        vert2.setPower(0.5);
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            int openPos = 0;
            int closePos = 1;
            hardwareMap.get(Servo.class, "extendoRight").setPosition(0);
            claw.setPosition(closePos);
            //slides up
            drive(0, 0, 0, 0, 1700, false, 1000);
            //go back for spec 1
            drive(-1.5, -1.5, -1.5, -1.5, 1700, false, 2000);
            //slides go down to clip spec 1
            drive(0, 0, 0, 0, 1300, true, 1000);
            //go right to get spec
            drive(-3.5, 3.5, 3.5, -3.5, 300, true, 3000);
            //spin 180 degrees
            claw.setPosition(openPos);
            drive(2.5, -2.5, 2.5, -2.5, 300, false, 3000);
            //go to back wall to pick up spec
            drive(-0.8, -0.8, -0.8, -0.8, 300, true, 3000);
            claw.setPosition(closePos);
            //strafe left
            drive(-2.5, 2.5, 2.5, -2.5, 1700, false, 3000);
            //go backwards toward hang
            drive(0.75, 0.75, 0.75, 0.75, 1700, false, 3000);
            //turn 180 to have vert claw face the submersible
            drive(-2.4, 2.4, -2.4, 2.4, 1700, false, 3000);
            //go backwards toward submersible (again)
            drive(-0.8, -0.8, -0.8, -0.8, 1800, false, 3000);
            //slides go down to place
            drive(0, 0, 0, 0, 1350, true, 3000);
            claw.setPosition(openPos);
            drive(0, 0, 0, 0, 0, true, 1000);
        }
    }

    public void drive(double flpos, double frpos, double blpos, double brpos, int slidePos, boolean clawOpen,  double t) {
        time.reset();
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive() && time.milliseconds() < t)
        {
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
            vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vert2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            fl.setPower(0.75);
            fr.setPower(0.75);
            bl.setPower(0.75);
            br.setPower(0.75);
            vert.setPower(0.85);
            vert2.setPower(0.85);
            tel.addData("time", time.time());
            tel.addData("flpos", fl.getCurrentPosition() / tpr);
            tel.addData("blpos", bl.getCurrentPosition() / tpr);
            tel.addData("frpos", fr.getCurrentPosition() / tpr);
            tel.addData("brpos", br.getCurrentPosition() / tpr);
            tel.addData("claw", claw.getPosition());
            tel.addData("slide pos", vert.getCurrentPosition());
            tel.addData("slide2 pos", vert2.getCurrentPosition());
            tel.addData("slides target pos", vert.getTargetPosition());
            tel.update();
        }

        if (clawOpen) {
            claw.setPosition(0);
        } else {
            claw.setPosition(1);
        }
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        vert.setPower(0.1);
        vert2.setPower(0.1);
        time.reset();
    }
}
