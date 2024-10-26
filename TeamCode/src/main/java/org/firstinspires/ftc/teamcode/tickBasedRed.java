package org.firstinspires.ftc.teamcode;
import java.util.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class tickBasedRed extends LinearOpMode {
    final double tpr = ((((1+((double)46/17))) * (1+((double)46/11))) * 28);
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx arm;
    DcMotorEx slides;
    Servo wrist;
    Servo claw;
    ElapsedTime time;
    ElapsedTime runtime;
    @Override
    public void runOpMode() throws InterruptedException {
        time = new ElapsedTime();
        runtime = new ElapsedTime();
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
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides = hardwareMap.get(DcMotorEx.class, "Slides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        waitForStart();
        if (isStopRequested()) return;
        if (runtime.seconds() < 29.9) {
            if (opModeIsActive()) {
                drive(0.5, 0.5, 0.5, 0.5, 5, 200, true, 1, true,5000);
                drive(5, -5, -5, 5, 0, 1, true, 0, true,5000);


            }
        }
    }

    public void drive(double flpos, double frpos, double blpos, double brpos, double armPos, double slidePos, boolean clawOpen, double wristPos, boolean isSample, double t) {
        while (time.seconds() < t && opModeIsActive()) {
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fl.setTargetPosition((int) (tpr * flpos));
            fr.setTargetPosition((int) (tpr * frpos));
            bl.setTargetPosition((int) (tpr * blpos));
            br.setTargetPosition((int) (tpr * brpos));
            arm.setTargetPosition((int) (tpr * armPos));
            slides.setTargetPosition((int) (tpr * slidePos));


            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            fl.setPower(0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(0.5);

            while (!(arm.getCurrentPosition() == arm.getTargetPosition() &&
                    slides.getCurrentPosition() == slides.getTargetPosition() &&
                    fl.getCurrentPosition() == fl.getTargetPosition() &&
                    isSample && opModeIsActive())){}
            wrist.setPosition(.3);
            claw.setPosition(1);

        }
        time.reset();

    }
}