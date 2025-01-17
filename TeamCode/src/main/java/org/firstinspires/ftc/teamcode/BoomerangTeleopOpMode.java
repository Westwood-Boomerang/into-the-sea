package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

enum CurrentState {
    Base,
    //ExtendoExtending,
    AlignClawGoldilocks,
    AlignClawBase,
    ExtendoExtending,
    ExtendoRetracting,
    ExtendoOut,
    SlidesMove,
    SlidesTop,
    TransferAlignStart,
    TransferDrop,
    TransferAlignEnd
}

@TeleOp(name = "BoomerangTeleop")
@Config
public class BoomerangTeleopOpMode extends OpMode {
    public static int slidesTop = 2800;
    public static double slidesPower = 0.5;
    public static int slideMultiplier = 30;

    DriveTrain driveTrain;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo bucket1;
    Servo bucket2;
    Servo specClaw;
    Servo extendo;
    Servo extClaw;
    Servo wrist1;
    Servo wrist2;
    Servo wrist3;
    Telemetry t;
    ElapsedTime time;
    ElapsedTime debounce;

    boolean bucketOut = false;
    boolean extClawOpen = false;
    boolean specClawOpen = false;

    double currentWristOffset = 0;
    int targetVertPos = 0;

    CurrentState state = CurrentState.Base;
    @Override
    public void init() {
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        time = new ElapsedTime();
        debounce = new ElapsedTime();
        //init drivetrain
        try {
            driveTrain = new DriveTrain(hardwareMap,
                    new String[]{"frontRight", "frontLeft", "backRight", "backLeft"},
                    org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Reverse.RevRight,
                    "imu",
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                            )
                    )
            );
        } catch (Exception err) {
            t.addLine("Failed to instantiate drivetrain. Error message: " + err.getMessage());
        }
        try {
            vert = hardwareMap.get(DcMotorEx.class, "Vert");
            vert2 = hardwareMap.get(DcMotorEx.class, "Vert2");
            vert.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vert2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vert.setDirection(DcMotorSimple.Direction.REVERSE);
            vert2.setDirection(DcMotorSimple.Direction.REVERSE);
            //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
            //slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
            vert.setTargetPosition(0);
            vert2.setTargetPosition(0);
            vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vert2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vert2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } catch (Exception err) {
            t.addLine("Failed to instantiate vertical slides. Error Message: " + err.getMessage());
        }
        try {
            bucket1 = hardwareMap.get(Servo.class, "bucket1");
            bucket2 = hardwareMap.get(Servo.class, "bucket2");
            bucket1.scaleRange(0,0.8);
            bucket2.scaleRange(0,0.5);
            //bucket1.setDirection(Servo.Direction.REVERSE);
            //bucket2.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err) {
            t.addLine("Failed to instantiate bucket servos. Error Message: " + err.getMessage());
        }

        try {
            specClaw = hardwareMap.get(Servo.class, "vertClaw");
            extClaw = hardwareMap.get(Servo.class, "extClaw");
            extClaw.scaleRange(0.45,0.55);
            //claw.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err){
            t.addLine("Failed in instantiate the claws. Error Message: " + err.getMessage());
        }

       try {
           //extendo = hardwareMap.get(Servo.class, "extendoLeft");
           extendo = hardwareMap.get(Servo.class, "extendoRight");

       } catch (Exception err){
           t.addLine("Failed to instantiate extendo. Error message: " + err.getMessage());
       }
        try {
            //extendo = hardwareMap.get(Servo.class, "extendo");
            wrist1 = hardwareMap.get(Servo.class, "wrist1");
            wrist2 = hardwareMap.get(Servo.class, "wrist2");
            wrist3 = hardwareMap.get(Servo.class, "wrist3");
        } catch (Exception err){
            t.addLine("Failed to instantiate the wrist. Error message: " + err.getMessage());
        }
        t.update();

        extendo.setPosition(0);
    }

    @Override
    public void loop() {
        t.addData("state", state);
        t.addData("vert1", vert.getCurrentPosition());
        t.addData("vert2", vert2.getCurrentPosition());
        t.addData("targetVert1", vert.getTargetPosition());
        t.addData("targetVert2", vert2.getTargetPosition());
        t.update();

        driveTrain.update(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);

        if (gamepad1.b && debounce.milliseconds() > 500) {
            specClawOpen = !specClawOpen;
            specClaw.setPosition(specClawOpen ? 0 : 1);
            debounce.reset();
        }

        if (gamepad2.b && debounce.milliseconds() > 500) {
            extClawOpen = !extClawOpen;
            extClaw.setPosition(extClawOpen ? 0 : 1);
            debounce.reset();
        }

        if (gamepad2.y && debounce.milliseconds() > 500) {
            bucketOut = !bucketOut;
            bucket1.setPosition(bucketOut ? 0 : 1);
            bucket2.setPosition(bucketOut ? 1 : 0);
            debounce.reset();
        }


        switch (state) {
            case Base:
                wrist2.setPosition(1);
                wrist1.setPosition(0);
                if (gamepad1.right_trigger > 0.3) {
                    extendo.setPosition(0.6);
                    state = CurrentState.SlidesMove;
                }
                if (gamepad1.right_bumper) {
                    extendo.setPosition(0.7);
                    state = CurrentState.ExtendoExtending;
                }
                break;
            case ExtendoExtending:
                if (time.milliseconds() >= 500) {
                    time.reset();
                    wrist2.setPosition(0.3);
                    wrist1.setPosition(0.7);
                    state = CurrentState.AlignClawGoldilocks;
                }
                break;
            case ExtendoRetracting:
                if (time.milliseconds() >= 1500) {
                    // AUTOMATIC: open claw
                    extClawOpen = true;
                    extClaw.setPosition(1);
                    state = CurrentState.Base;
                }
                break;
            case AlignClawGoldilocks:
                if (time.milliseconds() >= 500) {
                    state = CurrentState.ExtendoOut;
                    currentWristOffset = 0;
                }
                break;
            case AlignClawBase:
                wrist2.setPosition(1);
                wrist1.setPosition(0);
                if (time.milliseconds() > 500) {
                    time.reset();
                    extendo.setPosition(0.6);
                    state = CurrentState.ExtendoRetracting;
                }
                break;
            case ExtendoOut:
                if (gamepad1.left_bumper) {
                    time.reset();
                    state = CurrentState.AlignClawBase;
                }
                currentWristOffset = 0.3 * gamepad2.right_trigger;

                wrist1.setPosition(0.7 + currentWristOffset);
                wrist2.setPosition(0.3 - currentWristOffset);
                break;
            case SlidesMove:
                vert.setPower(slidesPower);
                vert2.setPower(slidesPower);
                if (gamepad1.right_trigger > 0.3) {
                    targetVertPos = Math.min(vert.getCurrentPosition() + slideMultiplier, slidesTop);
                } else if (gamepad1.left_trigger > 0.3) {
                    targetVertPos = Math.max(vert.getCurrentPosition() - slideMultiplier, 0);
                } else if (vert.getCurrentPosition() < 10) {
                    state = CurrentState.Base;
                } else if (vert.getCurrentPosition() < slidesTop / 2 && bucketOut) {
                    bucketOut = false;
                    bucket1.setPosition(1);
                    bucket2.setPosition(0);
                } else if (vert.getCurrentPosition() > slidesTop - 50) {
                    if (!bucketOut) {
                        bucketOut = true;
                        bucket1.setPosition(0);
                        bucket2.setPosition(1);
                    }
                    state = CurrentState.SlidesTop;
                }
                vert.setTargetPosition(targetVertPos);
                vert2.setTargetPosition(targetVertPos);
                break;
            case SlidesTop:
                if (gamepad1.left_trigger > 0.3) {
                    state = CurrentState.SlidesMove;
                }
                break;
        }
    }


}
