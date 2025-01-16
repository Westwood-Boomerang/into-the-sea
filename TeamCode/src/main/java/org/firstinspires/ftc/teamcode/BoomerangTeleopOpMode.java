package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
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
public class BoomerangTeleopOpMode extends OpMode {
    public static int slidesTop = 2800;
    public static double slidesPower = 0.75;
    public static double wrist1In = 0;
    public static double wrist1Out = 0.7;
    public static double wrist2In = 1;
    public static double wrist2Out = 0.3;
    public static double extendoIn = 0;
    public static double extendoOut = 0.7;
    public static double extendoMid = 0.6;
    public static double bucket1In = 0.8;
    public static double bucket1Out = 0.0;
    public static double bucket2In = 0.0;
    public static double bucket2Out = 0.5;

    DriveTrain driveTrain;
    DcMotorEx vert;
    DcMotorEx vert2;
    Servo bucket1;
    Servo bucket2;
    Servo specClaw;
    Servo rotater; //Send help
    Servo axonrotater; // I need help
    Servo extendo1;
    Servo extendo2;
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
            vert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vert2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception err) {
            t.addLine("Failed to instantiate vertical slides. Error Message: " + err.getMessage());
        }
        try {
            bucket1 = hardwareMap.get(Servo.class, "bucket1");
            bucket2 = hardwareMap.get(Servo.class, "bucket2");
            //bucket1.setDirection(Servo.Direction.REVERSE);
            //bucket2.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err) {
            t.addLine("Failed to instantiate bucket servos. Error Message: " + err.getMessage());
        }

        try {
            specClaw = hardwareMap.get(Servo.class, "vertClaw");
            extClaw = hardwareMap.get(Servo.class, "extClaw");
            //claw.setDirection(Servo.Direction.REVERSE);
        } catch (Exception err){
            t.addLine("Failed in instantiate the claws. Error Message: " + err.getMessage());
        }

       try {
           //extendo1 = hardwareMap.get(Servo.class, "extendoLeft");
           extendo2 = hardwareMap.get(Servo.class, "extendoRight");

       } catch (Exception err){
           t.addLine("Failed to instantiate extendo. Error message: " + err.getMessage());
       }
        try {
            //extendo1 = hardwareMap.get(Servo.class, "extendo1");
            wrist1 = hardwareMap.get(Servo.class, "wrist1");
            wrist2 = hardwareMap.get(Servo.class, "wrist2");
            wrist3 = hardwareMap.get(Servo.class, "wrist3");
        } catch (Exception err){
            t.addLine("Failed to instantiate the wrist. Error message: " + err.getMessage());
        }
        t.update();

        extendo2.setPosition(0);
    }

    @Override
    public void loop() {
        t.addData("state", state);
        t.addData("vert1", vert.getCurrentPosition());
        t.addData("vert2", vert2.getCurrentPosition());
        t.addData("targetVert1", vert.getTargetPosition());
        t.addData("targetVert2", vert2.getTargetPosition());
        t.update();

        driveTrain.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.start);

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
            bucket1.setPosition(bucketOut ? bucket1Out : bucket1In);
            bucket2.setPosition(bucketOut ? bucket2Out : bucket2In);
            debounce.reset();
        }


        switch (state) {
            case Base:
                wrist2.setPosition(wrist2In);
                wrist1.setPosition(wrist1In);
                if (gamepad1.right_trigger > 0.3) {
                    extendo2.setPosition(extendoMid);
                    state = CurrentState.SlidesMove;
                }
                break;
            case ExtendoExtending:
                if (time.milliseconds() >= 500) {
                    time.reset();
                    wrist2.setPosition(wrist2Out);
                    wrist1.setPosition(wrist1Out);
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
                wrist2.setPosition(wrist2In);
                wrist1.setPosition(wrist1In);
                if (time.milliseconds() > 500) {
                    time.reset();
                    extendo2.setPosition(extendoIn);
                    state = CurrentState.ExtendoRetracting;
                }
                break;
            case ExtendoOut:
                if (gamepad1.left_bumper) {
                    time.reset();
                    state = CurrentState.AlignClawBase;
                }
                currentWristOffset = 0.3 * gamepad2.right_trigger;

                wrist1.setPosition(wrist1Out + currentWristOffset);
                wrist2.setPosition(wrist2Out - currentWristOffset);
                break;
            case SlidesMove:
                vert.setPower(0.5);
                vert2.setPower(0.5);
                if (vert.getCurrentPosition() > 2700) {
                    vert.setVelocity(0);
                    vert2.setVelocity(0);

                    if (!bucketOut) {
                        bucketOut = true;
                        bucket1.setPosition(bucket1Out);
                        bucket2.setPosition(bucket2Out);
                    }
                    state = CurrentState.SlidesTop;
                } else if (gamepad1.right_trigger > 0.3 && vert.getCurrentPosition() < 2700) {
                    vert.setVelocity(gamepad1.right_trigger);
                    vert2.setVelocity(gamepad1.right_trigger);
                }
                else if (vert.getCurrentPosition() < 10) {
                    if (bucketOut) {
                        bucketOut = false;
                        bucket1.setPosition(bucket1In);
                        bucket2.setPosition(bucket2In);
                    }

                    vert.setVelocity(0);
                    vert2.setVelocity(0);
                    state = CurrentState.Base;
                    extendo2.setPosition(extendoIn);
                } else if (gamepad1.left_trigger > 0.3 && vert.getCurrentPosition() > 10) {
                    vert.setVelocity(-gamepad1.left_trigger);
                    vert2.setVelocity(-gamepad1.left_trigger);

                    if (vert.getCurrentPosition() < 50) {
                        bucket1.setPosition(bucket1In);
                        bucket2.setPosition(bucket2In);
                    }
                }
                break;
            case SlidesTop:
                if (gamepad1.left_trigger > 0.3) {
                    state = CurrentState.SlidesMove;
                }
                break;
        }
    }


}
