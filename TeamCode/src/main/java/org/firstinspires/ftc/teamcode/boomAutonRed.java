package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class boomAutonRed extends LinearOpMode{
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx arm;
    DcMotorEx slides;
    Servo claw;
    Servo wrist;
    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");


        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -50, Math.toRadians(90));



        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                //go to bar
                .splineTo(new Vector2d(0, -38), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                            slides.setPower(0.8);
                            slides.setTargetPosition(250);
                        }
                )
                .lineTo(new Vector2d(0, -34))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.5);
                    slides.setTargetPosition(225);
                    claw.setPosition(1);
                })
                .lineTo(new Vector2d(0, -52))
                //go to sample
                .lineToSplineHeading(new Pose2d(48, -37, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.5);
                    slides.setTargetPosition(0);
                    claw.setPosition(0);
                })
                //go to bucket
                .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.8);
                    slides.setTargetPosition(500);
                    wrist.setPosition(1);
                    claw.setPosition(1);
                })
                //go to sample
                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.5);
                    slides.setTargetPosition(0);
                    claw.setPosition(0);
                })
                //go to bucket
                .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.8);
                    slides.setTargetPosition(500);
                    wrist.setPosition(1);
                    claw.setPosition(1);
                })
                //go to sample
                .lineToSplineHeading(new Pose2d(58, -25, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.5);
                    slides.setTargetPosition(0);
                    claw.setPosition(0);
                })
                //go to bucket
                .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.8);
                    slides.setTargetPosition(500);
                    wrist.setPosition(1);
                    claw.setPosition(1);
                })
                .lineToSplineHeading(new Pose2d(60, -58, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    slides.setPower(0.5);
                    slides.setTargetPosition(0);
                    claw.setPosition(0);
                })
                .build();


        drive.followTrajectorySequence(traj1);
    }
}
