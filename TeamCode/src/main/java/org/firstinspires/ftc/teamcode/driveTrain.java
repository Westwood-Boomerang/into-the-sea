package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class driveTrain {

    private DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;
    private IMU imu;
    interface ScalarInterface {
        double scale(double x);
    }
    ScalarInterface scaleFn;
    public enum Reverse {
        RevLeft,
        RevRight,
        None
    }
    public driveTrain(HardwareMap hardwareMap, Reverse reversal, IMU.Parameters params, ScalarInterface scalar){
        FrontRight  =   hardwareMap.get(DcMotorEx.class, "Forward_Right");
        FrontLeft   =   hardwareMap.get(DcMotorEx.class, "Forward_Left");
        BackRight   =   hardwareMap.get(DcMotorEx.class, "Backward_Right");
        BackLeft    =   hardwareMap.get(DcMotorEx.class, "Backward_Left");

        if(reversal == Reverse.RevLeft){
            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(reversal == Reverse.RevRight){
            FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            //Nothing!
        }
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);
        scaleFn = scalar;
    }
    public driveTrain(HardwareMap hardwareMap, Reverse reversal, IMU.Parameters params){
        this(hardwareMap, reversal, params, (x) -> {return x;});
    }
    public driveTrain(HardwareMap hardwareMap, Reverse reversal) {
        this(hardwareMap, reversal,
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
                )
        );

    }
    public driveTrain(HardwareMap hardwareMap){
        this(hardwareMap, Reverse.RevLeft);
    }
    void update(double forward, double strafe, double turn, boolean resetIMU) {

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (resetIMU) {
            imu.resetYaw();
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = strafe * Math.cos(-botHeading) - -forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + -forward * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        //double denominator = Math.max(Math.abs(scaleFn.scale(rotY)) + Math.abs(scaleFn.scale(rotX)) + Math.abs(scaleFn.scale(turn)), 1);
        double denominator = 1;
        double frontLeftPower   =   scaleFn.scale(rotY + rotX + turn) / denominator;
        double backLeftPower    =   scaleFn.scale(rotY - rotX + turn) / denominator;
        double frontRightPower  =   scaleFn.scale(rotY - rotX - turn) / denominator;
        double backRightPower   =   scaleFn.scale(rotY + rotX - turn) / denominator;

        FrontLeft.setPower(frontLeftPower);
        BackLeft.setPower(backLeftPower);
        FrontRight.setPower(frontRightPower);
        BackRight.setPower(backRightPower);
    }

}
