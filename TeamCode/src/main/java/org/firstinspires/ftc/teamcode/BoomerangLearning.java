package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//BoomerangLearning is a subclass of class OpMode(Inheritance)
public class BoomerangLearning extends OpMode{

    //initializing motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    DcMotorEx arm;
    DcMotorEx slides;
    Servo claw;

    @Override
    public void init(){
        //maps motors to driver hub
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        slides = hardwareMap.get(DcMotorEx.class, "Slides");

        claw = hardwareMap.get(Servo.class, "Claw");

        //Tells the different parts how they should act in certain scenarios
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //reverses direction of motor(SUBJECT TO CHANGE)
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
        slides.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void loop(){
        //calibrates the parts of the robot with the controller.
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //coordinates the wheels
        frontLeft.setPower(drive + strafe + turn);
        frontRight.setPower(drive - strafe - turn);
        backLeft.setPower(drive - strafe + turn);
        backRight.setPower(drive + strafe - turn);

        //initializing variables
        boolean arm_up = false;
        int lift = slides.getCurrentPosition();
        boolean claw_up = false;

        //programs A button for claw
        if (gamepad2.a){
            if (claw_up) {
                claw.setPosition(1);
            }
            else{
                claw.setPosition(0);
            }
        }

        //checks to see if the arm is up. Then brings it down or takes it down.
        //programs B button for arm
        if (gamepad2.b){
            if (arm_up==false){
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(500);
                arm.setPower(0.5);
                arm_up = true;
            }
            else if (arm_up==true){
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(0);
                arm.setPower(0.5);
                arm_up = false;
            }
        }
        //keeps the slides while left trigger is not pressed
        if (gamepad2.right_trigger <= 0.3){
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setTargetPosition(lift);
        }
        //moves the slides up while right trigger is pressed
        else {
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setDirection(DcMotorEx.Direction.FORWARD);
            slides.setPower(0.5);
            lift = slides.getCurrentPosition();

        }
        //keeps the slides while left trigger is not pressed
        if (gamepad2.left_trigger <= 0.3){
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setTargetPosition(lift);
        }
        //moves the slides down while left trigger is pressed
        else {
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setDirection(DcMotorEx.Direction.REVERSE);
            slides.setPower(0.5);
            lift = slides.getCurrentPosition();

        }

    }
}


