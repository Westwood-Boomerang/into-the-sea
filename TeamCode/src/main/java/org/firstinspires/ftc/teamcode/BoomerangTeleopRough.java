package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/*
Notes:
    Target Positions are currently estimates... need telemetry for exact positions
    Need telemetry to determine max position for slides
 */
/*
Controller Functions:
    Left stick y - drive
    Left stick x - strafe
    Right stick x - turn
    A - claw open/close
    B - reaches position for picking up samples
    X - drops/recovers from dropping sample in bucket
    Left Trigger - Moves slides down
    Right Trigger - Moves slides up
 */
@TeleOp
//boomerangTeleOp is a subclass of class OpMode(Inheritance)
public class BoomerangTeleopRough extends OpMode{

    //initializing motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    DcMotorEx arm;
    DcMotorEx slides;
    Servo claw;
    Servo wrist;

    //initializing variables
    boolean arm_up = false;
    boolean claw_up = false;
    boolean sample = true;

    @Override
    public void init(){
        //maps motors to driver hub
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        slides = hardwareMap.get(DcMotorEx.class, "Slides");

        claw = hardwareMap.get(Servo.class, "Claw");
        wrist = hardwareMap.get(Servo.class, "Wrist");


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
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //coordinates the wheels
        frontLeft.setPower(drive + strafe + turn);
        frontRight.setPower(drive - strafe - turn);
        backLeft.setPower(drive - strafe + turn);
        backRight.setPower(drive + strafe - turn);

        int lift = slides.getCurrentPosition();

        //programs A button for claw
        if (gamepad1.a){
            if (!claw_up) {
                claw.setPosition(1);
                claw_up = true;
            }
            else{
                claw.setPosition(0);
                claw_up = false;
            }
        }

        //checks to see if the arm is up. Then brings it down or takes it down.
        //programs B button for arm
        if (gamepad1.b){
            if (!arm_up){
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(500);
                arm.setPower(0.5);
                wrist.setPosition(1);
                claw.setPosition(1);
                arm_up = true;
                claw_up = true;
            }
            else if (arm_up){
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(0);
                arm.setPower(0.5);
                wrist.setPosition(0.5);
                arm_up = false;
            }
        }

        if (gamepad1.x) {
            if (sample) {
                //arm.setDirection(DcMotorEx.Direction.REVERSE);
                arm.setPower(-0.5);
                arm.setTargetPosition(500);
                wrist.setPosition(0);}
            else {
                arm.setPower(0.5);
                arm.setTargetPosition(0);
                wrist.setPosition(0.5);
                claw.setPosition(0);
            }
        }
        if (gamepad1.y){
            if (!arm_up){
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                arm.setTargetPosition(500);
                arm_up = true;
            }
            else {
                arm.setPower(0.5);
                arm.setTargetPosition(0);
                arm_up = false;
            }
        }
        //keeps the slides while left trigger is not pressed
        if (gamepad1.right_trigger <= 0.3){
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setTargetPosition(lift);
        }
        //moves the slides up while right trigger is pressed
        else {
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            if (slides.getCurrentPosition() < 495) {
                slides.setPower(0.5);
                lift = slides.getCurrentPosition();
            }
            else{
                slides.setPower(-0.01);
                slides.setTargetPosition(495);
            }
        }
        //keeps the slides while left trigger is not pressed
        if (gamepad1.left_trigger <= 0.3){
            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slides.setTargetPosition(lift);
        }
        //moves the slides down while left trigger is pressed
        else {

            slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            if (slides.getCurrentPosition() > 0) {
            slides.setPower(-0.5);
               lift = slides.getCurrentPosition();
            }
                  else {
                slides.setPower(0.01);
                slides.setTargetPosition(0);
         }
      }

    }
}


