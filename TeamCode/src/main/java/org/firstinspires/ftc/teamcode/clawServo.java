package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawServo {
    Servo claw;
    clawServo(HardwareMap hardwareMap, boolean reverse, double rangeMin, double rangeMax){
        claw = hardwareMap.get(Servo.class, "Claw");
        claw.scaleRange(rangeMin, rangeMax);
        if(reverse){
            claw.setDirection(Servo.Direction.REVERSE);
        }
    }
    clawServo(HardwareMap hardwareMap, boolean reverse){
        this(hardwareMap, reverse, 0.0, 1.0);
    }
    clawServo(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }
    void update(boolean open, boolean close){
        if(open){
            claw.setPosition(1.0);
        } else if (close){
            claw.setPosition(0.0);
        }
    }
}
