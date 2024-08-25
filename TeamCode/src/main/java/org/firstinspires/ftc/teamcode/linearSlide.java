package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
public class linearSlide {
    private DcMotorEx slider;
    private int[] StoppingPoints;
    PIDFcontroller controller;
    private int Pointer; //A POINTer to the stopping POINTS

    public linearSlide(HardwareMap hardwareMap, int[] points, int currIndex, PIDFcontroller PID){
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        StoppingPoints = points;
        Arrays.sort(StoppingPoints); // Allows us to use ++ and -- to move through the points
        controller = PID;
        Pointer = currIndex;
    }
    public void update(boolean up, boolean down){
        if(up) {
            controller.CalculateAsnyc(StoppingPoints[++Pointer], slider.getCurrentPosition());
        } else if (down){
            controller.CalculateAsnyc(StoppingPoints[--Pointer], slider.getCurrentPosition());
        }
    }
}
