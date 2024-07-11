package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
public class linearSlide {
    private DcMotorEx slider;
    private int[] StoppingPoints;
    private int PointEr; //A POINTer to the stopping POINTS (see what I did there...?) (not rly a pointer but I wanted to make the joke)

    public linearSlide(HardwareMap hardwareMap, int[] points, int currIndex){
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        StoppingPoints = points;
        Arrays.sort(StoppingPoints); // Allows us to use ++ and -- to move through the points

    }
}
