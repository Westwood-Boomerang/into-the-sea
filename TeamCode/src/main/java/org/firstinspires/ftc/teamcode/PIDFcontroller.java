package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDFcontroller {
    private int ErrorChange;
    private double M_Gain;
    private int LastReference;

    private double LastEstimate;
    private double CurrentEstimate;
    private double IntegralSumLimit;
    private ElapsedTime time = new ElapsedTime(0);
    private int LastError = 0;
    private double Integral = 0.0;
    private double Derivative = 0.0;
    private int Error;
    private int position;
    private int Reference;
    private int tolerance = 0;
    private double Kp,Ki,Kd,Kf = 0.0;
    /**
      * The constructor used for all coefficients and tolerance
      * @param p the Proportional coefficient
      * @param i the Integral coefficient
      * @param d the Derivative coefficient
      * @param f an additional term, if necessary
      * @param SumLimit the max Integral sum limit
      * @param toll the allowable difference between the current ticks and the target (Error < tolerance)
      * @param gain the gain of the derivative
      *
     */
            public PIDFcontroller(double p, double i, double d, double f, double SumLimit, int tol, double gain){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        this.Kf = f;
        this.IntegralSumLimit = SumLimit;
        this.tolerance = tol;
        this.M_Gain = gain;
    }
    /**
      * The constructor used for only coefficients
      * @param p the Proportional coefficient
      * @param i the Integral coefficient
      * @param d the Derivative coefficient
      * @param f an additional term, if necessary
      * @param SumLimit the max Integral sum limit
      * @param gain the gain of the derivative
      * Tolerance is set to 100 ticks by default
      */
            public PIDFcontroller(double p, double i, double d,double f, double SumLimit, DcMotorEx motor, double gain){
        this(p,i,d,f,SumLimit,25,gain);
    }


    /**
      * Calculates the Coefficient using the provided P,I,D, and F values in asynchronous
      * @param Target The target position
      *
      */
    public double CalculateAsnyc(int Target,int position){
        Reference = Target;
        this.position = position;
        Error = Reference - position;
        if (Math.abs(Error) > tolerance){
            return CalculateFunc(Target,this.position);
        }
        return 0;
    }
    private double CalculateFunc(int Target, int currentPos){

        Reference = Target;
        position = currentPos;
        Error = Reference - position;

        CurrentEstimate = (M_Gain * LastEstimate) + (1-M_Gain)*ErrorChange;
        LastEstimate = CurrentEstimate;
        ErrorChange = Error-LastError;
        if(LastReference != Reference){
            Integral = 0;
        }

        Integral += (Error * time.seconds());
        Derivative = CurrentEstimate / time.seconds();


        if(Integral>IntegralSumLimit){
            Integral = IntegralSumLimit;
        } else if (Integral < -IntegralSumLimit){
            Integral = -IntegralSumLimit;
        }
        LastReference = Reference;

        double output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative) + (Kf * Reference);
        LastError = Error;
        time.reset();
        return output;
    }

    public void reset(){
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        tolerance = 100;
    }
    public void SetPIDFcoefficents(double p, double i, double d, double f){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        this.Kf = f;
    }

    public void setKd(double kd) {
        Kd = kd;
    }

    public void setKi(double ki) {
        Ki = ki;
    }

    public void setKp(double kp) {
        Kp = kp;
    }

    public void setKf(double kf) {
        Kf = kf;
    }

    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    public double getKd() {
        return Kd;
    }

    public double getKi() {
        return Ki;
    }

    public double getKp() {
        return Kp;
    }

    public double getKf() {
        return Kf;
    }
}