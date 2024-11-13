package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;
public class PIDController {

    protected double target;
    protected double kP, kI, kD;
    protected double proportional, integral, derivative;
    protected boolean shouldReset;

    protected double previousTime, previousError;

    protected double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    protected double lowerOutputBound = Double.NEGATIVE_INFINITY, higherOutputBound = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        shouldReset = true;
    }

    public void setPIDValues(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setInputBounds(double lowerInputBound, double higherInputBound) {
        this.lowerInputBound = lowerInputBound;
        this.higherInputBound = higherInputBound;
    }

    public void setOutputBounds(double lowerOutputBound, double higherOutputBound) {
        this.lowerOutputBound = lowerOutputBound;
        this.higherOutputBound = higherOutputBound;
    }

    public double getLowerInputBound(){
        return this.lowerInputBound;
    }

    public double getUpperInputBound(){
        return this.higherInputBound;
    }


    public void reset() {
        shouldReset = true;
    }


    // returns motor power given current position of motor
    public double update(double value) {

        value = Range.clip(value, lowerInputBound, higherInputBound);

        double error = value - target;

        return updateWithError(error);
    }

    private double updateWithError(double error) {
        if (Double.isNaN(error) || Double.isInfinite(error))
            return 0;

        proportional = kP * error;

        double currentTime = System.currentTimeMillis() / 1000.0;

        if (shouldReset) {
            shouldReset = false;
            integral = 0;
            derivative = 0;
            previousError = error;
        } else {
            double dT = currentTime - previousTime;

            integral += kI * error * dT;

            derivative = kD * (error - previousError) / dT;
        }

        previousTime = currentTime;
        previousError = error;

        double correction = proportional + integral + derivative;

        return Math.signum(correction) * Range.clip(Math.abs(correction),
                lowerOutputBound, higherOutputBound);
    }


    // getters/setters
    public double getkP() {
        return kP;
    }
    public double getkI() {
        return kI;
    }
    public double getkD() {
        return kD;
    }
    // the target position of motor
    public double getTarget() {
        return target;
    }
    public void setkP(double kP) {
        this.kP = kP;
    }
    public void setkI(double kI) {
        this.kI = kI;
    }
    public void setkD(double kD) {
        this.kD = kD;
    }
    public void setTarget(double target) {
        this.target = target;
    }
}