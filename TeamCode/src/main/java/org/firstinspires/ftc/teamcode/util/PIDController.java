package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;
public class PIDController {

    protected double target;
    public final double KP, KI, KD;
    protected double kP, kI, kD;
    protected double proportional, integral, derivative;
    protected boolean shouldReset;

    protected double previousTime, previousError;

    protected double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    protected double lowerOutputBound = Double.NEGATIVE_INFINITY, higherOutputBound = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        KP = kP;
        KI = kI;
        KD = kD;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        shouldReset = true;
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
        this.kP = KP;
        this.kI = KI;
        this.kD = KD;
    }


    // returns motor power given current position of motor
    public double update(double value) {

        value = Range.clip(value, lowerInputBound, higherInputBound);

        // BEFORE: double error = value - target;
        double error = target - value;

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
        }
        else {
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
    public double getTempKP() {
        return kP;
    }
    public double getTempKI() {
        return kI;
    }
    public double getTempKD() {
        return kD;
    }
    public double getTarget() {
        return target;
    }
    public void setTempKP(double kP) {
        this.kP = kP;
    }
    public void setTempKI(double kI) {
        this.kI = kI;
    }
    public void setTempKD(double kD) {
        this.kD = kD;
    }
    public void setTarget(double target) {
        this.target = target;
    }
}