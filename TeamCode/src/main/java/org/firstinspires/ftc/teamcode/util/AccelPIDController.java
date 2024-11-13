package org.firstinspires.ftc.teamcode.util;

public class AccelPIDController extends PIDController{

    private double start, accelerationPoint;
    private double kA;

    public AccelPIDController(double kA, double kP, double kI, double kD) {
        super(kP, kI, kD);
        this.kA = kA;
    }
    public double getkA() {
        return kA;
    }
    public double getStart() {
        return start;
    }
    public double getAccelerationPoint() {
        return accelerationPoint;
    }
    public void setkA(double kA) {
        this.kA = kA;
    }
    public void setStart(double start) {
        this.start = start;
    }
    public void setAccelerationPoint(double accelerationPoint) {
        this.accelerationPoint = accelerationPoint;
    }

    @Override
    public double update(double value) {

        if(value < accelerationPoint)
            // x = v_f^2 / 2a
            // v_f = (2ax)^0.5
            return Math.sqrt(2 * kA * (value - start));

        return super.update(value);
    }

    public void reset(double start, double accelerationPoint, double target) {
        shouldReset = true;
        this.start = start;
        this.accelerationPoint = accelerationPoint;
        this.target = target;
    }
}
