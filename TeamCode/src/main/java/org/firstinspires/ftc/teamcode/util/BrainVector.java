package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class BrainVector {
    private double x, y;

    public BrainVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public BrainVector directionVector(BrainVector v) {
        return new BrainVector(v.x - x, v.y - y);
    }

    public double vectorMagnitude(BrainVector v) {
        return Math.sqrt(Math.pow(v.x - x, 2) + Math.pow(v.y - y, 2));
    }

    public double calculateScalar(double reductionFactor, double magnitude) {
        return reductionFactor * magnitude;
    }

    public BrainVector calculateLeadPoint(BrainVector v, double reductionFactor) {
        BrainVector dV = directionVector(v);
        double magnitude = vectorMagnitude(v);
        double scalar = calculateScalar(reductionFactor, magnitude);
        double leadX = x + scalar * dV.x;
        double leadY = y + scalar * dV.y;
        return new BrainVector(leadX, leadY);
    }
}