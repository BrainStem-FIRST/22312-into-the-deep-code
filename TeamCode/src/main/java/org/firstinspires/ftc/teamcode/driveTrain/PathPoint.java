package org.firstinspires.ftc.teamcode.driveTrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PathPoint {
    private Pose2D point;
    private double displacementTolerance;
    private double headingTolerance;
    private double targetVelocity;
    private double velocityTolerance;
    private DistanceUnit distanceUnit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    public PathPoint(Pose2D point, double displacementTolerance, double headingTolerance) {
        this.point = point;
        this.displacementTolerance = displacementTolerance;
        this.headingTolerance = headingTolerance;
        this.targetVelocity = 0;
        this.velocityTolerance = 0;
    }

    public PathPoint(Pose2D point, double displacementTolerance, double headingTolerance,
                     double targetVelocity, double velocityTolerance) {
        this.point = point;
        this.displacementTolerance = displacementTolerance;
        this.headingTolerance = headingTolerance;
        this.targetVelocity = targetVelocity;
        this.velocityTolerance = velocityTolerance;
    }

    public Pose2D getPoint() {
        return point;
    }

    public double getDisplacementTolerance() {
        return displacementTolerance;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public boolean inTolerance(Pose2D comparisonPoint, double currentVelocity) {
        double x = comparisonPoint.getX(distanceUnit);
        double y = comparisonPoint.getY(distanceUnit);

        double centerX = point.getX(distanceUnit);
        double centerY = point.getY(distanceUnit);

        boolean headingInTolerance = (Math.abs(comparisonPoint.getHeading(angleUnit) -
                point.getHeading(angleUnit))) < headingTolerance;
        boolean displacementInTolerance = Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2) <
                Math.pow(displacementTolerance, 2);
        boolean velocityInTolerance = Math.abs(targetVelocity - currentVelocity) < velocityTolerance;

        return headingInTolerance && displacementInTolerance && velocityInTolerance;
    }
}