package org.firstinspires.ftc.teamcode.driveTrain.useless;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.driveTrain.PathPoint;
import org.firstinspires.ftc.teamcode.util.BrainVector;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayList;
import java.util.Vector;

@Config
public class PIDDrivetrain {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private Pose2D lastPinpointPose, currentPose, targetPose, leadPoint;
    private double currentVelocity, currentAcceleration, previousVelocity, previousAcceleration,
            previousForwardPower;
    private long previousTimestamp;
    private PIDController forwardPIDControl, strafePIDControl, headPIDControl;
    private Telemetry telemetry;

    private double calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
            calculatedRightBackPower;

    public static class Params {
        public double forward_kP = 0.125;
        public double forward_kI = 0.075;
        public double forward_kD = 0.02;

        public double strafe_kP = 0.0125;
        public double strafe_kI = 0.075;
        public double strafe_kD = 0.02;

        public double head_kP = 0.075;
        public double head_kI = 0.0025;
        public double head_kD = 0.0;

        public double kV = 0.1;
        public double kA = 0.1;
        public double kS = 0.1;
        public double rampRate = 0.01;

        public double xOffset = 4.724;
        public double yOffset = 5.197;
        public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriver pinpoint;

    private ArrayList<PathPoint> pathPoints;
    private PathPoint currentPathPoint;
    private DistanceUnit distanceUnit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    public PIDDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, Pose2D pose) {
        currentPose = pose;
        this.telemetry = telemetry;
        pathPoints = new ArrayList<PathPoint>();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        resetPosition(currentPose);

        forwardPIDControl = new PIDController(PARAMS.forward_kP, PARAMS.forward_kI, PARAMS.forward_kD);
        forwardPIDControl.setInputBounds(-72, 72);
        forwardPIDControl.setOutputBounds(-1.0,1.0);

        strafePIDControl = new PIDController(PARAMS.strafe_kP, PARAMS.strafe_kI, PARAMS.strafe_kD);
        strafePIDControl.setInputBounds(-72, 72);
        forwardPIDControl.setOutputBounds(-1.0,1.0);

        headPIDControl = new PIDController(PARAMS.head_kP, PARAMS.head_kI, PARAMS.head_kD);
        headPIDControl.setInputBounds(-360, 360);
        headPIDControl.setOutputBounds(-1.0, 1.0);



        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftBack = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void resetPosition(Pose2D pose) {
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(pose);
    }

    private Pose2D queryVelocity() {
        return pinpoint.getVelocity();
    }

    private Pose2D queryPose() {
        return pinpoint.getPosition();
    }

    public Pose2D getCurrentPose() { return currentPose; }

    public double getCurrentX() {
        return getCurrentPose().getX(distanceUnit);
    }

    public double getCurrentY() {
        return getCurrentPose().getY(distanceUnit);
    }

    public double getTargetX() {
        return getTargetPose().getX(distanceUnit);
    }

    public double getTargetY() {
        return getTargetPose().getY(distanceUnit);
    }

    public double getCurrentHeadingRadians() {
        return getCurrentPose().getHeading(AngleUnit.RADIANS);
    }

    public double getTargetHeadingRadians() {
        return getTargetPose().getHeading(AngleUnit.RADIANS);
    }

    public Pose2D getTargetPose() { return targetPose; }

    private void moveToTargetPose() {
        forwardPIDControl.setTarget(getTargetX());
        strafePIDControl.setTarget(getTargetY());
        headPIDControl.setTarget(targetPose.getHeading(angleUnit));
        calculateMotorPowers();

        setDrivePower(calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
                calculatedRightBackPower);
    }

    public void addPathPoint(Pose2D pathPoint, double displacementTolerance, double headingTolerance) {
        pathPoints.add(new PathPoint(pathPoint, displacementTolerance, headingTolerance));
    }

    public void addPathPoint(Pose2D pathPoint, double displacementTolerance, double headingTolerance,
                             double targetVelocity, double velocityTolerance) {
        pathPoints.add(new PathPoint(pathPoint, displacementTolerance, headingTolerance,
                targetVelocity, velocityTolerance));
    }

    private void drawField() {
        Pose2d currentPose = new Pose2d(getCurrentX(), getCurrentY(), getCurrentHeadingRadians());
        TelemetryPacket packet = new TelemetryPacket();

        if (targetPose != null) {
            Pose2d targetPose = new Pose2d(getTargetX(), getTargetY(), getTargetHeadingRadians());

            packet.fieldOverlay().setStroke("#4CAF50");
            Drawing.drawRobot(packet.fieldOverlay(), targetPose);
        }

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), currentPose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public boolean drivePath() {
        updatePoseEstimate();
        drawField();
        if (pathPoints.size() == 0) {
            stop();
            return true;
        } else {
            PathPoint firstPathPoint = pathPoints.get(0);
            if (firstPathPoint.inTolerance(currentPose, currentVelocity)) {
                pathPoints.remove(0);
            }

            if (pathPoints.size() > 0) {
                currentPathPoint = pathPoints.get(0);
                leadPoint = calculateLeadPoint(currentPathPoint.getPoint());
                setTargetPose(leadPoint);
                moveToTargetPose();
                return false;
            } else {
                stop();
                return true;
            }
        }
    }

    private Pose2D calculateLeadPoint(Pose2D targetPoint) {
        double x1 = currentPose.getX(distanceUnit);
        double y1 = currentPose.getY(distanceUnit);
        double x2 = targetPoint.getX(distanceUnit);
        double y2 = targetPoint.getY(distanceUnit);

        double distanceToGo = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        double setPercentage = PARAMS.rampRate * currentVelocity;

        BrainVector currentTargetPoint = new BrainVector(x1, y1);
        BrainVector targetVectorPoint = new BrainVector(x2, y2);
        BrainVector leadPoint = currentTargetPoint.calculateLeadPoint(targetVectorPoint, 0.005);

        telemetry.addData("leadPoint X", leadPoint.getX());
        telemetry.addData("leadPoint Y", leadPoint.getY());

        return new Pose2D(distanceUnit, leadPoint.getX(), leadPoint.getY(), angleUnit, targetPoint.getHeading(angleUnit));
    }

    private boolean isReversePower(double forwardPower) {
        return (forwardPower > 0 && previousForwardPower < 0) || (forwardPower < 0 && previousForwardPower > 0);
    }

    private void calculateMotorPowers() {
        double feedForward = PARAMS.kV * currentVelocity + PARAMS.kA * currentAcceleration + PARAMS.kS;

        double forwardPower = -feedForward * forwardPIDControl.update(currentPose.getX(distanceUnit));
        double strafePower =  strafePIDControl.update(currentPose.getY(distanceUnit));
        double headPower = headPIDControl.update(currentPose.getHeading(angleUnit));

        telemetry.addData("headPIDControl target", headPIDControl.getTarget());
        telemetry.addData("previousForwardPower", previousForwardPower);
        telemetry.addData("forwardPower", forwardPower);
        telemetry.addData("headPower", headPower);
        telemetry.addData("target heading", getTargetPose().getHeading(angleUnit));
        telemetry.addData("current heading", getCurrentPose().getHeading(angleUnit));
        telemetry.addData("currentVelocity", currentVelocity);
        telemetry.addData("currentAcceleration", currentAcceleration);
        telemetry.addData("feedForward", feedForward);

        calculatedLeftFrontPower = forwardPower + strafePower + headPower;
        calculatedLeftBackPower = forwardPower - strafePower + headPower;
        calculatedRightFrontPower = forwardPower - strafePower - headPower;
        calculatedRightBackPower = forwardPower + strafePower - headPower;

//        telemetry.addData("forwardPower", forwardPower);
//        telemetry.addData("strafePower", strafePower);
//        telemetry.addData("headPower", headPower);
//        telemetry.addData("calculatedLeftFrontPower", calculatedLeftFrontPower);
//        telemetry.addData("calculatedLeftBackPower", calculatedLeftBackPower);
//        telemetry.addData("calculatedRightFrontPower", calculatedRightFrontPower);
//        telemetry.addData("calculatedRightBackPower", calculatedRightBackPower);

        previousForwardPower = forwardPower;
    }

    public Pose2D updatePoseEstimate() {
        if (lastPinpointPose != currentPose) {
            pinpoint.setPosition(currentPose);
        }
        pinpoint.update();
        currentPose = queryPose();
        Pose2D velocityPose = queryVelocity();
        double xVelocity = velocityPose.getX(distanceUnit);
        double yVelocity = velocityPose.getY(distanceUnit);
        currentVelocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));
        long currentTimestamp = System.currentTimeMillis();
        currentAcceleration = (currentVelocity - previousVelocity) /
                (currentTimestamp - previousTimestamp) / 1000;

        previousVelocity = currentVelocity;
        previousTimestamp = currentTimestamp;
        lastPinpointPose = currentPose;

        return velocityPose;
    }

    public void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower,
                              double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

}