package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

public class DriveTrain extends MecanumDrive {
    private AllianceColor allianceColor;
    private int tick;

    private double leftStickX, leftStickY, rightStickX;
    private static final double threshold = 0.1;

    public float frontLeftPower = 0;
    public float backLeftPower = 0;
    public float frontRightPower = 0;
    public float backRightPower = 0;

    public DriveTrain(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, new Pose2d(0, 0, 0));
        this.allianceColor = allianceColor;
    }
    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
    public void setDTMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
