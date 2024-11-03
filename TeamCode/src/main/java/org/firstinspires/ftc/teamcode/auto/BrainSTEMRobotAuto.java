package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

public class BrainSTEMRobotAuto extends BrainSTEMRobot {

    public HardwareMap hwMap;
    public Telemetry telemetry;
    public AllianceColor allianceColor;

    public MecanumDrive drive;
    public LiftAuto lift;
    public CollectorAuto collector;

    public BrainSTEMRobotAuto(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor, Gamepad gamepad) {

        super(hwMap, telemetry, opMode, allianceColor);

        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        lift = new LiftAuto(hwMap, telemetry, allianceColor, this, gamepad);
        collector = new CollectorAuto(hwMap, telemetry, allianceColor, this, gamepad);
    }

    public void resetAllEncoders() {

    }
}
