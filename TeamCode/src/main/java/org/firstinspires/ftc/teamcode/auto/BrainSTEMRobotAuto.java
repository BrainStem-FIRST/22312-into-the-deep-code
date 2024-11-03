package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    public CollectorTele collector;

    public BrainSTEMRobotAuto(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {

        super(telemetry, opMode, allianceColor);

        this.hwMap = hwMap;

        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        lift = new LiftAuto(hwMap, telemetry, allianceColor);
        collector = new CollectorTele(hwMap, telemetry, allianceColor);
    }
}
