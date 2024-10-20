package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.CollectorTele;
import org.firstinspires.ftc.teamcode.tele.DriveTrainTele;
import org.firstinspires.ftc.teamcode.tele.LiftTele;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

public class BrainSTEMRobotAuto {

    public HardwareMap hwMap;
    public Telemetry telemetry;

    public MecanumDrive drive;
    public LiftAuto lift;
    public CollectorTele collector;

    public BrainSTEMRobotAuto(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        lift = new LiftAuto(hwMap, telemetry);
        collector = new CollectorTele(hwMap, telemetry);
    }
}
