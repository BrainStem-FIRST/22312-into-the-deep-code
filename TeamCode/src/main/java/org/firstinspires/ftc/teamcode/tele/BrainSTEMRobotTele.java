package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;

public class BrainSTEMRobotTele extends BrainSTEMRobot {

    private final DriveTrainTele driveTrain;
    private final Extension extension;
    private final Collector collector;
    private final CollectingSystem collectingSystem;
    private final LiftingSystemTele liftingSystemTele;

    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {
        super(telemetry, opMode, allianceColor);

        collectingSystem = new CollectingSystem(this, opMode.gamepad1);

        driveTrain = new DriveTrainTele(hwMap, telemetry, allianceColor);
        collector = new Collector(hwMap, telemetry, allianceColor, this, opMode.gamepad1, collectingSystem);
        extension = new Extension(hwMap, telemetry, allianceColor, this, opMode.gamepad1, collectingSystem);
        liftingSystemTele = new LiftingSystemTele(hwMap, telemetry, allianceColor);
    }

    public DriveTrainTele getDriveTrain() {
        return driveTrain;
    }

    public Extension getExtension() {
        return extension;
    }

    public Collector getCollector() {
        return collector;
    }

    public CollectingSystem getCollectingSystem() {
        return collectingSystem;
    }

    public LiftingSystemTele getLiftingSystemTele() {
        return liftingSystemTele;
    }

    public void update(double dt) {

        // update state managers
        collectingSystem.update(dt);
        liftingSystemTele.update();

        // update individual subsystems
        collector.update(dt);
        extension.update(dt);
    }
}