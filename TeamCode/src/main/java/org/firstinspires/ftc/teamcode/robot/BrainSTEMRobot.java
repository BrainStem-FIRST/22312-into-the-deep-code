package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.DriveTrainTele;

public class BrainSTEMRobot {

    public Telemetry telemetry;
    public OpMode opMode;
    private final AllianceColor allianceColor;

    private final DriveTrainTele driveTrain;
    private final Extension extension;
    private final Collector collector;
    private final CollectingSystem collectingSystem;
    private final Grabber grabber;
    private final Arm arm;
    private final Lift lift;
    private final LiftingSystem liftingSystem;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;

        driveTrain = new DriveTrainTele(hwMap, telemetry, allianceColor);

        collector = new Collector(hwMap, telemetry, allianceColor, this, opMode.gamepad1);
        extension = new Extension(hwMap, telemetry, allianceColor, this, opMode.gamepad1);
        collectingSystem = new CollectingSystem(this, opMode.gamepad1);

        grabber = new Grabber(hwMap, telemetry, allianceColor, this, opMode.gamepad1);
        arm = new Arm(hwMap, telemetry, allianceColor, this, opMode.gamepad1);
        lift = new Lift(hwMap, telemetry, allianceColor, this, opMode.gamepad1);
        liftingSystem = new LiftingSystem(this, opMode.gamepad1);

    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
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

    public Grabber getGrabber() {
        return grabber;
    }
    public Arm getArm() {
        return arm;
    }

    public Lift getLift() {
        return lift;
    }
    public LiftingSystem getLiftingSystem() {
        return liftingSystem;
    }

    public void update(double dt) {

        if (collectingSystem.getStateManager().getActiveStateType() == CollectingSystem.StateType.IN) {
            if (collector.getBlockColor() == Collector.BlockColor.YELLOW)
                liftingSystem.getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH);
        }

        // system managers
        collectingSystem.update(dt);
        liftingSystem.update(dt);

        // update individual subsystems
        collector.update(dt);
        extension.update(dt);

        grabber.update(dt);

    }
}