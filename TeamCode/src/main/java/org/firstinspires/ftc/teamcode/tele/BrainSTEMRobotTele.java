package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public class BrainSTEMRobotTele extends BrainSTEMRobot {

    private final DriveTrainTele driveTrain;
    private final ExtensionTele extension;
    private final CollectorTele collector;
    private final CollectingSystemTele collectingSystemTele;
    private final LiftingSystemTele liftingSystemTele;

    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {
        super(telemetry, opMode, allianceColor);

        driveTrain = new DriveTrainTele(hwMap, telemetry, allianceColor);
        collector = new CollectorTele(hwMap, telemetry, allianceColor);
        extension = new ExtensionTele(hwMap, telemetry, allianceColor);
        collectingSystemTele = new CollectingSystemTele(this, opMode.gamepad1);
        liftingSystemTele = new LiftingSystemTele(hwMap, telemetry, allianceColor);
    }

    public DriveTrainTele getDriveTrain() {
        return driveTrain;
    }

    public ExtensionTele getExtension() {
        return extension;
    }

    public CollectorTele getCollector() {
        return collector;
    }

    public CollectingSystemTele getCollectingSystemTele() {
        return collectingSystemTele;
    }

    public LiftingSystemTele getLiftingSystemTele() {
        return liftingSystemTele;
    }

    public void update(double dt) {
        // collector only looks at color sensor data once per frame
        collector.resetUpdateBlockColor();

        // update state managers
        collectingSystemTele.update(dt);
        liftingSystemTele.update();
    }

    public boolean tryExtendAndCollect() {
        return collectingSystemTele.getStateManager().tryEnterState(CollectingSystemTele.StateType.EXTENDING);
    }

    // retract and hinge up (if necessary)
    public void manualReset() {
        collectingSystemTele.getStateManager().tryEnterState(CollectingSystemTele.StateType.RETRACTING);
    }
}