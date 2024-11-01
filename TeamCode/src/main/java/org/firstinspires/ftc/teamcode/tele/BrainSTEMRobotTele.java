package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;

public class BrainSTEMRobotTele {
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private final AllianceColor allianceColor;

    public DriveTrainTele driveTrain;
    public ExtensionTele extension;
    public CollectorTele collector;
    public LiftingSystemTele liftingSystemTele;

    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {

        this.telemetry = telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;

        driveTrain = new DriveTrainTele(hwMap, telemetry, allianceColor);
        collector = new CollectorTele(hwMap, telemetry, allianceColor);
        extension = new ExtensionTele(hwMap, telemetry, allianceColor);
        liftingSystemTele = new LiftingSystemTele(hwMap, telemetry, allianceColor);
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void update() {
        // collector/extension handling for automatic transitions
        collector.update();
        // checking if collector found color and is hinging up
        if (collector.getHingeState() == Collector.HingeState.HINGING_UP)
            extension.setState(Extension.State.RETRACTING);
        extension.update();


        // lifting system handling for automatic transitions
        if(collector.getBlockColor() == Collector.BlockColor.YELLOW)
            prepBasketDeposit();
        // TODO: standardize BlockColor enums (we have AllianceColor, BlockColor but only need one)
        else if(collector.getBlockColor() == Collector.BlockColor.RED) // assuming you are on red team
            prepBlockDrop();
        liftingSystemTele.update();
    }

    // collecting actions
    public void tryExtendAndCollect() {
        // ensuring collector is ready for extension before extension
        if (collector.getCollectState() == Collector.CollectState.EMPTY && collector.getHingeState() == Collector.HingeState.UP) {
            extension.setState(Extension.State.EXTENDING);
            // collector automatically sets hingeState to DOWN when it finishes hinging
            collector.setHingeState(Collector.HingeState.HINGING_DOWN);
            // collector keeps collecting until block is in chamber, then it sets collectState to FULL_MAX
            collector.setCollectState(Collector.CollectState.COLLECTING);
        }
    }
    // Note: this assumes collector does not have block in it
    public void resetExtensionAndCollector() {
        collector.setHingeState(Collector.HingeState.HINGING_UP);
        collector.setCollectState(Collector.CollectState.EMPTY);
        extension.setState(Extension.State.RETRACTING);
    }

    // lifting system actions
    // requires block in trough
    public void prepTroughPickup() {
        if(collector.getCollectState() == Collector.CollectState.FULL_SLOW && extension.getState() == Extension.State.IN) // ensuring trough is full and ready
            liftingSystemTele.setState(LiftingSystemTele.State.TROUGH);
    }
    public void execTroughPickup() {
        if(liftingSystemTele.getState() == LiftingSystemTele.State.TROUGH && liftingSystemTele.getCurStateReady()) {}
            // call liftingSystem trough pickup execution function

    }
    public void prepBasketDeposit() {
        if(liftingSystemTele.getGrabber().hasBlock() && liftingSystemTele.getState() == LiftingSystemTele.State.TROUGH && liftingSystemTele.getCurStateReady())
            liftingSystemTele.setState(LiftingSystemTele.State.BASKET_DROP);
    }
    public void execBasketDeposit() {
        if(liftingSystemTele.getState() == LiftingSystemTele.State.BASKET_DROP && liftingSystemTele.getCurStateReady()) {}
        // call liftingSystem basket drop execution function
    }
    public void prepBlockDrop() {
        if(liftingSystemTele.getGrabber().hasBlock() && liftingSystemTele.getState() == LiftingSystemTele.State.TROUGH && liftingSystemTele.getCurStateReady())
            liftingSystemTele.setState(LiftingSystemTele.State.BLOCK_DROP);
    }
    public void execBlockDrop() {
        if(liftingSystemTele.getState() == LiftingSystemTele.State.BLOCK_DROP && liftingSystemTele.getCurStateReady()) {}
        // call liftingSystem block drop execution function
    }
    public void prepSpecimenRam() {
        if(liftingSystemTele.getGrabber().hasBlock() && liftingSystemTele.getState() == LiftingSystemTele.State.SPECIMEN_PICKUP && liftingSystemTele.getCurStateReady())
            liftingSystemTele.setState(LiftingSystemTele.State.SPECIMEN_RAM);
    }
    public void execSpecimenRam() {
        if(liftingSystemTele.getState() == LiftingSystemTele.State.SPECIMEN_RAM && liftingSystemTele.getCurStateReady()) {}
        // call liftingSystem ram specimen execution function
    }

    // requires specimen on wall
    public void prepSpecimenPickup() {
        if(!liftingSystemTele.getGrabber().hasBlock())
            liftingSystemTele.setState(LiftingSystemTele.State.SPECIMEN_PICKUP);
    }
    public void execSpecimenPickup() {
        if(liftingSystemTele.getState() == LiftingSystemTele.State.SPECIMEN_PICKUP && liftingSystemTele.getCurStateReady()) {}
        // call liftingSystem pickup specimen execution function
    }
}