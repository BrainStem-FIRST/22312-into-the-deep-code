package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotStates.robot.PlayingState;
import org.firstinspires.ftc.teamcode.robotStates.robot.SettingUpState;

public class BrainSTEMRobot extends Subsystem<BrainSTEMRobot.StateType> {
    private final PinpointDrive driveTrain;
    private final Extension extension;
    private final Hinge hinge;
    private final Collector collector;
    private final CollectingSystem collectingSystem;
    private final Grabber grabber;
    private final Arm arm;
    private final Lift lift;
    private final LiftingSystem liftingSystem;
    private final Hanger hanger;
    private boolean canTransfer; // resets during retraction of collecting system
    private boolean isHighDeposit;
    private boolean isHighRam;
    private boolean isDepositing;

    public enum StateType {
        SETTING_UP,
        PLAYING
    }

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor, null, StateType.SETTING_UP);

        driveTrain = new PinpointDrive(hwMap, new Pose2d(0, 0, 0));

        collector = new Collector(hwMap, telemetry, allianceColor, this);
        extension = new Extension(hwMap, telemetry, allianceColor, this);
        hinge = new Hinge(hwMap, telemetry, allianceColor, this);
        collectingSystem = new CollectingSystem(this);

        grabber = new Grabber(hwMap, telemetry, allianceColor, this);
        arm = new Arm(hwMap, telemetry, allianceColor, this);
        lift = new Lift(hwMap, telemetry, allianceColor, this);
        liftingSystem = new LiftingSystem(this);

        hanger = new Hanger(hwMap, telemetry, allianceColor, this);

        stateManager.addState(StateType.SETTING_UP, new SettingUpState());
        stateManager.addState(StateType.PLAYING, new PlayingState());
        stateManager.setupStates(this, stateManager);

        canTransfer = false; // only set to false in collectTemp and spitTemp states; reset to true for every retraction and after every transfer
        isHighDeposit = true;
        isHighRam = true;
        isDepositing = true;
    }
    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, Pose2d beginPose) {
        super(hwMap, telemetry, allianceColor, null, StateType.SETTING_UP);

        driveTrain = new PinpointDrive(hwMap, beginPose);

        collector = new Collector(hwMap, telemetry, allianceColor, this);
        extension = new Extension(hwMap, telemetry, allianceColor, this);
        hinge = new Hinge(hwMap, telemetry, allianceColor, this);
        collectingSystem = new CollectingSystem(this);

        grabber = new Grabber(hwMap, telemetry, allianceColor, this);
        arm = new Arm(hwMap, telemetry, allianceColor, this);
        lift = new Lift(hwMap, telemetry, allianceColor, this);
        liftingSystem = new LiftingSystem(this);

        hanger = new Hanger(hwMap, telemetry, allianceColor, this);

        stateManager.addState(StateType.SETTING_UP, new SettingUpState());
        stateManager.addState(StateType.PLAYING, new PlayingState());
        stateManager.setupStates(this, stateManager);

        canTransfer = true;
        isHighDeposit = true;
        isHighRam = true;
        isDepositing = true;
    }

    public void update(double dt) {
        // playing state now updates all the subsystems
        stateManager.update(dt);
    }

    public PinpointDrive getDriveTrain() {
        return driveTrain;
    }

    public Extension getExtension() {
        return extension;
    }
    public Hinge getHinge() { return hinge; }

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

    public Hanger getHanger() { return hanger; }

    public BlockColor getColorFromAlliance() {
        return allianceColor == AllianceColor.BLUE ? BlockColor.BLUE : BlockColor.RED;
    }
    public boolean canCollect() {
        return !grabber.hasBlock();
    }
    public boolean canTransfer() {
        return canTransfer;
    }
    public void setCanTransfer(boolean canTransfer) {
        this.canTransfer = canTransfer;
    }
    public boolean isHighDeposit() {
        return isHighDeposit;
    }
    public void setIsHighDeposit(boolean isHighDeposit) {
        this.isHighDeposit = isHighDeposit;
    }
    public boolean isHighRam() {
        return isHighRam;
    }
    public void setIsHighRam(boolean isHighRam) {
        this.isHighRam = isHighRam;
    }
    public boolean isDepositing() {
        return isDepositing;
    }
    public void setIsDepositing(boolean isDepositing) {
        this.isDepositing = isDepositing;
    }

    public Action retractAndDepositAndExtend(int extensionTick) {
        return new SequentialAction(
                getCollectingSystem().retractAction(),
                getCollector().stopCollect(),
                getLiftingSystem().transferBlockOnce(),
                new ParallelAction(
                        getCollectingSystem().startCollectSequence(extensionTick),
                        getLiftingSystem().depositHigh()
                )
        );
    }
    public Action retractAndDeposit() {
        return new SequentialAction(
                getCollectingSystem().retractAction(),
                getCollector().stopCollect(),
                getLiftingSystem().transferBlockOnce(),
                getLiftingSystem().depositHigh()
        );
    }
}