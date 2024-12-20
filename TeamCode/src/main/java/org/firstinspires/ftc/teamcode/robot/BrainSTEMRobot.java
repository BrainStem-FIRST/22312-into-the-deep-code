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
import org.firstinspires.ftc.teamcode.util.Input;

public class BrainSTEMRobot extends Subsystem<BrainSTEMRobot.StateType> {
    private final Input input;
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
    private boolean isHighDeposit;
    private boolean isHighRam;
    private boolean isDepositing;

    public enum StateType {
        SETTING_UP,
        PLAYING
    }

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, Pose2d beginPose, Input input) {
        super(hwMap, telemetry, allianceColor, null, StateType.SETTING_UP);
        this.input = input;

        driveTrain = new PinpointDrive(hwMap, beginPose, this);

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

        isHighDeposit = true;
        isHighRam = true;
        isDepositing = true;
    }

    public void update(double dt) {
        // playing state now updates all the subsystems
        stateManager.update(dt);
    }

    public void addTelemetry() {
        telemetry.addData("robot alliance", robot.getColorFromAlliance());
        telemetry.addData("robot state", robot.getStateManager().getActiveStateType());
    }

    public Input getInput() {
        return input;
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