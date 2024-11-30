package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;

public class BrainSTEMRobot {

    public Telemetry telemetry;
    private final AllianceColor allianceColor;
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
    private BlockColor blockColorHeld;
    private boolean isHighDeposit = true;
    private boolean isHighRam = true;
    private boolean inDepositingMode = true;
    private boolean inPidMode = false;


    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

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

        blockColorHeld = BlockColor.NONE;
    }

    public void setup() {
        double start = System.currentTimeMillis() / 1000.0;
        double relativeTime = 0;

        while(true) {
            // setup lift first, then collection and extension
            // once both are set, stop
            if(setupLiftingSystem() && setupCollectingSystem())
                    break;

            telemetry.addData("robot status", "setting up");
            telemetry.addData("relative time", relativeTime);
            telemetry.addData("lift encoder", lift.getLiftMotor().getCurrentPosition());
            telemetry.update();
            relativeTime = System.currentTimeMillis() / 1000.0 - start;
        }
    }
    public boolean setupLiftingSystem() {
        grabber.getGrabServo().setPosition(Grabber.OPEN_POSITION);
        Subsystem.setMotorPosition(lift.getLiftMotor(), Lift.TROUGH_SAFETY_POS);
        boolean liftInRange = Subsystem.inRange(lift.getLiftMotor(), Lift.TROUGH_SAFETY_POS, Lift.DESTINATION_THRESHOLD);
        if(liftInRange)
            arm.getArmServo().setPosition(Arm.TRANSFER_POS);
        return liftInRange && arm.getArmServo().getPosition() == Arm.TRANSFER_POS;
    }

    public boolean setupCollectingSystem() {
        // this allows the hinge to go to the up position
        hinge.getTransitionState().setGoalState(Hinge.HINGE_DOWN_POSITION, Hinge.StateType.DOWN);

        // perform full retraction sequence to make sure collecting system is setup correctly
        collectingSystem.getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
        return collectingSystem.getStateManager().getActiveStateType() == CollectingSystem.StateType.IN;
    }

    public void update(double dt) {
        // system managers
        collectingSystem.update(dt);
        liftingSystem.update(dt);

        // update individual subsystems
        collector.update(dt);
        hinge.update(dt);
        extension.update(dt);

        grabber.update(dt);
        arm.update(dt);
        lift.update(dt);

        //hanger.update(dt);

        driveTrain.updatePoseEstimate();
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
    public BlockColor getBlockColorHeld() {
        return blockColorHeld;
    }
    public void setBlockColorHeld(BlockColor blockColorHeld) {
        this.blockColorHeld = blockColorHeld;
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
    public boolean getInDepositingMode() {
        return inDepositingMode;
    }
    public void setInDepositingMode(boolean inDepositingMode) {
        this.inDepositingMode = inDepositingMode;
    }
    public boolean getInPidMode() {
        return inPidMode;
    }
    public void setInPidMode(boolean inPidMode) {
        this.inPidMode = inPidMode;
    }
}