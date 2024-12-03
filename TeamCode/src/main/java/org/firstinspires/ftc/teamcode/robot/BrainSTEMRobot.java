package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private boolean canTransfer;
    private boolean isHighDeposit;
    private boolean isHighRam;
    private boolean isDepositing;


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

        canTransfer = true;
        isHighDeposit = true;
        isHighRam = true;
        isDepositing = true;
    }
    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, Pose2d beginPose) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

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

        canTransfer = true;
        isHighDeposit = true;
        isHighRam = true;
        isDepositing = true;
    }

    public void setup() {
        double start = System.currentTimeMillis() / 1000.0;
        double time = 0, liftDoneTime = 0;

        while(true) {
            // setup lift first, then collection and extension
            // once both are set, stop
            if (setupLiftingSystem(time)) {
                if (liftDoneTime == 0)
                    liftDoneTime = time;

                if(setupCollectingSystem(time - liftDoneTime))
                    break;
            }

            telemetry.addData("robot status", "setting up");
            telemetry.addData("time", time);
            telemetry.addData("lift encoder", lift.getLiftMotor().getCurrentPosition());
            telemetry.update();
            time = System.currentTimeMillis() / 1000.0 - start;
        }
    }
    private boolean setupLiftingSystem(double time) {
        grabber.getGrabServo().setPosition(Grabber.OPEN_POS);
        Subsystem.setMotorPosition(lift.getLiftMotor(), Lift.TROUGH_SAFETY_POS);
        if(lift.getLiftMotor().getCurrentPosition() >= Lift.TROUGH_SAFETY_POS - Lift.DESTINATION_THRESHOLD)
            arm.getArmServo().setPosition(Arm.TRANSFER_POS);
        return arm.getArmServo().getPosition() == Arm.TRANSFER_POS && time > 1;
    }
    private boolean setupCollectingSystem(double time) {
        hinge.setHingeServoPosition(Hinge.HINGE_UP_POSITION);

        if (time >= Hinge.FULL_ROTATION_TIME)
            extension.setExtensionMotorPower(-0.5);
        if (extension.hitRetractHardStop()) {
            extension.setExtensionMotorPosition(0);
            extension.getExtensionMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        return extension.hitRetractHardStop();
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

        hanger.update(dt);

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

    public Action retractAndTransferAndDeposit() {
        return new SequentialAction(
                getCollectingSystem().retractAction(),
                getCollector().stopCollector(),
                getLiftingSystem().transferBlock(),
                getLiftingSystem().depositHigh()
        );
    }
}