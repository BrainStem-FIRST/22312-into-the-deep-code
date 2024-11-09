package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PinpointDrive;

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
    private BlockColor blockColorHeld;
    private boolean isHighDeposit = true;
    private boolean isHighRam = true;


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

        blockColorHeld = BlockColor.NONE;
    }

    public boolean setup() {
        telemetry.addData("motor position", lift.getLiftMotor().getCurrentPosition());
        setupCollectingSystem();
        grabber.getGrabServo().setPosition(Grabber.OPEN_POSITION);

        if(Math.abs(arm.getArmServo().getPosition() - Arm.DOWN_POS) <= Arm.DESTINATION_THRESHOLD ||
                Math.abs(lift.getLiftMotor().getCurrentPosition() - Lift.TROUGH_SAFETY_POS) <= Lift.DESTINATION_THRESHOLD) {
            arm.getArmServo().setPosition(Arm.DOWN_POS);
            Subsystem.setMotorPosition(lift.getLiftMotor(), Lift.TROUGH_POS);
        }
        else
            Subsystem.setMotorPosition(lift.getLiftMotor(), Lift.TROUGH_SAFETY_POS);

        return Math.abs(arm.getArmServo().getPosition() - Arm.DOWN_POS) <= Arm.DESTINATION_THRESHOLD &&
                Math.abs(lift.getLiftMotor().getCurrentPosition() - Lift.TROUGH_POS) <= Lift.DESTINATION_THRESHOLD;
    }

    public void setupCollectingSystem() {
        hinge.setHingeServoPosition(Hinge.HINGE_UP_POSITION);
    }

    public void update(double dt) {
        // system managers
        collectingSystem.update(dt);
        liftingSystem.update(dt);

        // update individual subsystems
        collector.update(dt);
        extension.update(dt);

        grabber.update(dt);
        arm.update(dt);
        lift.update(dt);

    }

    public MecanumDrive getDriveTrain() {
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
}