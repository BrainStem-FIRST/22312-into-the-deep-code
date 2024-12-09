package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean canTransfer; // resets during retraction of collecting system
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

        canTransfer = false;
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
        double time = 0;

        setupHangingSystem();

        while(!setupLiftingSystem() || !setupCollectingSystem()) {

            telemetry.addData("", "");
            telemetry.addData("robot status", "setting up");
            telemetry.addData("time", time);
            telemetry.update();
            time = System.currentTimeMillis() / 1000.0 - start;
        }
    }
    private boolean setupLiftingSystem() {
        telemetry.addData("setting up lifting system", "");
        grabber.getGrabServo().setPosition(Grabber.OPEN_POS);
        Subsystem.setMotorPosition(lift.getLiftMotor(), Lift.TROUGH_SAFETY_POS);
        if(lift.getLiftMotor().getCurrentPosition() >= Lift.TROUGH_SAFETY_POS - Lift.DESTINATION_THRESHOLD)
            arm.getArmServo().setPosition(Arm.TRANSFER_POS);
        return arm.getArmServo().getPosition() == Arm.TRANSFER_POS;
    }
    private boolean setupCollectingSystem() {
        telemetry.addData("setting up collecting system", "");
        ElapsedTime timer = new ElapsedTime();
        hinge.setHingeServoPosition(Hinge.HINGE_UP_POSITION);

        if (timer.seconds() >= Hinge.HINGE_UP_TIME)
            extension.retractExtensionMotor();
        if (extension.hitRetractHardStop()) {
            extension.getExtensionMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        return extension.hitRetractHardStop();
    }
    private void setupHangingSystem() {
        hanger.getHangMotor().setTargetPosition(Hanger.FULL_DOWN_ENCODER);
    }

    //  NOTE: COLLECTING SYSTEM NEEDS TO BE UPDATED BEFORE LIFTING SYSTEM TO ENSURE COLOR SENSOR VALUES ARE UP TO DATE WHEN LIFTING SYSTEM USES THEM
    public void update(double dt) {
        // drive train
        driveTrain.updatePoseEstimate();

        // collecting system
        collectingSystem.update(dt);
        collector.update(dt);
        hinge.update(dt);
        extension.update(dt);

        // lifting system
        liftingSystem.update(dt);
        grabber.update(dt);
        arm.update(dt);
        lift.update(dt);

        // hanging system
        hanger.update(dt);
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