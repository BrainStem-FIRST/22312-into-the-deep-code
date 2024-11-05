package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class BrainSTEMRobot {

    public Telemetry telemetry;
    private final AllianceColor allianceColor;

    private final PIDDrivetrain driveTrain;

    public final Extension extension;
    private final Collector collector;
    private final CollectingSystem collectingSystem;

    private final Grabber grabber;
    private final Arm arm;
    private final Lift lift;
    private final LiftingSystem liftingSystem;


    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, Input input, AllianceColor allianceColor) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

        driveTrain = new PIDDrivetrain(hwMap, telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        collector = new Collector(hwMap, telemetry, allianceColor, this, input);
        extension = new Extension(hwMap, telemetry, allianceColor, this, input);
        collectingSystem = new CollectingSystem(this, input);

        grabber = new Grabber(hwMap, telemetry, allianceColor, this, input);
        arm = new Arm(hwMap, telemetry, allianceColor, this, input);
        lift = new Lift(hwMap, telemetry, allianceColor, this, input);
        liftingSystem = new LiftingSystem(this, input);
    }

    public void setup() {
        collector.setHingeServoPosition(Collector.HINGE_UP_POSITION);
        grabber.getGrabServo().setPosition(Grabber.OPEN_POSITION);
        // WOULD NOT WANT TO SET POSITION OF ARM IN CASE LIFT IS DOWN AND ARM IS NOT
    }

    public void update(double dt) {
        telemetry.addData("Extension motor current position", extension.getExtensionMotor().getCurrentPosition());


        collectingSystem.update(dt);

        collector.update(dt);
        extension.update(dt);
        /*
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
        */
        //extension.update(dt);
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
    public PIDDrivetrain getDriveTrain() {
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
}