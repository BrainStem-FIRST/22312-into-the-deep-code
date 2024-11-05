package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

]//
public class BrainSTEMRobot {

    public Telemetry telemetry;
    private final AllianceColor allianceColor;

    private final MecanumDrive driveTrain;

    public Extension extension;
    private Collector collector;
    private CollectingSystem collectingSystem;

    private Grabber grabber;
    private Arm arm;
    private Lift lift;
    private LiftingSystem liftingSystem;


    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

        driveTrain = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));

        collector = new Collector(hwMap, telemetry, allianceColor, this);
        extension = new Extension(hwMap, telemetry, allianceColor, this);
        collectingSystem = new CollectingSystem(this);

        grabber = new Grabber(hwMap, telemetry, allianceColor, this);
        arm = new Arm(hwMap, telemetry, allianceColor, this);
        lift = new Lift(hwMap, telemetry, allianceColor, this);
        liftingSystem = new LiftingSystem(this);
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
    public MecanumDrive getDriveTrain() {
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