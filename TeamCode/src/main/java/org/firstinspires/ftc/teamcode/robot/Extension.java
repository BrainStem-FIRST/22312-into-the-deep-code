package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.BrainSTEMRobotAuto;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.FindingBlockState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension extends Subsystem {

    // TODO: find extension encoder ticks for these 3
    // max position
    public static final int MAX_POSITION = 1400;
    // TODO - implement magnet sensor and encoder reset
    public static final double SEARCH_POWER = 0.55;
    public static final double RETRACT_POWER = -0.9;

    public enum StateType {
        IN, FINDING_BLOCK, RETRACTING
    }

    private DcMotorEx extensionMotor;
    private DigitalChannel magnetResetSwitch;

    private double targetPower;
    private int targetPosition;
    private DcMotor.RunMode runMode;

    private final StateManager<StateType> stateManager;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        magnetResetSwitch = hwMap.get(DigitalChannel.class, "ExtensionMagnetSwitch");
        magnetResetSwitch.setMode(DigitalChannel.Mode.INPUT);

        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.FINDING_BLOCK, new FindingBlockState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(getRobot(), stateManager);
        stateManager.tryEnterState(StateType.IN);

        // this is only for tele
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public DcMotorEx getExtensionMotor() {
        return extensionMotor;
    }
    public void setExtensionMotorPosition(int position) {
        Subsystem.setMotorPosition(extensionMotor, position);
    }
    public void setExtensionMotorPower(double power) {
        Subsystem.setMotorPower(extensionMotor, power);
    }
    public DigitalChannel getMagnetResetSwitch() {
        return magnetResetSwitch;
    }

    public DcMotor.RunMode getRunMode() {
        return runMode;
    }
    public void setRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
    }
    public double getTargetPower() {
        return targetPower;
    }
    public void setTargetPower(double targetPower) {
        this.targetPower = targetPower;
    }
    public int getTargetPosition() {
        return targetPosition;
    }
    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    @Override
    public void update(double dt) {
        // if the magnet switch detects the extension is close enough, it will reset its encoders
        if (!magnetResetSwitch.getState()) {
            Subsystem.setMotorPower(extensionMotor, 0);
            extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (extensionMotor.getCurrentPosition() >= MAX_POSITION)
            Subsystem.setMotorPower(extensionMotor, 0);

        stateManager.update(dt);
    }
}