package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.extensionStates.ExtendingState;
import org.firstinspires.ftc.teamcode.robotStates.extensionStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.extensionStates.OutState;
import org.firstinspires.ftc.teamcode.robotStates.extensionStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.tele.BrainSTEMRobotTele;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension extends Subsystem {

    // TODO: find extension encoder ticks for these 3
    public static final int THRESHOLD = 30;
    public static final int RETRACTED_POSITION = 0;
    public static final int EXTENDED_POSITION = 1000;

    public enum StateType {
        IN, EXTENDING, OUT, RETRACTING
    }

    private final DcMotorEx extensionMotor;

    private final StateManager<StateType> stateManager;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobotTele robot, Gamepad gamepad, CollectingSystem collectingSystem) {
        super(hwMap, telemetry, allianceColor, robot, gamepad);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.OUT, new OutState());
        stateManager.addState(StateType.EXTENDING, new ExtendingState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(robot, gamepad, stateManager);
        stateManager.tryEnterState(StateType.IN);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public DcMotorEx getExtensionMotor() {
        return extensionMotor;
    }
    public void setExtensionMotorPosition(int position) {
        setMotorPosition(extensionMotor, position);
    }

    public void update(double dt) {
        // TODO: set the active state based off the active state of collectSystem
        stateManager.update(dt);
    }
}