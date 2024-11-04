package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.FindingBlock;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension extends Subsystem {

    // TODO: find extension encoder ticks for these 3
    // max position
    public static final int MAX_POSITION = 1400;
    public static final int RETRACTED_POSITION = 0;
    // TODO - implement magnet sensor and encoder reset
    public static final int RETRACTED_THRESHOLD = 30;
    public static final int EXTENDED_POSITION = 900;

    public enum StateType {
        IN, FINDING_BLOCK, RETRACTING
    }

    public final DcMotorEx extensionMotor;

    private final StateManager<StateType> stateManager;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(hwMap, telemetry, allianceColor, robot, gamepad1, gamepad2);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.FINDING_BLOCK, new FindingBlock());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(robot, gamepad1, gamepad2, stateManager);
        stateManager.tryEnterState(StateType.IN);
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

    @Override
    public void update(double dt) {


        stateManager.update(dt);
    }
}