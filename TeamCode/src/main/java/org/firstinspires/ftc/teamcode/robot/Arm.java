package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.ServoTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class Arm extends Subsystem {
    // TODO: find arm servo positions and fine tune destination threshold (for this subsystem and all other subsystems)
    public static final int MIN_TICK = 0, MAX_TICK = 100;
    public static final double DOWN_POS = 0, LEFT_POS = 0.25, UP_POS = 0.5, RIGHT_POS = 0.75;
    public static final double DESTINATION_THRESHOLD = 0.1;
    public enum StateType {
        DOWN, LEFT, UP, RIGHT, TRANSITION
    }
    public final StateManager<StateType> stateManager;
    private final ServoTransitionState<StateType> transitionState;
    private final ServoImplEx armServo;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Input input) {
        super(hwMap, telemetry, allianceColor, robot, input);

        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));

        stateManager = new StateManager<>(Arm.StateType.DOWN);

        stateManager.addState(StateType.DOWN, new NothingState<>(StateType.DOWN));
        stateManager.addState(StateType.LEFT, new NothingState<>(StateType.LEFT));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.RIGHT, new NothingState<>(StateType.RIGHT));

        transitionState = new ServoTransitionState<>(StateType.TRANSITION, armServo, DESTINATION_THRESHOLD);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, gamepad1, gamepad2, stateManager);
        stateManager.tryEnterState(StateType.DOWN);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    public ServoImplEx getArmServo() {
        return armServo;
    }
    public ServoTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
}
