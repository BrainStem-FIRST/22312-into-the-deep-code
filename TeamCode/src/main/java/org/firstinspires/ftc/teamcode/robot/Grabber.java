package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class Grabber extends Subsystem {
    //TODO: find min tick and max tick positions for servo
    public static final int MIN_TICK = 0, MAX_TICK = 100;
    public static final double CLOSE_POSITION = 0, OPEN_POSITION = 1;
    public static final double DESTINATION_THRESHOLD = 0.1;
    public enum StateType {
        OPEN, OPENING, CLOSED, CLOSING
    }
    private final StateManager<StateType> stateManager;
    private final ServoImplEx grabServo;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Input input) {
        super(hwMap, telemetry, allianceColor, robot, input);

        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        grabServo.setPwmRange(new PwmControl.PwmRange(CLOSE_POSITION, OPEN_POSITION));

        stateManager = new StateManager<>(StateType.CLOSED);

        stateManager.addState(StateType.OPEN, new OpenState());
        stateManager.addState(StateType.OPENING, new OpeningState());
        stateManager.addState(StateType.CLOSED, new CloseState());
        stateManager.addState(StateType.CLOSING, new ClosingState());

        stateManager.setupStates(robot, input, stateManager);
        stateManager.tryEnterState(StateType.CLOSED);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }
    public ServoImplEx getGrabServo() {
        return grabServo;
    }
}
