package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class Grabber extends Subsystem {
    //TODO: find min tick and max tick positions for servo
    public static final int MIN_TICK = 0, MAX_TICK = 100;
    public static final double CLOSE_POSITION = 0, OPEN_POSITION = 1;
    public static final double DESTINATION_THRESHOLD = 0.1;
    private boolean hasBlock = false, hasSpecimen = false;
    private BlockColor blockColorHeld = BlockColor.NONE;
    public enum StateType {
        OPEN, OPENING, CLOSED, CLOSING
    }
    private final StateManager<StateType> stateManager;
    private final ServoImplEx grabServo;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        grabServo.setPwmRange(new PwmControl.PwmRange(CLOSE_POSITION, OPEN_POSITION));

        stateManager = new StateManager<>(StateType.CLOSED);

        stateManager.addState(StateType.OPEN, new NothingState<>(StateType.OPEN));
        stateManager.addState(StateType.OPENING, new OpeningState());
        stateManager.addState(StateType.CLOSED, new NothingState<>(StateType.CLOSED));
        stateManager.addState(StateType.CLOSING, new ClosingState());

        stateManager.setupStates(robot, stateManager);
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
    public boolean getHasBlock() {
        return hasBlock;
    }
    public void setHasBlock(boolean hasBlock) {
        this.hasBlock = hasBlock;
    }
    public boolean getHasSpecimen() {
        return hasSpecimen;
    }
    public void setHasSpecimen(boolean hasSpecimen) {
        this.hasSpecimen = hasSpecimen;
    }
    public void setBlockColorHeld(BlockColor blockColor) {
        this.blockColorHeld = blockColor;
    }
    public BlockColor getBlockColorHeld() {
        return blockColorHeld;
    }
}
