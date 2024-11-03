package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    private final Gamepad gamepad;
    public enum StateType {
        TROUGH, TROUGH_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_TROUGH, TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH
    }
    private final StateManager<StateType> stateManager;
    public LiftingSystem(BrainSTEMRobot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        stateManager = new StateManager<>(StateType.TROUGH);
    }
    public void update(double dt) {
        stateManager.update(dt);

        // b executes the current action (ex: actually dropping block into basket)
        if(gamepad.b) {
            if(stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT)
                robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
        }
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
}
