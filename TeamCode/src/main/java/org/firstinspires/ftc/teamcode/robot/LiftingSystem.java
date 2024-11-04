package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    private final Gamepad gamepad1, gamepad2;
    public enum StateType {
        TROUGH, TROUGH_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_TROUGH, TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH
    }
    private final StateManager<StateType> stateManager;
    public LiftingSystem(BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        stateManager = new StateManager<>(StateType.TROUGH);
    }
    public void update(double dt) {
        stateManager.update(dt);

        // b executes the current action (ex: actually dropping block into basket)
        if(gamepad1.b) {
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
