package org.firstinspires.ftc.teamcode.robotStates;

public abstract class TransitionState<StateType extends Enum<StateType>> extends RobotState<StateType> {
    double goalPosition;
    StateType goalStateType;
    public TransitionState(StateType stateType) {
        super(stateType);
    }

    /**
     * @param goalPosition the goal position to reach
     * @param goalStateType the next stateType to transition to after finishing transition
     */
    public void setGoalState(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }

    /**
     * changes goal position of transition, but keeps the state in transition (should be used if in transition and need to change destination point)
     * @param goalPosition the new position to transition to
     */
    public void overrideGoalPosition(double goalPosition) {
        this.goalPosition = goalPosition;
    }
    public double getGoalStatePosition() {
        return goalPosition;
    }
}
