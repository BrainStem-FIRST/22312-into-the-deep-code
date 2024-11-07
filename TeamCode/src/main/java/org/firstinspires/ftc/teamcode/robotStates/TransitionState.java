package org.firstinspires.ftc.teamcode.robotStates;

public abstract class TransitionState<StateType extends Enum<StateType>> extends RobotState<StateType> {
    double goalPosition;
    StateType goalStateType;
    final double DESTINATION_THRESHOLD;
    public TransitionState(StateType stateType, double DESTINATION_THRESHOLD) {
        super(stateType);
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
    }
    public void setGoalState(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition (because don't want to override that transition)
        if(stateManager.getActiveStateType() != stateType) {
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
    public void overrideGoalState(double goalPosition) {
        this.goalPosition = goalPosition;
        stateManager.tryEnterState(this.stateType);
    }
    public double getGoalStatePosition() {
        return goalPosition;
    }
}
