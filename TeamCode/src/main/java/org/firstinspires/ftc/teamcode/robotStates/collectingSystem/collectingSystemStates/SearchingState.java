package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class SearchingState extends RobotState<CollectingSystem.StateType> {

    public SearchingState() {
        super(CollectingSystem.StateType.SEARCH);
    }

    @Override
    public void executeOnEntered() {
        Subsystem.setMotorPower(robot.getExtension().getExtensionMotor(), 0);
        robot.setCanTransfer(false);

        // make the extension go to min position
        if (robot.getExtension().getExtensionMotor().getCurrentPosition() < Extension.MIN_SEARCH_AND_COLLECT_POSITION)
            robot.getExtension().getStateManager().tryEnterState(Extension.StateType.JUMP_TO_MIN);
        else
            robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);

        // if previous state was search and collect, this will hinge up and stop collector
        robot.getHinge().goToHingeUpState();
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.NOTHING);
    }
    @Override
    public void execute(double dt) {
        // transitioning between collector doing nothing and spitting
        if (robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.NOTHING)
            robot.getHinge().goToHingeUpState();
        else if (robot.getCollector().isSpitting())
            robot.getHinge().goToHingeMiddleState();
    }

    @Override
    public boolean canEnter() {
        return true;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.SEARCH;
    }
}
