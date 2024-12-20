package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchAndCollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.ShortExtendState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class CollectingSystem {

    // TODO: FIND THESE - currently set to max extension position
    // tick to start forcing hinge to go up if it is in middle position
    public static final int FORCE_HINGE_UP_FROM_MIDDLE_POSITION = Extension.MAX_POSITION;

    // same as force hinge up from middle but for down position
    public static final int FORCE_HINGE_UP_FROM_DOWN_POSITION = Extension.MAX_POSITION;

    public enum StateType {
        IN, SEARCH, SEARCH_AND_COLLECT, RETRACTING, SHORT_EXTEND
    }

    private final BrainSTEMRobot robot;
    private final StateManager<StateType> stateManager;
    public CollectingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.IN);
        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.SEARCH, new SearchingState());
        stateManager.addState(StateType.SEARCH_AND_COLLECT, new SearchAndCollectState());
        stateManager.addState(StateType.SHORT_EXTEND, new ShortExtendState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(robot, stateManager);
        stateManager.tryEnterState(StateType.IN);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("can collect", robot.canCollect());
        telemetry.addData("collecting system state", robot.getCollectingSystem().getStateManager().getActiveStateType());
        robot.getCollector().addTelemetry(telemetry);
        robot.getHinge().addTelemetry(telemetry);
        robot.getExtension().addTelemetry(telemetry);
    }
    public boolean hingeMustBeUp() {
        if (getRobot().getHinge().getStateManager().getActiveStateType() == Hinge.StateType.MIDDLE)
            return getRobot().getExtension().getExtensionMotorPosition() <= FORCE_HINGE_UP_FROM_MIDDLE_POSITION;
        if (getRobot().getHinge().getStateManager().getActiveStateType() == Hinge.StateType.DOWN)
            return getRobot().getExtension().getExtensionMotorPosition() <= FORCE_HINGE_UP_FROM_DOWN_POSITION;
        return false;
    }

    public void update(double dt) {
        stateManager.update(dt);

        // short extend while hanging
        if (robot.getInput().getGamepadTracker1().isAPressed()
                && robot.getLift().getTransitionState().getNextStateType() == Lift.StateType.RAM_AFTER)
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public Action startCollectSequence(int extendMotorTick) {
        return new ParallelAction(
                getRobot().getExtension().extendAction(extendMotorTick),

                new SequentialAction(
                        new SleepAction(0.1),
                        getRobot().getHinge().hingeDownAction()
                )

        );
    }

    public Action collect() {
        return new SequentialAction(
                getRobot().getCollector().collect(),
                getRobot().getHinge().hingeUpAction()
        );
    }
    public Action retractAction() {
        return new ParallelAction(
                getRobot().getCollector().collectUntilHardStop(),
                getRobot().getExtension().retractAction()
        );
    }
}