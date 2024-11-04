package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.HingeDownState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.HingeUpState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


// state mechanics: once collector/spit state initiated, will power until time reached, them automatically turn off
// do not need to turn motor off outside of this class

// need to add color sensor; could represent the one detecting block in grabber
// the collecting sequence should end only when color sensor detects block over extended period
// collecting sequence:
// collect motor spins in, and servo slowly turns inward and rapidly resets to try and match block
// if block is good, signal for de-extension
// if block not good, spin wheels in opposite direction and run collect motor in opposite direction
// then would need to adjust robot (and potentially extension) position to search new area for block

public class Collector extends Subsystem {

    public static final double MAX_SPIN_POWER = 1, HOLD_SPIN_POWER = 0.3;

    // store the absolute bounds for the servo (just in case)
    public static final int MAX_TICK = 2500, MIN_TICK = 100;
    public static final int HINGE_UP_TICK = 2213, HINGE_DOWN_TICK = 1236;

    public static final double HINGE_UP_POSITION = 0.01, HINGE_DOWN_POSITION = 0.99, HINGE_THRESHOLD = 0.05;

    // number of seconds to spit for
    // actual variable tracking time is stored in BaseState class and used in SpitState class
    public static final double SPITTING_TIME = 0.8;

    public enum StateType {
        READY_TO_HINGE_DOWN, HINGE_DOWN, COLLECTING, SPITTING, HINGE_UP, DONE_HINGING_UP
    }

    private final StateManager<StateType> stateManager;
    private final ServoImplEx hingeServo;
    private final DcMotorEx spindleMotor;

    // IN PROGRESS: replace touch sensor w color sensor and implement spitting state
    private final BlockColorSensor blockColorSensor;

    public Collector(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(hwMap, telemetry, allianceColor, robot, gamepad1, gamepad2);

        hingeServo = hwMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(HINGE_UP_TICK, HINGE_DOWN_TICK));

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");

        blockColorSensor = new BlockColorSensor(hwMap, telemetry);

        stateManager = new StateManager<>(StateType.READY_TO_HINGE_DOWN);

        NothingState<StateType> readyToHingeDownState = new NothingState<>(StateType.READY_TO_HINGE_DOWN);
        readyToHingeDownState.addMotor(spindleMotor);
        stateManager.addState(StateType.READY_TO_HINGE_DOWN, readyToHingeDownState);

        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.SPITTING, new SpitState());
        stateManager.addState(StateType.HINGE_UP, new HingeUpState());
        stateManager.addState(StateType.HINGE_DOWN, new HingeDownState());

        NothingState<StateType> doneHingingUpState = new NothingState<>(StateType.DONE_HINGING_UP);
        doneHingingUpState.addMotor(spindleMotor);
        stateManager.addState(StateType.DONE_HINGING_UP, doneHingingUpState);

        stateManager.setupStates(robot, gamepad1, gamepad2, stateManager);
        stateManager.tryEnterState(StateType.READY_TO_HINGE_DOWN);
    }

    public StateManager<Collector.StateType> getStateManager() {
        return stateManager;
    }

    public BlockColorSensor getColorSensor() {
        return blockColorSensor;
    }
    public DcMotorEx getSpindleMotor() { return spindleMotor; }
    public void setSpindleMotorPower(double power) {
        Subsystem.setMotorPower(spindleMotor, power);
    }
    public ServoImplEx getHingeServo() { return hingeServo; }
    public void setHingeServoPosition(double position) {
        hingeServo.setPosition(position);
    }

    @Override
    public void update(double dt) {
        blockColorSensor.resetUpdateBlockColor();
        stateManager.update(dt);
    }
    public boolean hasValidBlockColor() {
        return blockColorSensor.getBlockColor() == BlockColorSensor.BlockColor.YELLOW ||
                (blockColorSensor.getBlockColor() == BlockColorSensor.BlockColor.BLUE && getAllianceColor() == AllianceColor.BLUE) ||
                (blockColorSensor.getBlockColor() == BlockColorSensor.BlockColor.RED && getAllianceColor() == AllianceColor.RED);
    }
}




