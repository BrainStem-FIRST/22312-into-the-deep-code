package org.firstinspires.ftc.teamcode.robot;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

import com.qualcomm.robotcore.hardware.DcMotorEx;


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

    public static final double MAX_SPIN_POWER = 0.8;
    public static final double COLLECT_TEMP_POWER = 0.5, SPIT_TEMP_POWER = 0.5;

    // after the block color sensor stops detecting the block, still spit for 1 second

    // number of seconds to spit for
    public static final double SPITTING_TIME = 1;

    public enum StateType {
        NOTHING,
        COLLECTING,
        COLLECTING_TEMP, // stops next frame unless called continuously
        SPITTING, // only stops when block color sensor detects nothing + safety time
        SPITTING_TEMP, // stops next frame unless called continuously
        VALID_BLOCK
    }

    private final StateManager<StateType> stateManager;
    private final DcMotorEx spindleMotor;

    // IN PROGRESS: replace touch sensor w color sensor and implement spitting state
    private final BlockColorSensor blockColorSensor;

    public Collector(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");

        blockColorSensor = new BlockColorSensor(hwMap);

        stateManager = new StateManager<>(StateType.NOTHING);
        NothingState<StateType> nothingState = new NothingState<>(StateType.NOTHING);
        nothingState.addMotor(spindleMotor);
        stateManager.addState(StateType.NOTHING, nothingState);
        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.SPITTING, new SpitState());
        stateManager.addState(StateType.SPITTING_TEMP, new SpitTempState());
        stateManager.addState(StateType.VALID_BLOCK, new NothingState<>(StateType.VALID_BLOCK));
        stateManager.setupStates(getRobot(), stateManager);
    }

    public StateManager<Collector.StateType> getStateManager() {
        return stateManager;
    }

    public BlockColorSensor getBlockColorSensor() {
        return blockColorSensor;
    }
    public DcMotorEx getSpindleMotor() { return spindleMotor; }
    public void setSpindleMotorPower(double power) {
        Subsystem.setMotorPower(spindleMotor, power);
    }

    @Override
    public void update(double dt) {
        blockColorSensor.update(dt);
        stateManager.update(dt);
    }

    public boolean isCollecting() {
        return stateManager.getActiveStateType() == StateType.COLLECTING || stateManager.getActiveStateType() == StateType.COLLECTING_TEMP;
    }
    public boolean isSpitting() {
        return stateManager.getActiveStateType() == StateType.SPITTING || stateManager.getActiveStateType() == StateType.SPITTING_TEMP;
    }
    public boolean hasValidBlockColor() {
        return blockColorSensor.getBlockColor() == BlockColor.YELLOW || blockColorSensor.getBlockColor() == getRobot().getColorFromAlliance();
    }
    public Action collectAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setSpindleMotorPower(Collector.MAX_SPIN_POWER);
                return !hasValidBlockColor();
            }
        };
    }
}




