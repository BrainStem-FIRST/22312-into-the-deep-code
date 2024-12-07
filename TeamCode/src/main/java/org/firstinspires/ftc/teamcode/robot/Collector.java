package org.firstinspires.ftc.teamcode.robot;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectTempState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.MotorCurrentTracker;
import org.firstinspires.ftc.teamcode.util.MotorPowerJamTracker;

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

    public static final int AUTO_COLOR_VALIDATION_REQUIRED = 1;
    public static final double COLLECT_POWER = 1, SPIT_POWER = -1;
    public static final double COLLECT_TEMP_POWER = 0.4, SPIT_TEMP_POWER = -0.5, AUTO_SPIT_SLOW_POWER = -0.3;

    // after the block color sensor stops detecting the block, still spit for 1 second
    public static double SAFETY_SPIT_TIME = 0.8;
    // # encoder ticks the spindle motor must travel in 1 frame to not be considered jammed
    public static int JAM_ENCODER_TICK_REQUIREMENT = 10;
    // number of tries the spindle motor gets to travel jam encoder ticks before it is considered jammed
    // i.e. the collector gets 4 frame to try and travel at least 10 encoder ticks per frame before it is considered jammed
    public static int JAM_FRAME_REQUIREMENT = 4;


    private final StateManager<StateType> stateManager;
    public enum StateType {
        NOTHING,
        COLLECTING,
        COLLECTING_TEMP, // stops next frame unless called continuously
        SPITTING, // only stops when block color sensor detects nothing + safety time
        SPITTING_TEMP, // stops next frame unless called continuously
        VALID_BLOCK
    }
    public static int JAM_CURRENT_THRESHOLD = 6000;
    public static int JAM_VALIDATION_FRAMES = 1;
    private final MotorCurrentTracker motorCurrentTracker;
    private final DcMotorEx spindleMotor;

    private final BlockColorSensor blockColorSensor;
    private BlockColor blockColorInTrough;
    private int autoColorValidationFrames;

    public Collector(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");
        spindleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorCurrentTracker = new MotorCurrentTracker(spindleMotor, JAM_CURRENT_THRESHOLD, JAM_VALIDATION_FRAMES);

        blockColorSensor = new BlockColorSensor(hwMap, robot);
        blockColorInTrough = BlockColor.NONE;

        stateManager = new StateManager<>(StateType.NOTHING);
        stateManager.addState(StateType.NOTHING, new NothingState<>(StateType.NOTHING, spindleMotor));
        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.COLLECTING_TEMP, new CollectTempState());
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
    public BlockColor getBlockColorInTrough() {
        return blockColorInTrough;
    }
    public boolean hasValidBlockColor() {
        return blockColorInTrough == robot.getColorFromAlliance() || blockColorInTrough == BlockColor.YELLOW;
    }
    public void setBlockColorInTrough(BlockColor blockColorInTrough) {
        this.blockColorInTrough = blockColorInTrough;
    }
    public boolean isCollecting() {
        return stateManager.getActiveStateType() == StateType.COLLECTING || stateManager.getActiveStateType() == StateType.COLLECTING_TEMP;
    }
    public boolean isSpitting() {
        return stateManager.getActiveStateType() == StateType.SPITTING || stateManager.getActiveStateType() == StateType.SPITTING_TEMP;
    }
    public Action collectAction() {
        return telemetryPacket -> {
            // updating motor current check
            motorCurrentTracker.updateCurrentTracking();
            if(motorCurrentTracker.hasValidatedAbnormalCurrent())
                setSpindleMotorPower(Collector.SPIT_POWER);
            else
                setSpindleMotorPower(Collector.COLLECT_POWER);

            // checking for color sensor validation
            if (blockColorSensor.getRawBlockColor() != BlockColor.NONE)
                autoColorValidationFrames++;
            else
                autoColorValidationFrames = 0;

            return autoColorValidationFrames < AUTO_COLOR_VALIDATION_REQUIRED
                    && blockColorSensor.getRawBlockColor() != BlockColor.NONE;
        };
    }
    public Action collectUntilHardStop() {
        return telemetryPacket -> {
            setSpindleMotorPower(Collector.COLLECT_TEMP_POWER);
            return !robot.getExtension().hitRetractHardStop();
        };
    }
    public Action stopCollector() {
        return telemetryPacket -> {
            setSpindleMotorPower(0);
            return false;
        };
    }
}




