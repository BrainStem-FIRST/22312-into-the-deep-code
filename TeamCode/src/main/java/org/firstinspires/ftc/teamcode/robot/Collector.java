package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.ColorSensor;
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
    private final int HINGE_UP_TICK = 2336, HINGE_DOWN_TICK = 1438;
    public static final double HINGE_UP_POSITION = 0.99, HINGE_DOWN_POSITION = 0.01, HINGE_THRESHOLD = 0.05;

    // number of seconds to spit for
    // actual variable tracking time is stored in BaseState class and used in SpitState class
    public static final double SPITTING_TIME = 0.8;

    public enum StateType {
        NOTHING, HINGE_UP, HINGE_DOWN, COLLECTING, SPITTING
    }
    public enum BlockColor {
        RED,
        YELLOW,
        BLUE,
        NONE
    }
    final public static int MAX_COLOR_THRESHOLD = 20;
    final public static int[] RED_BLOCK_COLOR = { 255, 0, 0 };
    final public static int[] YELLOW_BLOCK_COLOR = { 255, 255, 0 };
    final public static int[] BLUE_BLOCK_COLOR = { 0, 0, 255 };

    private final StateManager<StateType> stateManager;
    private final ServoImplEx hingeServo;
    private final DcMotorEx spindleMotor;

    // IN PROGRESS: replace touch sensor w color sensor and implement spitting state
    private final ColorSensor blockColorSensor;
    private boolean updatedBlockColor;
    private BlockColor blockColor;

    public Collector(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad) {
        super(hwMap, telemetry, allianceColor, robot, gamepad);

        hingeServo = hwMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(HINGE_DOWN_TICK, HINGE_UP_TICK));

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");

        blockColorSensor = hwMap.get(ColorSensor.class, "BlockColorSensor");
        updatedBlockColor = false;
        blockColor = BlockColor.NONE;

        stateManager = new StateManager<>(StateType.NOTHING);
        stateManager.addState(StateType.NOTHING, new NothingState<>(StateType.NOTHING));
        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.SPITTING, new SpitState());
        stateManager.addState(StateType.HINGE_UP, new HingeUpState());
        stateManager.addState(StateType.HINGE_DOWN, new HingeDownState());

        stateManager.setupStates(robot, gamepad, stateManager);
        stateManager.tryEnterState(StateType.NOTHING);
    }

    public StateManager<Collector.StateType> getStateManager() {
        return stateManager;
    }

    public DcMotorEx getSpindleMotor() { return spindleMotor; }
    public void setSpindleMotorPower(double power) {
        setMotorPower(spindleMotor, power);
    }
    public ServoImplEx getHingeServo() { return hingeServo; }
    public void setHingeServoPosition(double position) {
        hingeServo.setPosition(position);
    }

    public void resetUpdateBlockColor() {
        updatedBlockColor = false;
    }

    @Override
    public void update(double dt) {
        resetUpdateBlockColor();
        stateManager.update(dt);
    }
    public boolean hasValidBlockColor() {
        return getBlockColor() == BlockColor.YELLOW ||
                (getBlockColor() == BlockColor.BLUE && getAllianceColor() == AllianceColor.BLUE) ||
                (getBlockColor() == BlockColor.RED && getAllianceColor() == AllianceColor.RED);
    }
    public BlockColor getBlockColor() {
        if (!updatedBlockColor) {
            updatedBlockColor = true;

            // actually find block color
            if (hasColor(RED_BLOCK_COLOR))
                blockColor = BlockColor.RED;
            else if (hasColor(BLUE_BLOCK_COLOR))
                blockColor = BlockColor.BLUE;
            else if (hasColor(YELLOW_BLOCK_COLOR))
                blockColor = BlockColor.YELLOW;
            else
                blockColor = BlockColor.NONE;
        }
        return blockColor;
    }

    private boolean hasColor(int[] color) {
        int diff = Math.abs(blockColorSensor.red() - color[0]) + Math.abs(blockColorSensor.green() - color[1]) + Math.abs(blockColorSensor.blue() - color[2]);
        return diff < MAX_COLOR_THRESHOLD;
    }
}




