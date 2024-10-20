package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;
import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


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

    public static final double MAX_SPIN_POWER = 1;
    public static final double HOLD_SPIN_POWER = 0.3;
    // TODO: FIND SERVO POSITIONS
    public static final double HINGE_UP_POSITION = 1300;
    public static final double HINGE_DOWN_POSITION = 500;
    // number of frames on REV hub to spit out for
    public static final int SPITTING_FRAME_TIME = 20;
    public enum CollectState {
        COLLECTING, SPITTING, EMPTY, FULL_SLOW, FULL_MAX
    }

    public enum HingeState {
        HINGING_UP,
        HINGING_DOWN,
        UP,
        DOWN
    }
    public static enum BlockColor {
        RED,
        YELLOW,
        BLUE,
        NONE
    }
    final public static int MAX_COLOR_THRESHOLD = 20;
    final public static int[] RED_BLOCK_COLOR = { 255, 0, 0 };
    final public static int[] YELLOW_BLOCK_COLOR = { 255, 255, 0 };
    final public static int[] BLUE_BLOCK_COLOR = { 0, 0, 255 };
    private CollectState collectState;
    private HingeState hingeState;
    private int spittingFrames = 0;

    private ServoImplEx hingeServo;
    private DcMotorEx spindleMotor;

    // IN PROGRESS: replace touch sensor w color sensor and implement spitting state
    private ColorSensor blockColorSensor;
    private boolean updatedBlockColor;
    private BlockColor blockColor;

    public Collector(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        collectState = CollectState.EMPTY;
        hingeServo = hwMap.get(ServoImplEx.class, "CollectHingeServo");

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");
        blockColorSensor = hwMap.get(ColorSensor.class, "BlockColorSensor");
        updatedBlockColor = false;
        blockColor = BlockColor.NONE;

        hingeServo.setPwmRange(new PwmControl.PwmRange(HINGE_DOWN_POSITION, HINGE_UP_POSITION));
    }

    public CollectState getCollectState() {
        return collectState;
    }
    public void setCollectState(CollectState collectState) {
        this.collectState = collectState;
    }

    public HingeState getHingeState() { return hingeState; }
    public void setHingeState(HingeState hingeState) { this.hingeState = hingeState; }

    public ServoImplEx getHingeServo() {
        return hingeServo;
    }

    public void setHingeServoPosition(double position) {
        hingeServo.setPosition(position);
    }
    public DcMotorEx getSpindleMotor() {
        return spindleMotor;
    }

    public void resetUpdateBlockColor() {
        updatedBlockColor = false;
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

    public int getSpittingFrames() {
        return spittingFrames;
    }
    public void resetSpittingFrames() {
        spittingFrames = 0;
    }
    public void incrementSpittingFrames() {
        spittingFrames++;
    }
}




