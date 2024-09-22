package org.firstinspires.ftc.teamcode.robot;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static final double HINGE_UP_POSITION = 1;
    public static final double HINGE_DOWN_POSITION = 0;
    public static final int SPITTING_FRAME_TIME = 20;
    public enum State {
        COLLECTING, SPITTING, EMPTY, FULL
    }
    private State state;
    private int spittingFrames = 0;

    private ServoImplEx hingeServos[];
    private DcMotorEx spindleMotor;
    private TouchSensor fullyHasBlockSensor;

    public Collector(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        state = State.EMPTY;

        hingeServos = new ServoImplEx[2];
        hingeServos[0] = hwMap.get(ServoImplEx.class, "CollectHingeServoLeft");
        hingeServos[1] = hwMap.get(ServoImplEx.class, "CollectHingeServoRight");

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");
        fullyHasBlockSensor = hwMap.get(TouchSensor.class, "CollectFullyTouchSensor");
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }

    public ServoImplEx[] getHingeServos() {
        return hingeServos;
    }
    public DcMotorEx getSpindleMotor() {
        return spindleMotor;
    }
    public TouchSensor getFullyCollectedSensor() {
        return fullyHasBlockSensor;
    }

    public void updateSpindleMotorState() {
        switch(state) {
            case EMPTY:
            case FULL:
                getSpindleMotor().setPower(0);
                break;
            case COLLECTING:
                getSpindleMotor().setPower(MAX_SPIN_POWER);
                break;
            case SPITTING:
                getSpindleMotor().setPower(-MAX_SPIN_POWER);
                break;
        }
    }

    public void takeInBlock() {
        setState(State.COLLECTING);
    }

    public void spitOutBlock() {
        setState(State.SPITTING);
    }

    public void stopSpindles() {
        setState(getFullyCollectedSensor().isPressed() ? State.FULL : State.EMPTY);
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




