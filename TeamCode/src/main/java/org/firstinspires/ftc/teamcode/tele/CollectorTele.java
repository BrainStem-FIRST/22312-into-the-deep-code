package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Collector;


// state mechanics: once collector/spit state initiated, will power until time reached, them automatically turn off
// do not need to turn motor off outside of this class

// need to add color sensor; could represent the one detecting block in grabber
// the collecting sequence should end only when color sensor detects block over extended period
// collecting sequence:
// collect motor spins in, and servo slowly turns inward and rapidly resets to try and match block
// if block is good, signal for de-extension
// if block not good, spin wheels in opposite direction and run collect motor in opposite direction
// then would need to adjust robot (and potentially extension) position to search new area for block

public class CollectorTele extends Collector {

    public CollectorTele(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {

        super(hwMap, telemetry, allianceColor);
        setHingeState(HingeState.UP);
        setCollectState(CollectState.EMPTY);
    }

    public void update() {
        updateHingeState();
        updateSpindleMotorState();
    }

    private void updateSpindleMotorState() {
        switch(getCollectState()) {
            case EMPTY: // could be in retracted state or after spitting
                getSpindleMotor().setPower(0);
                break;

            case FULL_SLOW: // when extension in extended/retracting and collector is hinged up with block
                getSpindleMotor().setPower(HOLD_SPIN_POWER);
                break;

            case FULL_MAX: // when extension is extended/retracting and collector is hinging up with block
                if (getHingeState() == Collector.HingeState.UP)
                    setCollectState(CollectState.FULL_SLOW);

            case COLLECTING: // when extension is extending/extended and collector is hinging/hinged down with no block
                // checking for potential end in collection
                if (getBlockColor() == BlockColor.RED)
                    setCollectState(CollectState.SPITTING);
                else if (getBlockColor() == BlockColor.BLUE || getBlockColor() == BlockColor.YELLOW) {
                    setCollectState(CollectState.FULL_MAX);
                    setHingeState(HingeState.HINGING_UP);
                } else
                    getSpindleMotor().setPower(MAX_SPIN_POWER);
                break;

            case SPITTING: // when collector has wrong block collected
                // checking for potential end of spitting
                if (getSpittingFrames() >= SPITTING_FRAME_TIME) {
                    setCollectState(CollectState.EMPTY);
                    resetSpittingFrames();
                } else {
                    incrementSpittingFrames();
                    getSpindleMotor().setPower(-MAX_SPIN_POWER);
                }
                break;
        }
    }

    private void updateHingeState() {
        switch (getHingeState()) {
            case HINGING_UP:
                setHingeServoPosition(1);
                if (getHingeServo().getPosition() >= 1)
                    setHingeState(HingeState.UP);
                break;
            case HINGING_DOWN:
                setHingeServoPosition(0);
                if (getHingeServo().getPosition() <= 0)
                    setHingeState(HingeState.DOWN);
                break;
            default: break;
        }
    }
    public void setFull() {
        setHingeState(Collector.HingeState.HINGING_UP);
        setCollectState(Collector.CollectState.FULL_MAX);
    }
    public void reset() {
        setCollectState(Collector.CollectState.EMPTY);
        setHingeState(Collector.HingeState.UP);
    }
}
