package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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

    public CollectorTele(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
    }

    public void update() {

        // stop motor once you have finished intaking block
        if (getFullyCollectedSensor().isPressed()) {
            setState(State.FULL);
        }
        // spit out blocks for certain amount of time
        if (getState() == State.SPITTING) {
            if (getSpittingFrames() >= SPITTING_FRAME_TIME) setState(State.EMPTY);
             else incrementSpittingFrames();
        }
        else resetSpittingFrames();
        
        updateSpindleMotorState();
    }
}




