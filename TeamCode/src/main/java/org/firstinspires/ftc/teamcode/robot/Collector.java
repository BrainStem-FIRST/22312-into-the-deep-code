package org.firstinspires.ftc.teamcode.robot;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectTempState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.MotorCurrentTracker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;



// state mechanics: once collector/spit state initiated, will power until time reached, them automatically turn off
// do not need to turn motor off outside of this class

// need to add color sensor; could represent the one detecting block in grabber
// the collecting sequence should end only when color sensor detects block over extended period
// collecting sequence:
// collect motor spins in, and servo slowly turns inward and rapidly resets to try and match block
// if block is good, signal for de-extension
// if block not good, spin wheels in opposite direction and run collect motor in opposite direction
// then would need to adjust robot (and potentially extension) position to search new area for block
@Config
public class Collector extends Subsystem<Collector.StateType> {

    public static int AUTO_COLOR_VALIDATION_REQUIRED = 2;
    public static double AUTO_MAX_COLLECT_TIME = 5;
    public static double TELE_COLLECT_POWER = 1, AUTO_COLLECT_POWER = 1, SPIT_POWER = -1;
    public static double COLLECT_TEMP_POWER = 0.8, SPIT_TEMP_POWER = -0.7, AUTO_JAM_SPIT_POWER = -0.4;

    // after the block color sensor stops detecting the block, still spit for 1 second
    public static double SAFETY_SPIT_TIME = 0.4;

    public enum StateType {
        NOTHING,
        COLLECTING,
        COLLECTING_TEMP, // stops next frame unless called continuously
        SPITTING, // only stops when block color sensor detects nothing + safety time
        SPITTING_TEMP, // stops next frame unless called continuously
        VALID_BLOCK
    }
    public static int TELE_JAM_CURRENT_THRESHOLD = 5000, AUTO_JAM_CURRENT_THRESHOLD = 4500;
    public static int TELE_JAM_VALIDATION_FRAMES = 5, AUTO_JAM_VALIDATION_FRAMES = 3;
    public static int TELE_JAM_SAFETY_FRAMES = 3, AUTO_JAM_SAFETY_FRAMES = 12;
    private final MotorCurrentTracker teleCurrentTracker, autoCurrentTracker;
    private final DcMotorEx spindleMotor;

    private final BlockColorSensor blockColorSensor;
    private BlockColor blockColorInTrough;
    private int autoColorValidationFrames;

    public Collector(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.NOTHING);

        spindleMotor = hwMap.get(DcMotorEx.class, "CollectSpindleMotor");
        spindleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        teleCurrentTracker = new MotorCurrentTracker(spindleMotor, TELE_JAM_CURRENT_THRESHOLD, TELE_JAM_VALIDATION_FRAMES, TELE_JAM_SAFETY_FRAMES);
        autoCurrentTracker = new MotorCurrentTracker(spindleMotor, AUTO_JAM_CURRENT_THRESHOLD, AUTO_JAM_VALIDATION_FRAMES, AUTO_JAM_SAFETY_FRAMES);

        blockColorSensor = new BlockColorSensor(hwMap, robot);
        blockColorInTrough = BlockColor.NONE;

        stateManager.addState(StateType.NOTHING, new NothingState<>(StateType.NOTHING, spindleMotor));
        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.COLLECTING_TEMP, new CollectTempState());
        stateManager.addState(StateType.SPITTING, new SpitState());
        stateManager.addState(StateType.SPITTING_TEMP, new SpitTempState());
        stateManager.addState(StateType.VALID_BLOCK, new NothingState<>(StateType.VALID_BLOCK));
        stateManager.setupStates(getRobot(), stateManager);
    }

    public MotorCurrentTracker getTeleCurrentTracker() {
        return teleCurrentTracker;
    }
    public MotorCurrentTracker getAutoCurrentTracker() {
        return autoCurrentTracker;
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
    public Action collect() {
        return new Action() {
            final ElapsedTime autoCollectTimer = new ElapsedTime();
            boolean isFirst = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (isFirst) {
                    autoCollectTimer.reset();
                    isFirst = false;
                }

                // updating motor current check
                autoCurrentTracker.updateCurrentTracking();
                if (autoCurrentTracker.hasValidatedAbnormalCurrent())
                    setSpindleMotorPower(Collector.AUTO_JAM_SPIT_POWER);
                else
                    setSpindleMotorPower(Collector.AUTO_COLLECT_POWER);

                // checking for color sensor validation
                if (blockColorSensor.getRawBlockColor() != BlockColor.NONE)
                    autoColorValidationFrames++;
                else
                    autoColorValidationFrames = 0;

                boolean end = autoCollectTimer.seconds() >= AUTO_MAX_COLLECT_TIME ||
                        (autoColorValidationFrames >= AUTO_COLOR_VALIDATION_REQUIRED
                        && blockColorSensor.getRawBlockColor() != BlockColor.NONE);

                telemetry.addData("time collecting", autoCollectTimer.seconds());
                telemetry.addData("auto max collect time", AUTO_MAX_COLLECT_TIME);
                telemetry.addData("motor current", spindleMotor.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("has validated abnormal current", autoCurrentTracker.hasValidatedAbnormalCurrent());
                telemetry.addData("motor power", spindleMotor.getPower());
                telemetry.addData("raw block color", blockColorSensor.getRawBlockColor());
                telemetry.addData("should end", end);
                telemetry.update();

                if(end)
                    Subsystem.setMotorPower(spindleMotor, 0);

                return !end;
            }
        };
    }
    /*
    public Action collect() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();

                telemetry.addData("time collecting", getTime());
                telemetry.addData("frames running", getFramesRunning());
                telemetry.addData("motor current", spindleMotor.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("has validated abnormal current", autoCurrentTracker.hasValidatedAbnormalCurrent());
                telemetry.addData("motor power", spindleMotor.getPower());
                telemetry.update();

                // updating motor current check
                autoCurrentTracker.updateCurrentTracking();
                if (autoCurrentTracker.hasValidatedAbnormalCurrent())
                    setSpindleMotorPower(Collector.SPIT_TEMP_POWER);
                else
                    setSpindleMotorPower(Collector.AUTO_COLLECT_POWER);

                // checking for color sensor validation
                if (blockColorSensor.getRawBlockColor() != BlockColor.NONE)
                    autoColorValidationFrames++;
                else
                    autoColorValidationFrames = 0;

                if (getTime() >= AUTO_MAX_COLLECT_TIME)
                    return false;

                return autoColorValidationFrames < AUTO_COLOR_VALIDATION_REQUIRED
                        && blockColorSensor.getRawBlockColor() != BlockColor.NONE;
            }
        };
    }
    */
    public Action collectUntilHardStop() {
        return telemetryPacket -> {
            setSpindleMotorPower(Collector.COLLECT_TEMP_POWER);
            return !robot.getExtension().hitRetractHardStop();
        };
    }
    public Action stopCollect() {
        return telemetryPacket -> {
            setSpindleMotorPower(0);
            return false;
        };
    }
}




