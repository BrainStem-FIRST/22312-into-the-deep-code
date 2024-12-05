package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.TimedAction;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.FindingBlockState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.JumpToMin;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension extends Subsystem {
    public static final double AUTO_SLOW_EXTEND_POWER = 0.3;

    // TODO: find extension encoder ticks for these 3
    public static final int MIN_POSITION = 5;
    // max position
    public static final int MAX_POSITION = 1880;

    public static final int MIN_SEARCH_AND_COLLECT_POSITION = 500;

    public static int SHORT_EXTEND_POSITION = 500;
    // threshold whenever extension is going to a target position
    public static int GO_TO_THRESHOLD = 10;

    public static final double SEARCH_POWER = 0.55;
    public static final double RETRACT_POWER_FAST = -1, RETRACT_POWER_SLOW = -0.6, RETRACT_POWER_IN = -0.1, RETRACT_POWER_TRANSFER = -0.2;
    public static final int RETRACT_SLOW_POSITION = 200;

    public enum StateType {
        IN, JUMP_TO_MIN, FINDING_BLOCK, RETRACTING
    }
    private final PIDController pid;
    private final DcMotorEx extensionMotor;
    private final DigitalChannel magnetResetSwitch;
    private double targetPower;


    private final StateManager<StateType> stateManager;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // magnet switch doesn't work, getState() always returns true
        magnetResetSwitch = hwMap.get(DigitalChannel.class, "ExtensionMagnetSwitch");
        magnetResetSwitch.setMode(DigitalChannel.Mode.INPUT);

        pid = new PIDController(0.01, 0, 0);
        pid.setTarget(MIN_POSITION);
        pid.setOutputBounds(-1,1);

        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.JUMP_TO_MIN, new JumpToMin());
        stateManager.addState(StateType.FINDING_BLOCK, new FindingBlockState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(getRobot(), stateManager);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public DcMotorEx getExtensionMotor() {
        return extensionMotor;
    }
    public void setExtensionMotorPosition(int position) {
        Subsystem.setMotorPosition(extensionMotor, position);
    }
    public void setExtensionMotorPower(double power) {
        Subsystem.setMotorPower(extensionMotor, power);
    }
    public void retractExtensionMotor() {
        Subsystem.setMotorPower(extensionMotor, extensionMotor.getCurrentPosition() > RETRACT_SLOW_POSITION ? RETRACT_POWER_FAST : RETRACT_POWER_SLOW);
    }

    public double getTargetPower() {
        return targetPower;
    }
    public void setTargetPower(double targetPower) {
        this.targetPower = targetPower;
    }

    public boolean isMagnetSwitchActivated() {
        return !magnetResetSwitch.getState();
    }
    public boolean hitRetractHardStop() {
        return isMagnetSwitchActivated();
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    // returns true to keep going
    public Action extendAction(int targetPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPosition(targetPosition);
                return !Subsystem.inRange(getExtensionMotor(), targetPosition, GO_TO_THRESHOLD);
                //return Math.abs(getExtensionMotor().getCurrentPosition() - targetPosition) > GO_TO_THRESHOLD;
            }
        };
    }
    public Action slowExtendAction() {
        return new SequentialAction(
                slowExtensionAction(),
                stopExtensionAction()
        );
    }
    private Action slowExtensionAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (getExtensionMotor().getCurrentPosition() < Extension.MAX_POSITION)
                    setExtensionMotorPower(Extension.AUTO_SLOW_EXTEND_POWER);
                else
                    return false;
                return getRobot().getCollector().getBlockColorSensor().getRawBlockColor() == BlockColor.NONE;
            }
        };
    }
    public Action stopExtensionAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPower(0);
                return false;
            }
        };
    }
    public Action retractAction() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();

                //if (getFramesRunning() == 1)
                //    pid.reset();
                //if(getRobot().getInPidMode())
                //    setExtensionMotorPower(pid.update(extensionMotor.getCurrentPosition()));
                //else
                //  setExtensionMotorPower(Extension.RETRACT_POWER_FAST);
                retractExtensionMotor();

                if (hitRetractHardStop()) {
                    setExtensionMotorPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    public PIDController getPid() {
        return pid;
    }
}