package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Extension extends Subsystem<Extension.StateType> {
    // TODO: find extension encoder ticks for these 3
    public static int MIN_POSITION = 5,
            RETRACT_SLOW_POSITION = 120,
            MIN_SEARCH_AND_COLLECT_POSITION = 450,
            MAX_POSITION = 1880;

    public static int GO_TO_THRESHOLD = 10, // if extension going to target position and is within this threshold of distance, from it, we consider the transition complete
            EXTRA_MIN_SAFETY_DIST = GO_TO_THRESHOLD + 10;  // extra distance that extension goes to in short extend to ensure you can hinge down straight after

    public static double SEARCH_POWER = 0.6,
            RETRACT_POWER_FAST = -1,
            RETRACT_POWER_SLOW = -0.7,
            RETRACT_POWER_IN = -0.3,
            TRANSFER_POWER_IN = -0.2;

    public enum StateType {
        IN, JUMP_TO_MIN, FINDING_BLOCK, RETRACTING
    }
    private final DcMotorEx extensionMotor;
    private final PIDController pid;
    private final DigitalChannel magnetResetSwitch;
    private double targetPower;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.IN);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // magnet switch doesn't work, getState() always returns true
        magnetResetSwitch = hwMap.get(DigitalChannel.class, "ExtensionMagnetSwitch");
        magnetResetSwitch.setMode(DigitalChannel.Mode.INPUT);

        pid = new PIDController(0.01, 0, 0);
        pid.setTarget(MIN_POSITION);
        pid.setOutputBounds(-1,1);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.JUMP_TO_MIN, new JumpToMin());
        stateManager.addState(StateType.FINDING_BLOCK, new FindingBlockState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(getRobot(), stateManager);
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
        targetPower = extensionMotor.getCurrentPosition() > RETRACT_SLOW_POSITION ? RETRACT_POWER_FAST : RETRACT_POWER_SLOW;
        Subsystem.setMotorPower(extensionMotor, targetPower);
    }

    public double getTargetPower() {
        return targetPower;
    }
    public void setTargetPower(double targetPower) {
        this.targetPower = targetPower;
    }
    public DigitalChannel getMagnetSwitch() {
        return magnetResetSwitch;
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
    public Action retractContinuously() {
        return telemetryPacket -> {
            if (!hitRetractHardStop())
                retractExtensionMotor();
            else
                setExtensionMotorPower(0);
            return false;
        };
    }

    public PIDController getPid() {
        return pid;
    }
}