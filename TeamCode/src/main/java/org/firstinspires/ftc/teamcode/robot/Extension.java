package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.FindingBlockState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension extends Subsystem {

    // TODO: find extension encoder ticks for these 3
    public static final int MIN_POSITION = 10;
    // max position
    public static final int MAX_POSITION = 1400;

    // threshold whenever extension is going to a target position
    public static int GO_TO_THRESHOLD = 10;

    // TODO - implement magnet sensor and encoder reset
    public static final double SEARCH_POWER = 0.55;
    public static final double RETRACT_POWER = -0.9;

    public enum StateType {
        IN, FINDING_BLOCK, RETRACTING
    }

    private final DcMotorEx extensionMotor;
    private final DigitalChannel magnetResetSwitch;
    private double targetPower;

    private final StateManager<StateType> stateManager;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        magnetResetSwitch = hwMap.get(DigitalChannel.class, "ExtensionMagnetSwitch");
        magnetResetSwitch.setMode(DigitalChannel.Mode.INPUT);

        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
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
    public DigitalChannel getMagnetResetSwitch() {
        return magnetResetSwitch;
    }
    public double getTargetPower() {
        return targetPower;
    }
    public void setTargetPower(double targetPower) {
        this.targetPower = targetPower;
    }

    public boolean hitRetractHardStop() {
        return !magnetResetSwitch.getState() || extensionMotor.getCurrentPosition() < MIN_POSITION;
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public Action extendAction(int targetPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPosition(targetPosition);
                return Math.abs(getExtensionMotor().getCurrentPosition() - targetPosition) > GO_TO_THRESHOLD;
            }
        };
    }
    public Action retractAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPower(Extension.RETRACT_POWER);
                if (!getMagnetResetSwitch().getState()) {
                    setExtensionMotorPower(0);
                    return false;
                }
                return true;
            }
        };
    }
}