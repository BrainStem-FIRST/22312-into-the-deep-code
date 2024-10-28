package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem {
    private final DcMotorEx liftMotor;
    public final int MOTOR_THRESHOLD = 10;
    public enum State {
        LOWERED, PICKUP_HEIGHT, RAM_HEIGHT, BASKET_HEIGHT
    }
    private State state = State.LOWERED;
    private State goalState = null;
    private final HashMap<State, Double> statePositions = new HashMap<>();

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeStatePositions();
    }
    private void initializeStatePositions() {
        statePositions.put(State.LOWERED, 0.0);
        statePositions.put(State.PICKUP_HEIGHT, 0.4);
        statePositions.put(State.RAM_HEIGHT, 0.8);
        statePositions.put(State.BASKET_HEIGHT, 1.0);
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
    public State getGoalState() {
        return goalState;
    }
    public void setGoalState(State goalState) {
        this.goalState = goalState;
    }
    public HashMap<State, Double> getStatePositions() {
        return statePositions;
    }
    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }
}

