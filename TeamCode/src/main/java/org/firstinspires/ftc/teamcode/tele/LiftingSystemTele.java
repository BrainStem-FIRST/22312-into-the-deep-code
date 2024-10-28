package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;

public class LiftingSystemTele {
    private LiftTele lift;
    private ArmTele arm;
    private GrabberTele grabber;

    public enum State {
        LOWERED, MID_DROP, MID_RAM, HIGH_DROP
    }
    public State state = State.LOWERED;

    public LiftingSystemTele(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        lift = new LiftTele(hw, telemetry, allianceColor);
        arm = new ArmTele(hw, telemetry, allianceColor);
        grabber = new GrabberTele(hw, telemetry, allianceColor);
    }

    public void update() {
        lift.update();
        arm.update();
        grabber.update();
    }




    // trough pick up functions
    public void preparePickupBlockFromTrough() {
        lift.setGoalState(Lift.State.LOWERED);
        arm.setGoalState(Arm.State.DOWN);
        grabber.setGoalState(Grabber.State.OPEN);
    }
    // used to actually pick up block from trough
    public void pickupBlockFromTrough() {
        grabber.setGoalState(Grabber.State.CLOSED);
    }

    // human player pick up functions
    public void prepareHumanPlayerPickup() {
        lift.setGoalState(Lift.State.PICKUP_HEIGHT);
        arm.setGoalState(Arm.State.AT_PICKUP);
        grabber.setGoalState(Grabber.State.OPEN);
    }
    public void pickupFromHumanPlayer() {
        grabber.setGoalState(Grabber.State.CLOSED);
    }

    // specimen ram functions
    public void prepareSpecimenRam() {
        arm.setGoalState(Arm.State.AT_HARD_STOP);
    }
    public void ramSpecimen() {
        lift.setGoalState(Lift.State.RAM_HEIGHT);
    }
    public void releaseSpecimen() {
        grabber.setGoalState(Grabber.State.OPEN);
    }

    // basket deposit functions
    public void prepareDepositInBasket() {
        lift.setGoalState(Lift.State.BASKET_HEIGHT);
        arm.setGoalState(Arm.State.AT_PICKUP);
    }
    public void depositInBasket() {
        grabber.setGoalState(Grabber.State.OPEN);
    }
}
